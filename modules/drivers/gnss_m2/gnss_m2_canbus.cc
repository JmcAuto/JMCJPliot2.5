/******************************************************************************
 * Copyright 2017 The JmcAuto Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 */

#include "modules/drivers/gnss_m2/gnss_m2_canbus.h"
#include "modules/drivers/gnss_m2/gnss_m2_message_manager.h"
#include "modules/drivers/gnss_m2/proto/m2.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss_m2/protocol/odometer_input_99.h"



/**
 * @namespace jmc_auto::drivers::conti_radar
 * @brief jmc_auto::drivers
 */
namespace jmc_auto {
namespace drivers {
namespace gnss_m2 {

const double g=9.80144145;
const double PI=3.1415926535897932;
const double deg2radians=PI/180;
std::string GnssM2Canbus::Name() const {
  return FLAGS_m2_driver_name;
}

jmc_auto::common::Status GnssM2Canbus::Init() {
  AdapterManager::Init(FLAGS_m2_adapter_config_filename);
  AINFO << "The adapter manager is successfully initialized.";
  if (!::jmc_auto::common::util::GetProtoFromFile(FLAGS_m2_sensor_conf_file,
                                                &gnss_m2_conf_)) {
    return OnError("Unable to load canbus conf file: " +
                   FLAGS_m2_sensor_conf_file);
  }

  AINFO << "The canbus conf file is loaded: " << FLAGS_m2_sensor_conf_file;
  ADEBUG << "Canbus_conf:" << gnss_m2_conf_.ShortDebugString();

  // Init can client
  auto *can_factory = CanClientFactory::instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(
      gnss_m2_conf_.can_card_parameter());
  if (!can_client_) {
    return OnError("Failed to create can client.");
  }
  AINFO << "Can client is successfully created.";

  sensor_message_manager_ =
      std::unique_ptr<GnssM2MessageManager>(new GnssM2MessageManager());
  if (sensor_message_manager_ == nullptr) {
    return OnError("Failed to create message manager.");
  }
  AINFO << "Sensor message manager is successfully created.";

  if (can_receiver_.Init(can_client_.get(), sensor_message_manager_.get(),
                         gnss_m2_conf_.enable_receiver_log()) !=
      ErrorCode::OK) {
    return OnError("Failed to init can receiver.");
  }
  AINFO << "The can receiver is successfully initialized.";

  if (can_sender_.Init(can_client_.get(), gnss_m2_conf_.enable_receiver_log()) !=
      ErrorCode::OK) {
    return OnError("Failed to init can sender.");
  }
  AINFO << "The can sender is successfully initialized.";

  odometer_input_99_ = dynamic_cast<Odometerinput99 *>(sensor_message_manager_->GetMutableProtocolDataById(Odometerinput99::ID));
  if (odometer_input_99_ == nullptr)
  {
    AERROR << "Odometer_input_99 does not exist in the gnss_m2_MessageManager!";
    return OnError("Odometer_input_99 does not exist in the gnss_m2_MessageManager.");;
  }
  can_sender_.AddMessage(Odometerinput99::ID, odometer_input_99_,false);
 

  return Status::OK();
}


jmc_auto::common::Status GnssM2Canbus::Start() {
  // 1. init and start the can card hardware
  if (can_client_->Start() != ErrorCode::OK) {
    return OnError("Failed to start can client");
  }
  AINFO << "Can client is started.";
  // 2. start receive first then send
  if (can_receiver_.Start() != ErrorCode::OK) {
    return OnError("Failed to start can receiver.");
  }
  AINFO << "Can receiver is started.";

  if (can_sender_.Start() != ErrorCode::OK)
  {
    return OnError("Failed to start can sender.");
  }
  AINFO << "Can sender is started.";
  const double duration = 1.0 / FLAGS_m2_sensor_freq;
  timer_ = AdapterManager::CreateTimer(
        ros::Duration(duration), &GnssM2Canbus::OnTimer, this);
  // last step: publish monitor messages
  jmc_auto::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("Canbus is started.");

  return Status::OK();
}

void GnssM2Canbus::Stop() {
  timer_.stop();

  can_receiver_.Stop();
  can_client_->Stop();
}

void GnssM2Canbus::OnTimer(const ros::TimerEvent &event){
 //AINFO << "Can receiver is started.";
  PublishSensorData();
}

void GnssM2Canbus::OnChassis(const jmc_auto::canbus::Chassis &chassis)
{
  if (chassis.has_speed_mps() && chassis.has_gear_location())
  {
    odometer_input_99_->set_vehspd_input(chassis.speed_mps() / 1000 * 3600);
    if (chassis.gear_location() == jmc_auto::canbus::Chassis::GEAR_DRIVE)
    {
      odometer_input_99_->set_gearposition_input(0); //前进
    }
    else if (chassis.gear_location() == jmc_auto::canbus::Chassis::GEAR_REVERSE)
    {
      odometer_input_99_->set_gearposition_input(1); //后退
    }
    else if (chassis.gear_location() == jmc_auto::canbus::Chassis::GEAR_NEUTRAL || chassis.gear_location() == jmc_auto::canbus::Chassis::GEAR_PARKING)
    {
      odometer_input_99_->set_gearposition_input(2); //停止
    }
    else
    {
      AERROR<<"无效的档位";
      // AERROR << "无效的档位"；
    }
  }
  else
  {
    AERROR<<"没有车速/档位信号";
  }

}

void GnssM2Canbus::PublishSensorData() {
  jmc_auto::drivers::gnss_m2::M2 gnss_m2;
  sensor_message_manager_->GetSensorData(&gnss_m2);
  jmc_auto::drivers::gnss::Imu imu;
  jmc_auto::drivers::gnss::Ins ins;
  jmc_auto::drivers::gnss::GnssBestPose gnssbestpose;

  //赋值给imu
  if (gnss_m2.has_imu_1_721())
  {
    imu.mutable_linear_acceleration()->set_x(gnss_m2.imu_1_721().accel_x()*g);
    imu.mutable_linear_acceleration()->set_y(gnss_m2.imu_1_721().accel_y()*g);
    //ins中赋值
    ins.mutable_linear_acceleration()->set_x(gnss_m2.imu_1_721().accel_x()*g);
    ins.mutable_linear_acceleration()->set_y(gnss_m2.imu_1_721().accel_y()*g);
  }
  if (gnss_m2.has_imu_2_722())
  {
    imu.mutable_linear_acceleration()->set_z(gnss_m2.imu_2_722().accel_z()*g);
    imu.mutable_angular_velocity()->set_x(gnss_m2.imu_2_722().gyro_x()*deg2radians);
    //ins
    ins.mutable_linear_acceleration()->set_z(gnss_m2.imu_2_722().accel_z()*g);
    ins.mutable_angular_velocity()->set_x(gnss_m2.imu_2_722().gyro_x()*deg2radians);
  }
  if (gnss_m2.has_imu_3_723())
  {
    imu.mutable_angular_velocity()->set_z(gnss_m2.imu_3_723().gyro_z()*deg2radians);
    imu.mutable_angular_velocity()->set_y(gnss_m2.imu_3_723().gyro_y()*deg2radians);
    //ins
    ins.mutable_angular_velocity()->set_z(gnss_m2.imu_3_723().gyro_z()*deg2radians);
    ins.mutable_angular_velocity()->set_y(gnss_m2.imu_3_723().gyro_y()*deg2radians);
  }
  //赋值给INS
  
  if (gnss_m2.has_combinealtitude_704())
  {
    ins.mutable_position()->set_height(gnss_m2.combinealtitude_704().altitude());
  }
  if (gnss_m2.has_combineposition_703())
  {
    ins.mutable_position()->set_lon(gnss_m2.combineposition_703().longitude());
    ins.mutable_position()->set_lat(gnss_m2.combineposition_703().latitude());
  }
  if (gnss_m2.has_combinevelocity_705())
  {
    ins.mutable_linear_velocity()->set_x(gnss_m2.combinevelocity_705().velocityeast());
    ins.mutable_linear_velocity()->set_y(gnss_m2.combinevelocity_705().velocitynorth());
    ins.mutable_linear_velocity()->set_z(gnss_m2.combinevelocity_705().velocityup());
  }
  if (gnss_m2.has_combineattitude_706())
  {
    ins.mutable_euler_angles()->set_x(gnss_m2.combineattitude_706().roll()*deg2radians);
    ins.mutable_euler_angles()->set_y(gnss_m2.combineattitude_706().pitch()*deg2radians);
    ins.mutable_euler_angles()->set_z(gnss_m2.combineattitude_706().yaw()*deg2radians);
  }
  // if (gnss_m2.has_stdposition_707())
  // {
  //   float *temp;

  //   ins.position_covariance().set_x(gnss_m2.combineattitude_706().roll()*deg2radians);
   
  // }
  if (gnss_m2.has_combinestatus_710())
  {
    if (gnss_m2.combinestatus_710().gpsflag()==64&&gnss_m2.combinestatus_710().navflag()==11)
    {
      ins.set_type(jmc_auto::drivers::gnss::Ins::GOOD);
    }
    else if(gnss_m2.combinestatus_710().navflag()==15||gnss_m2.combinestatus_710().navflag()==0||gnss_m2.combinestatus_710().navflag()==1||gnss_m2.combinestatus_710().navflag()==2){
      ins.set_type(jmc_auto::drivers::gnss::Ins::INVALID);
    }
    else{
      ins.set_type(jmc_auto::drivers::gnss::Ins::CONVERGING);
    }
    
  }
  //赋值给gps
  
  
  AdapterManager::FillRawImuHeader(FLAGS_sensor_node_name, &imu);
  AdapterManager::PublishRawImu(imu);

  AdapterManager::FillInsHeader(FLAGS_sensor_node_name, &ins);
  AdapterManager::PublishIns(ins);
}

 
// Send the error to monitor and return it
Status GnssM2Canbus::OnError(const std::string &error_msg) {
  jmc_auto::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.ERROR(error_msg);
  return Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace conti_radar
}  // namespace drivers
}  // namespace jmc_auto
