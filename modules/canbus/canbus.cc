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

#include "modules/canbus/canbus.h"
#define _GNU_SOURCE
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/vehicle/vehicle_factory.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"

#include <sensor_msgs/Imu.h>
#include "modules/common/time/time_util.h"
namespace jmc_auto
{
  namespace canbus
  {

    using jmc_auto::common::ErrorCode;
    using jmc_auto::common::Status;
    using jmc_auto::common::adapter::AdapterManager;
    using jmc_auto::common::time::Clock;
    using jmc_auto::control::ControlCommand;
    using jmc_auto::drivers::canbus::CanClientFactory;
    // using jmc_auto::guardian::GuardianCommand;

    std::string Canbus::Name() const { return FLAGS_canbus_module_name; }

    Status Canbus::Init()
    {
      cpu_set_t mask;
      CPU_ZERO(&mask);
      CPU_SET(7, &mask);
      sched_setaffinity(0, sizeof(cpu_set_t), &mask);
      AdapterManager::Init(FLAGS_canbus_adapter_config_filename);
      AINFO << "The adapter manager is successfully initialized.";

      // load conf
      if (!common::util::GetProtoFromFile(FLAGS_canbus_conf_file, &canbus_conf_))
      {
        return OnError("Unable to load canbus conf file: " +
                       FLAGS_canbus_conf_file);
      }

      AINFO << "The canbus conf file is loaded: " << FLAGS_canbus_conf_file;
      ADEBUG << "Canbus_conf:" << canbus_conf_.ShortDebugString();

      // Init can client
      auto *can_factory = CanClientFactory::instance();
      can_factory->RegisterCanClients();
      can_client_ = can_factory->CreateCANClient(canbus_conf_.can_card_parameter());
      if (!can_client_)
      {
        return OnError("Failed to create can client.");
      }
      AINFO << "Can client is successfully created.";

      VehicleFactory vehicle_factory;
      vehicle_factory.RegisterVehicleFactory();
      auto vehicle_object =
          vehicle_factory.CreateVehicle(canbus_conf_.vehicle_parameter());
      if (!vehicle_object)
      {
        return OnError("Failed to create vehicle:");
      }

      message_manager_ = vehicle_object->CreateMessageManager();
      if (message_manager_ == nullptr)
      {
        return OnError("Failed to create message manager.");
      }
      AINFO << "Message manager is successfully created.";

      if (can_receiver_.Init(can_client_.get(), message_manager_.get(),
                             canbus_conf_.enable_receiver_log()) != ErrorCode::OK)
      {
        return OnError("Failed to init can receiver.");
      }
      AINFO << "The can receiver is successfully initialized.";

      if (can_sender_.Init(can_client_.get(), canbus_conf_.enable_sender_log()) !=
          ErrorCode::OK)
      {
        return OnError("Failed to init can sender.");
      }
      AINFO << "The can sender is successfully initialized.";

      vehicle_controller_ = vehicle_object->CreateVehicleController();
      if (vehicle_controller_ == nullptr)
      {
        return OnError("Failed to create vehicle controller.");
      }
      AINFO << "The vehicle controller is successfully created.";

      if (vehicle_controller_->Init(canbus_conf_.vehicle_parameter(), &can_sender_,
                                    message_manager_.get()) != ErrorCode::OK)
      {
        return OnError("Failed to init vehicle controller.");
      }
      AINFO << "The vehicle controller is successfully initialized.";

      CHECK(AdapterManager::GetControlCommand()) << "Control is not initialized.";
      // CHECK(AdapterManager::GetGuardian()) << "Guardian is not initialized.";
      // TODO(QiL) : depreacte this
      if (!FLAGS_receive_guardian)
      {
        // AdapterManager::AddControlCommandCallback(&Canbus::OnControlCommand, this);
        AdapterManager::AddControlCommandCallback(&Canbus::OnControlCommand, this);
        AdapterManager::AddRemoteControlCallback(&Canbus::OnRemoteControlCommand, this);
      }
      else
      {
        // AdapterManager::AddGuardianCallback(&Canbus::OnGuardianCommand, this);
      }

      return Status::OK();
    }

    Status Canbus::Start()
    {
      // 1. init and start the can card hardware
      if (can_client_->Start() != ErrorCode::OK)
      {
        return OnError("Failed to start can client");
      }
      AINFO << "Can client is started.";

      // 2. start receive first then send
      if (can_receiver_.Start() != ErrorCode::OK)
      {
        return OnError("Failed to start can receiver.");
      }
      AINFO << "Can receiver is started.";

      // 3. start send
      if (can_sender_.Start() != ErrorCode::OK)
      {
        return OnError("Failed to start can sender.");
      }

      // 4. start controller
      if (vehicle_controller_->Start() == false)
      {
        return OnError("Failed to start vehicle controller.");
      }

      // 5. set timer to triger publish info periodly
      const double duration = 1.0 / FLAGS_chassis_freq;
      timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                           &Canbus::OnTimer, this);
      // sent_cmd_timer_ = AdapterManager::CreateTimer(ros::Duration(0.01),
      //                                               &Canbus::setControlcmd, this);
      // last step: publish monitor messages
      jmc_auto::common::monitor::MonitorLogBuffer buffer(&monitorger_);
      buffer.INFO("Canbus is started.");

      return Status::OK();
    }

    void Canbus::PublishChassis()
    {
      Chassis chassis = vehicle_controller_->chassis();
      AdapterManager::FillChassisHeader(FLAGS_canbus_node_name, &chassis);
      AdapterManager::PublishChassis(chassis);
      //发送ros格式的IMU消息
      sensor_msgs::Imu imu_data;
      // sensor_msgs::Imu imu_data;
      //imu_data.header.stamp =  chassis.ins_time();
      //imu_data.header.stamp = ros::Time::now();
      AINFO<<"now time "<<ros::Time::now();
      //获取imu的时间戳
      
      double imu_measure=jmc_auto::common::time::TimeUtil::Gpsweek2unix(chassis.ins_time()/1000,chassis.header().timestamp_sec());
      AINFO<<"imu measure time1 "<<imu_measure;
      imu_data.header.stamp.sec=int(imu_measure);
      imu_data.header.stamp.nsec=(imu_measure-imu_data.header.stamp.sec)*1e+9;
      // Unix2gps(chassis.header().timestamp_sec());
      imu_data.header.frame_id = "imu_frame_id";
      imu_data.header.seq = chassis.header().sequence_num();
      imu_data.linear_acceleration.x=chassis.acc_x();
      imu_data.linear_acceleration.y=chassis.acc_y();
      imu_data.linear_acceleration.z=chassis.acc_z();
      imu_data.angular_velocity.x=chassis.gyro_x();
      imu_data.angular_velocity.y=chassis.gyro_y();
      imu_data.angular_velocity.z=chassis.gyro_z();
      AdapterManager::PublishRosImu(imu_data);
      // imu_data.header.frame_id = "base_link";
      // imu_data.orientation.x = q3;
      // imu_data.orientation.y = -q2;
      // imu_data.orientation.z = -q1;
      // imu_data.orientation.w = q4;
      // if (chassis.driving_mode()== Chassis::AUTO_SPEED_ONLY)
      // {
      //   IS_STOP_MODE = true;
      // }
      // else
      // {
      //   IS_STOP_MODE = false;
      // }
      // if (chassis.speed_mps() ==0)
      // {
      //   IS_VEHCILE_STOP = true;
      // }
      // else
      // {
      //   IS_VEHCILE_STOP = false;
      // }
      AINFO << chassis.DebugString();
      ADEBUG << chassis.ShortDebugString();
    }

    void Canbus::PublishChassisDetail()
    {
      ChassisDetail chassis_detail;
      message_manager_->GetSensorData(&chassis_detail);
      ADEBUG << chassis_detail.ShortDebugString();

      AdapterManager::PublishChassisDetail(chassis_detail);
    }

    void Canbus::OnTimer(const ros::TimerEvent &)
    {
      PublishChassis();
      if (FLAGS_enable_chassis_detail_pub)
      {
        PublishChassisDetail();
      }
    }

    void Canbus::Stop()
    {
      timer_.stop();
      can_sender_.Stop();
      can_receiver_.Stop();
      can_client_->Stop();
      vehicle_controller_->Stop();
    }
    ControlCommand Canbus::RemoteCmdToControlCmd(const jmc_auto::remote::RemoteControl &RemoteControlCommand)
    {
      ControlCommand control_command;
      if (RemoteControlCommand.has_pedal_throttle_percent())
      {
        control_command.set_throttle(RemoteControlCommand.pedal_throttle_percent());
      }
      if (RemoteControlCommand.has_pedal_brake_percent())
      {
        control_command.set_brake(RemoteControlCommand.pedal_brake_percent());
      }
      if (RemoteControlCommand.has_steerwheel_angle())
      {
        control_command.set_steering_target(RemoteControlCommand.steerwheel_angle());
      }

      if (RemoteControlCommand.has_gear_data())
      {
        control_command.set_gear_location(RemoteControlCommand.gear_data());
      }

      if (RemoteControlCommand.has_mode_apply())
      {
        control_command.set_driving_mode(RemoteControlCommand.mode_apply());
      }

      if (RemoteControlCommand.emergency_stop())
      {
        control_command.set_driving_mode(Chassis::DrivingMode::Chassis_DrivingMode_AUTO_SPEED_ONLY);
      }

      return control_command;
    }
    void Canbus::OnRemoteControlCommand(const jmc_auto::remote::RemoteControl  &RemoteControlCommand)
    {
      AINFO << "RemoteControlCommand:" + RemoteControlCommand.DebugString();
      // int64_t current_timestamp =
      //     jmc_auto::common::time::AsInt64<common::time::micros>(Clock::Now());
      // if command coming too soon, just ignore it.
      // if (current_timestamp - last_timestamp_ < FLAGS_min_cmd_interval * 1000) {
      //   ADEBUG << "Control command comes too soon. Ignore.\n Required "
      //             "FLAGS_min_cmd_interval["
      //          << FLAGS_min_cmd_interval << "], actual time interval["
      //          << current_timestamp - last_timestamp_ << "].";
      //   return;
      // }

      // last_timestamp_ = current_timestamp;
      // ADEBUG << "Control_sequence_number:"
      //        << control_command.header().sequence_num() << ", Time_of_delay:"
      //        << current_timestamp - control_command.header().timestamp_sec();
     
      ControlCommand control_command = RemoteCmdToControlCmd(RemoteControlCommand);

      AINFO<<"control_command:"<<control_command.DebugString();
      if (control_command.has_driving_mode())
      {
        if (control_command.driving_mode() == Chassis::COMPLETE_AUTO_DRIVE)
        {
          AINFO<<"NO REMOTE MODE";
          IS_Remote_MODE = false;
        }
        else
        {
          AINFO<<"REMOTE MODE";
          IS_Remote_MODE = true;
        }
      }
      
      

      if (IS_Remote_MODE && vehicle_controller_->chassis().driving_mode() != Chassis::COMPLETE_AUTO_DRIVE)
      {
        // setControlcmd();
        AINFO<<"REMOTE CONTROL";
        if (vehicle_controller_->Update(control_command) != ErrorCode::OK)
        {
          AERROR << "Failed to process callback function OnControlCommand because "
                    "vehicle_controller_->Update error.";
          return;
        }
        //can_sender_.Update();
      }
    }
    void Canbus::OnControlCommand(const ControlCommand &control_command)
    {
      AINFO << "control_command:" + control_command.DebugString();
      int64_t current_timestamp =
          jmc_auto::common::time::AsInt64<common::time::micros>(Clock::Now());
      // if command coming too soon, just ignore it.
      if (current_timestamp - last_timestamp_ < FLAGS_min_cmd_interval * 1000)
      {
        ADEBUG << "Control command comes too soon. Ignore.\n Required "
                  "FLAGS_min_cmd_interval["
               << FLAGS_min_cmd_interval << "], actual time interval["
               << current_timestamp - last_timestamp_ << "].";
        return;
      }

      last_timestamp_ = current_timestamp;
      ADEBUG << "Control_sequence_number:"
             << control_command.header().sequence_num() << ", Time_of_delay:"
             << current_timestamp - control_command.header().timestamp_sec();

      //if (!IS_Remote_MODE && (vehicle_controller_->chassis().driving_mode() == Chassis::COMPLETE_AUTO_DRIVE||vehicle_controller_->chassis().driving_mode() == Chassis::COMPLETE_MANUAL||vehicle_controller_->chassis().driving_mode() == Chassis::AUTO_SPEED_ONLY))
      {
        if (vehicle_controller_->Update(control_command) != ErrorCode::OK)
        {
          AERROR << "Failed to process callback function OnControlCommand because "
                    "vehicle_controller_->Update error.";
          return;
        }

        can_sender_.Update();
      }
    }

    // void Canbus::OnGuardianCommand(const GuardianCommand &guardian_command) {
    //   jmc_auto::control::ControlCommand control_command;
    //   control_command.CopyFrom(guardian_command.control_command());
    //   OnControlCommand(control_command);
    // }

    // Send the error to monitor and return it
    Status Canbus::OnError(const std::string &error_msg)
    {
      jmc_auto::common::monitor::MonitorLogBuffer buffer(&monitorger_);
      buffer.ERROR(error_msg);
      return Status(ErrorCode::CANBUS_ERROR, error_msg);
    }

  } // namespace canbus
} // namespace jmc_auto
