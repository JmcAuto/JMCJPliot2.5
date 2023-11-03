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

#include "modules/drivers/radar/tte_conti_radar/conti_radar_canbus.h"
#include "modules/drivers/radar/tte_conti_radar/conti_radar_message_manager.h"
#include "modules/drivers/proto/tte_conti_radar.pb.h"



/**
 * @namespace jmc_auto::drivers::conti_radar
 * @brief jmc_auto::drivers
 */
namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

std::string TteContiRadarCanbus::Name() const {
  return FLAGS_tte_driver_name;
}

jmc_auto::common::Status TteContiRadarCanbus::Init() {
  AdapterManager::Init(FLAGS_tte_adapter_config_filename);
  AINFO << "The adapter manager is successfully initialized.";
  if (!::jmc_auto::common::util::GetProtoFromFile(FLAGS_tte_sensor_conf_file,
                                                &conti_radar_conf_)) {
    return OnError("Unable to load canbus conf file: " +
                   FLAGS_tte_sensor_conf_file);
  }

  AINFO << "The canbus conf file is loaded: " << FLAGS_tte_sensor_conf_file;
  ADEBUG << "Canbus_conf:" << conti_radar_conf_.ShortDebugString();

  // Init can client
  auto *can_factory = CanClientFactory::instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(
      conti_radar_conf_.can_card_parameter());
  if (!can_client_) {
    return OnError("Failed to create can client.");
  }
  AINFO << "Can client is successfully created.";

  sensor_message_manager_ =
      std::unique_ptr<ContiRadarMessageManager>(new ContiRadarMessageManager());
  if (sensor_message_manager_ == nullptr) {
    return OnError("Failed to create message manager.");
  }
  AINFO << "Sensor message manager is successfully created.";

  if (can_receiver_.Init(can_client_.get(), sensor_message_manager_.get(),
                         conti_radar_conf_.enable_receiver_log()) !=
      ErrorCode::OK) {
    return OnError("Failed to init can receiver.");
  }
  AINFO << "The can receiver is successfully initialized.";

  return Status::OK();
}


jmc_auto::common::Status TteContiRadarCanbus::Start() {
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
  const double duration = 1.0 / FLAGS_tte_sensor_freq;
  timer_ = AdapterManager::CreateTimer(
        ros::Duration(duration), &TteContiRadarCanbus::OnTimer, this);
  // last step: publish monitor messages
  jmc_auto::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("Canbus is started.");

  return Status::OK();
}

void TteContiRadarCanbus::Stop() {
  timer_.stop();

  can_receiver_.Stop();
  can_client_->Stop();
}

void TteContiRadarCanbus::OnTimer(const ros::TimerEvent &event){
 //AINFO << "Can receiver is started.";
  PublishSensorData();
}

void TteContiRadarCanbus::PublishSensorData() {
  TteContiRadar conti_radar;
  sensor_message_manager_->GetSensorData(&conti_radar);
  //TteContiRadar temp_conti_radar=conti_radar;
  //TteContiRadar temp_conti_radar=const_cast<TteContiRadar>(conti_radar);
  //Debug_leftusslot_ptab_469 a=conti_radar.debug_leftusslot_ptab_469();
  // double temp=0；
  // temp_conti_radar.debug_apafrontdistanceinfo_458().set_apafrm_distance(temp);
  //temp_conti_radar.mutable_debug_leftusslot_ptab_469()->CopyFrom(conti_radar.debug_leftusslot_ptab_469());
  //temp_conti_radar.set_debug_apafrontdistanceinfo_458(conti_radar.debug_apafrontdistanceinfo_458());
  AINFO << conti_radar.ShortDebugString();
  //AERROR << FLAGS_apafrm_distance_outliner;
  if (conti_radar.has_debug_apafrontdistanceinfo_458())
  {
    double temp=conti_radar.mutable_debug_apafrontdistanceinfo_458()->apafrm_distance();
    //apafrm_distance.startFliter(temp);
    conti_radar.mutable_debug_apafrontdistanceinfo_458()->set_apafrm_distance(apafrm_distance.startFliter(temp));

    temp=conti_radar.mutable_debug_apafrontdistanceinfo_458()->apafrs_distance();
	AINFO<<"apafrs_distance"<<temp;
    //apafrs_distance.startFliter(temp);
    conti_radar.mutable_debug_apafrontdistanceinfo_458()->set_apafrs_distance(apafrs_distance.startFliter(temp));

    temp=conti_radar.mutable_debug_apafrontdistanceinfo_458()->apafr_distance();
    AINFO<<"apafr_distance"<<temp;
	//apafr_distance.startFliter(temp);
    conti_radar.mutable_debug_apafrontdistanceinfo_458()->set_apafr_distance(apafr_distance.startFliter(temp));
	AINFO<<"apafr_distance2"<<temp;
	AINFO<<"apafr_distance3"<<conti_radar.mutable_debug_apafrontdistanceinfo_458()->apafr_distance();

    temp=conti_radar.mutable_debug_apafrontdistanceinfo_458()->apafls_distance();
    //apafls_distance.startFliter(temp);
    conti_radar.mutable_debug_apafrontdistanceinfo_458()->set_apafls_distance(apafls_distance.startFliter(temp));

    temp=conti_radar.mutable_debug_apafrontdistanceinfo_458()->apaflm_distance();
    //apaflm_distance.startFliter(temp);
    conti_radar.mutable_debug_apafrontdistanceinfo_458()->set_apaflm_distance(apaflm_distance.startFliter(temp));

    temp=conti_radar.mutable_debug_apafrontdistanceinfo_458()->apafl_distance();
    //apafl_distance.startFliter(temp);
    conti_radar.mutable_debug_apafrontdistanceinfo_458()->set_apafl_distance(apafl_distance.startFliter(temp));
  }
  
  if (conti_radar.has_debug_apareardistanceinfo_457())
  {
    double temp=conti_radar.mutable_debug_apareardistanceinfo_457()->aparrm_distance();
    //apafrm_distance.startFliter(temp);
    conti_radar.mutable_debug_apareardistanceinfo_457()->set_aparrm_distance(aparrm_distance.startFliter(temp));

    temp=conti_radar.mutable_debug_apareardistanceinfo_457()->aparrs_distance();
    //apafrs_distance.startFliter(temp);
    conti_radar.mutable_debug_apareardistanceinfo_457()->set_aparrs_distance(aparrs_distance.startFliter(temp));

    temp=conti_radar.mutable_debug_apareardistanceinfo_457()->aparr_distance();
    //apafr_distance.startFliter(temp);
    conti_radar.mutable_debug_apareardistanceinfo_457()->set_aparr_distance(aparr_distance.startFliter(temp));

    temp=conti_radar.mutable_debug_apareardistanceinfo_457()->aparls_distance();
    //apafls_distance.startFliter(temp);
    conti_radar.mutable_debug_apareardistanceinfo_457()->set_aparls_distance(aparls_distance.startFliter(temp));

    temp=conti_radar.mutable_debug_apareardistanceinfo_457()->aparlm_distance();
    //apaflm_distance.startFliter(temp);
    conti_radar.mutable_debug_apareardistanceinfo_457()->set_aparlm_distance(aparlm_distance.startFliter(temp));

    temp=conti_radar.mutable_debug_apareardistanceinfo_457()->aparl_distance();
    //apafl_distance.startFliter(temp);
    conti_radar.mutable_debug_apareardistanceinfo_457()->set_aparl_distance(aparl_distance.startFliter(temp));
  }


  AdapterManager::FillTteContiRadarHeader(FLAGS_sensor_node_name, &conti_radar);
  AdapterManager::PublishTteContiRadar(conti_radar);
}

// Send the error to monitor and return it
Status TteContiRadarCanbus::OnError(const std::string &error_msg) {
  jmc_auto::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.ERROR(error_msg);
  return Status(ErrorCode::CANBUS_ERROR, error_msg);
}

}  // namespace conti_radar
}  // namespace drivers
}  // namespace jmc_auto
