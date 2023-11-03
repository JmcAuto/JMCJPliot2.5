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

#ifndef MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_CONTI_RADAR_CANBUS_H_
#define MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR_CONTI_RADAR_CANBUS_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/jmc_auto_app.h"
#include "modules/common/macro.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/canbus/can_comm/can_receiver.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/message_manager.h"
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
#include "modules/drivers/radar/tte_conti_radar/common/tte_gflags.h"
#include "modules/drivers/radar/tte_conti_radar/conti_radar_message_manager.h"
#include "modules/drivers/proto/tte_conti_radar.pb.h"

#include "modules/drivers/radar/tte_conti_radar/proto/conti_radar_conf.pb.h"
/**
 * @namespace jmc_auto::drivers
 * @brief jmc_auto::drivers
 */
namespace jmc_auto {
namespace drivers {
namespace tte_conti_radar {

/**
* @class TteContiRadarCanbus
*
* @brief template of canbus-based sensor module main class (e.g., conti_radar).
*/

using jmc_auto::common::adapter::AdapterConfig;
using jmc_auto::common::adapter::AdapterManager;
using jmc_auto::common::monitor::MonitorMessageItem;
using jmc_auto::common::Status;
using jmc_auto::common::ErrorCode;
using jmc_auto::common::time::Clock;
using jmc_auto::drivers::canbus::CanClientFactory;
using jmc_auto::drivers::canbus::CanClient;
using jmc_auto::drivers::canbus::CanReceiver;
using jmc_auto::drivers::canbus::SenderMessage;
class fliterTte{
  public:
  fliterTte(double a,int b){
    limitTimes=b;
    outliner=a;
    predata=a;
    AINFO<<"limitTimes"<<limitTimes;
    AINFO<<"outliner"<<outliner;
  }
 inline double startFliter(double  data){
    if (outliner-data>2)
    {
     currentTimes=0;
     predata= data;
     return data;
    }
    else
    {
      
      AINFO<<"currentTimes"<<currentTimes;
        if (currentTimes<limitTimes)
        {
          AINFO<<"send predata";
          AINFO<<"currentTimes"<<currentTimes;
          currentTimes++;
          //data=predata;
          return predata;
        }
        //else
        //{
        //  AINFO<<"send outliner";
        //  data=outliner;
          //return outliner;
       // }
       return data;
       AINFO<<"outliner"<<outliner;
       AINFO<<"predata"<<predata;
    }
    
  }

  private:
  int limitTimes=0;
  int currentTimes=0;
  double outliner=0;
  double predata=0;
};
class TteContiRadarCanbus : public jmc_auto::common::JmcAutoApp {
 public:
  // TODO(lizh): check whether we need a new msg item, say
  // MonitorMessageItem::SENSORCANBUS
  TteContiRadarCanbus()
      : monitor_logger_(jmc_auto::common::monitor::MonitorMessageItem::CANBUS) {}

  /**
  * @brief obtain module name
  * @return module name
  */
  std::string Name() const override;

  /**
  * @brief module initialization function
  * @return initialization status
  */
  jmc_auto::common::Status Init() override;

  /**
  * @brief module start function
  * @return start status
  */
  jmc_auto::common::Status Start() override;

  /**
  * @brief module stop function
  */
  void Stop() override;

 private:
  void OnTimer(const ros::TimerEvent &event);
  void PublishSensorData();
  Status OnError(const std::string &error_msg);
  void RegisterCanClients();

  CanConf conti_radar_conf_;
  std::shared_ptr<CanClient> can_client_;
  CanReceiver<TteContiRadar> can_receiver_;
  std::unique_ptr<ContiRadarMessageManager> sensor_message_manager_;
  ros::Timer timer_;

  int64_t last_timestamp_ = 0;
  jmc_auto::common::monitor::MonitorLogger monitor_logger_;

  //滤波
  fliterTte apafrm_distance=fliterTte(FLAGS_apafrm_distance_outliner,FLAGS_apafrm_distance_limittimes);
  fliterTte apafrs_distance=fliterTte(FLAGS_apafrs_distance_outliner,FLAGS_apafrs_distance_limittimes);
  fliterTte apafr_distance=fliterTte(FLAGS_apafr_distance_outliner,FLAGS_apafr_distance_limittimes);
  fliterTte apafls_distance=fliterTte(FLAGS_apafls_distance_outliner,FLAGS_apafls_distance_limittimes);
  fliterTte apaflm_distance=fliterTte(FLAGS_apaflm_distance_outliner,FLAGS_apaflm_distance_limittimes);
  fliterTte apafl_distance=fliterTte(FLAGS_apafl_distance_outliner,FLAGS_apafrs_distance_limittimes);
  // double apafrm_distance_outliner=FLAGS_apafrm_distance_outliner;
  // int apafrm_distance_limittimes=FLAGS_apafrm_distance_limittimes;
  // double apafrs_distance_outliner=FLAGS_apafrs_distance_outliner;
  // int apafrs_distance_limittimes=FLAGS_apafrs_distance_limittimes;
  // double apafr_distance_outliner=FLAGS_apafr_distance_outliner;
  // int apafr_distance_limittimes=FLAGS_apafr_distance_limittimes;
  // double apafls_distance_outliner=FLAGS_apafls_distance_outliner;
  // int apafls_distance_limittimes=FLAGS_apafls_distance_limittimes;
  // double apaflm_distance_outliner=FLAGS_apaflm_distance_outliner;
  // int apaflm_distance_limittimes=FLAGS_apaflm_distance_limittimes;
  // double apafl_distance_outliner=FLAGS_apafl_distance_outliner;
  // int apafrs_distance_limittimes=FLAGS_apafrs_distance_limittimes;

  // fliterTte apafrm_distance(apafrm_distance_outliner,apafrm_distance_limittimes);

  // fliterTte apafrs_distance(apafrs_distance_outliner,apafrs_distance_limittimes);
  // fliterTte apafrs_distance(apafr_distance_outliner,apafr_distance_limittimes);
  // fliterTte apafrs_distance(apafls_distance_outliner,apafls_distance_limittimes);
  // fliterTte apafrs_distance(apaflm_distance_outliner,apaflm_distance_limittimes);
  // fliterTte apafrs_distance(apafl_distance_outliner,apafrs_distance_limittimes);


  fliterTte aparrs_distance=fliterTte(FLAGS_aparrs_distance_outliner,FLAGS_aparrs_distance_limittimes);
  fliterTte aparrm_distance=fliterTte(FLAGS_aparrm_distance_outliner,FLAGS_aparrm_distance_limittimes);
  fliterTte aparr_distance=fliterTte(FLAGS_aparr_distance_outliner,FLAGS_aparr_distance_limittimes);
  fliterTte aparls_distance=fliterTte(FLAGS_aparls_distance_outliner,FLAGS_aparls_distance_limittimes);
  fliterTte aparl_distance=fliterTte(FLAGS_aparl_distance_outliner,FLAGS_aparl_distance_limittimes);
  fliterTte aparlm_distance=fliterTte(FLAGS_aparlm_distance_outliner,FLAGS_aparlm_distance_limittimes);
};

}  // namespace conti_radar
}  // namespace drivers
}  // namespace jmc_auto

#endif  // MODULES_DRIVERS_RADAR_TTE_CONTI_RADAR__CONTI_RADAR_CANBUS_H_
