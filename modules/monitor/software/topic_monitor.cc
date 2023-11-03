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

#include "modules/monitor/software/topic_monitor.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/util/string_util.h"
#include "modules/monitor/common/monitor_manager.h"

DEFINE_string(topic_monitor_name, "TopicMonitor", "Name of the topic monitor.");

DEFINE_double(topic_monitor_interval, 5, "Topic status checking interval (s).");

namespace jmc_auto {
namespace monitor {
namespace {

using jmc_auto::common::adapter::AdapterBase;
using jmc_auto::common::adapter::AdapterConfig;
using jmc_auto::common::adapter::AdapterManager;
using jmc_auto::common::util::StrCat;
using jmc_auto::common::util::StringPrintf;

AdapterBase *GetAdapterByMessageType(const AdapterConfig::MessageType type) {
  switch (type) {
    case AdapterConfig::POINT_CLOUD:
      return CHECK_NOTNULL(AdapterManager::GetPointCloud());
    case AdapterConfig::IMAGE_LONG:
      return CHECK_NOTNULL(AdapterManager::GetImageLong());
    case AdapterConfig::IMAGE_SHORT:
      return CHECK_NOTNULL(AdapterManager::GetImageShort());
    case AdapterConfig::IMAGE_FRONT:
      return CHECK_NOTNULL(AdapterManager::GetImageFront());
     case AdapterConfig::IMAGE_ROUNDVIEW:
      return CHECK_NOTNULL(AdapterManager::GetImageRoundview());
    case AdapterConfig::LOCALIZATION:
      return CHECK_NOTNULL(AdapterManager::GetLocalization());
    case AdapterConfig::PERCEPTION_OBSTACLES:
      return CHECK_NOTNULL(AdapterManager::GetPerceptionObstacles());
    case AdapterConfig::PREDICTION:
      return CHECK_NOTNULL(AdapterManager::GetPrediction());
    case AdapterConfig::PLANNING_TRAJECTORY:
      return CHECK_NOTNULL(AdapterManager::GetPlanning());
    case AdapterConfig::CONTROL_COMMAND:
      return CHECK_NOTNULL(AdapterManager::GetControlCommand());
    case AdapterConfig::CONTI_RADAR:
      return CHECK_NOTNULL(AdapterManager::GetContiRadar());
    case AdapterConfig::TTE_CONTI_RADAR:
      return CHECK_NOTNULL(AdapterManager::GetTteContiRadar());
    //case AdapterConfig::RELATIVE_MAP:
    //  return CHECK_NOTNULL(AdapterManager::GetRelativeMap());
    default:
      break;
  }
  AFATAL << "No adapter registered for " << type;
  return nullptr;
}

}  // namespace

TopicMonitor::TopicMonitor(const TopicConf &config, TopicStatus *status)
    : RecurrentRunner(FLAGS_topic_monitor_name, FLAGS_topic_monitor_interval)
    , config_(config), status_(status) {
}

void TopicMonitor::RunOnce(const double current_time) {
  auto *adapter = GetAdapterByMessageType(config_.type());
  if (!adapter->HasReceived()) {
    status_->set_message_delay(-1);
    return;
  }
  const double delay = adapter->GetDelaySec();
  if (delay > config_.acceptable_delay()) {
    status_->set_message_delay(delay);
  } else {
    status_->clear_message_delay();
  }
}

}  // namespace monitor
}  // namespace jmc_auto
