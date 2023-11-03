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

#include "modules/perception/perception.h"

#include "ros/include/ros/ros.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/onboard/async_fusion_subnode.h"
#include "modules/perception/obstacle/onboard/camera_process_subnode.h"
#include "modules/perception/obstacle/onboard/roundview_camera_subnode.h"
#include "modules/perception/obstacle/onboard/camera_shared_data.h"
#include "modules/perception/obstacle/onboard/cipv_subnode.h"
#include "modules/perception/obstacle/onboard/fusion_shared_data.h"
#include "modules/perception/obstacle/onboard/fusion_subnode.h"
#include "modules/perception/obstacle/onboard/lane_post_processing_subnode.h"
#include "modules/perception/obstacle/onboard/lane_shared_data.h"
#include "modules/perception/obstacle/onboard/lidar_process_subnode.h"
#include "modules/perception/obstacle/onboard/motion_service.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/obstacle/onboard/radar_process_subnode.h"
#include "modules/perception/obstacle/onboard/visualization_subnode.h"
#include "modules/perception/obstacle/onboard/ultrasonic_obstacle_subnode.h"
#include "modules/perception/traffic_light/onboard/tl_preprocessor_subnode.h"
#include "modules/perception/traffic_light/onboard/tl_proc_subnode.h"
#include "modules/perception/slot_check/slotdet_process_subnode.h"
#include "modules/perception/slot_check/slotcheck_process_subnode.h"
#include "modules/perception/slot_check/slot_shared_data.h"

#include "modules/perception/slot_check/map_slotdet_process_subnode.h"
#include "modules/perception/slot_check/map_slotcheck_process_subnode.h"

namespace jmc_auto {
namespace perception {

using jmc_auto::common::adapter::AdapterManager;
using jmc_auto::common::Status;
using jmc_auto::common::ErrorCode;

std::string Perception::Name() const { return "perception"; }

Status Perception::Init() {
  AdapterManager::Init(FLAGS_perception_adapter_config_filename);

  RegistAllOnboardClass();
  const std::string dag_config_path = jmc_auto::common::util::GetAbsolutePath(
      FLAGS_work_root, FLAGS_dag_config_path);

  if (!dag_streaming_.Init(dag_config_path)) {
    AERROR << "failed to Init DAGStreaming. dag_config_path:"
           << dag_config_path;
    return Status(ErrorCode::PERCEPTION_ERROR, "failed to Init DAGStreaming.");
  }
  callback_thread_num_ = 5;

  return Status::OK();
}

void Perception::RegistAllOnboardClass() {
  /// register sharedata
  RegisterFactoryLidarObjectData();
  RegisterFactoryRadarObjectData();
  RegisterFactoryCameraObjectData();
  RegisterFactoryCameraSharedData();
  RegisterFactorySlotSharedData();
//  RegisterFactoryCIPVObjectData();
//  RegisterFactoryLaneSharedData();
  RegisterFactoryFusionSharedData();
//  traffic_light::RegisterFactoryTLPreprocessingData();

  /// register subnode
  RegisterFactoryLidar64ProcessSubnode();
  RegisterFactoryLidar16ProcessSubnode();
  RegisterFactoryRadarProcessSubnode();
  RegisterFactoryCameraProcessSubnode();
  RegisterFactoryRoundviewCameraSubnode();
//  RegisterFactoryCIPVSubnode();
//  RegisterFactoryLanePostProcessingSubnode();
  RegisterFactoryAsyncFusionSubnode();
  RegisterFactoryFusionSubnode();
  RegisterFactoryMotionService();
  RegisterFactoryUltrasonicObstacleSubnode();

  //if(FLAGS_is_map_slotcheck)
  //{
    RegisterFactoryMapSlotDetProcessSubnode();
    RegisterFactoryMapSlotCheckProcessSubnode();
  //}else
  //{
    RegisterFactorySlotDetProcessSubnode();
    RegisterFactorySlotCheckProcessSubnode();
  //}

    
  
  lowcostvisualizer::RegisterFactoryVisualizationSubnode();
//  traffic_light::RegisterFactoryTLPreprocessorSubnode();
//  traffic_light::RegisterFactoryTLProcSubnode();
}

Status Perception::Start() {
  dag_streaming_.Start();
  return Status::OK();
}

void Perception::Stop() {
  dag_streaming_.Stop();
  dag_streaming_.Join();
}

}  // namespace perception
}  // namespace jmc_auto
