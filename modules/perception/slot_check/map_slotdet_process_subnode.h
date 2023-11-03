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
//210702-根据测试情况,将队列q1设置为Vector来进行编译说明。

#ifndef MODULES_PERCEPTION_SLOT_CHECK_MAP_SLOTDET_PROCESS_SUBNODE_H_
#define MODULES_PERCEPTION_SLOT_CHECK_MAP_SLOTDET_PROCESS_SUBNODE_H_
#include <boost/circular_buffer.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <queue>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/slotcheck_Info.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/drivers/proto/tte_conti_radar.pb.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/map/hdmap/hdmap_common.h"

#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_avm.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_psdet.h"

#if 0
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#endif
#include <cv_bridge/cv_bridge.h>

namespace jmc_auto
{
  namespace perception
  {
    class MapSlotDetProcessSubnode : public Subnode
    {
    public:
      MapSlotDetProcessSubnode() = default;
      ~MapSlotDetProcessSubnode() = default;

      jmc_auto::common::Status ProcEvents() override
      {
        return jmc_auto::common::Status::OK();
      }

    private:
      bool InitInternal() override;

      void OnAvmProcess(const sensor_msgs::Image &msg);

      void OnLocalization(const jmc_auto::localization::LocalizationEstimate &localization);

      bool MessageToMat(const sensor_msgs::Image &msg, cv::Mat *img);

      void PublishValidSlot();

      Eigen::Vector3d vehpose;
      jmc_auto::common::Quaternion quaternion;
      Eigen::Vector3d VehCoInvGeoCo(const jmc_auto::common::Quaternion &Quaternion, const Eigen::Vector3d &p, const Eigen::Vector3d &vehpose) 
      {
      Eigen::Quaternion<double> quaternion(Quaternion.qw(),Quaternion.qx(), Quaternion.qy(), Quaternion.qz());
      //return quaternion.toRotationMatrix().inverse() * p + vehpose;
      return quaternion.toRotationMatrix() * p + vehpose;
      } //车辆坐标系点直接转换成大地坐标系的坐标
      int count;
      bool inited_ = false;
      std::shared_ptr<jmc_auto::perception::JmcAvm> avm_inst_{nullptr};
      std::shared_ptr<jmc_auto::perception::JmcPsDetector> psdet_inst_{nullptr};

      const jmc_auto::hdmap::HDMap *hdmap_ = nullptr; 
      jmc_auto::hdmap::ParkingSpaceInfoConstPtr parking_spaces;

      std::vector<jmc_auto::perception::ValidMapSlot> valid_slots; 
      jmc_auto::perception::ValidMapSlot slot;   
    };

    REGISTER_SUBNODE(MapSlotDetProcessSubnode);
  } // namespace perception
} // namespace jmc_auto

#endif // MODULES_PERCEPTION_SLOT_CHECK_SLOTDET_PROCESS_SUBNODE_H_
