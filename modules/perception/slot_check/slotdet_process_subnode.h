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

#ifndef MODULES_PERCEPTION_SLOT_CHECK_SLOTDET_PROCESS_SUBNODE_H_
#define MODULES_PERCEPTION_SLOT_CHECK_SLOTDET_PROCESS_SUBNODE_H_
#include <boost/circular_buffer.hpp>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <queue>
#include <iostream>
#include <fstream>

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
#include "modules/perception/slot_check/slot_shared_data.h"
#include "modules/perception/onboard/subnode.h"

#include "modules/planning/proto/pad_msg.pb.h"

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

    typedef struct slotPoint{
        std::string soltId;
        jmc_auto::perception::ValidMapSlot_slotype slotType;
        Eigen::Vector3d p0; // left_up 
        Eigen::Vector3d p1; // left_down
        Eigen::Vector3d p2; // right_down
        Eigen::Vector3d p3; // right_up
    }tslotPoint;

    typedef struct localizationTimeStamp{
        double locTimeStamp;
        Eigen::Vector3d locVehpose;
        jmc_auto::common::Quaternion locQuaternion;
    }tlocalizationTimeStamp;


    class SlotDetProcessSubnode : public Subnode
    {
    public:
      SlotDetProcessSubnode() = default;
      ~SlotDetProcessSubnode() = default;

      jmc_auto::common::Status ProcEvents() override
      {
        return jmc_auto::common::Status::OK();
      }
      void OnPad(const jmc_auto::planning::PadMessage &pad_msg);

    private:
      bool InitInternal() override;

      void OnAvmProcess(const sensor_msgs::Image &msg);

      void OnLocalization(const jmc_auto::localization::LocalizationEstimate &localization);

      bool MessageToMat(const sensor_msgs::Image &msg, cv::Mat *img);

      void PublishValidSlot();
      
      void PublishDataAndEvent(const double timestamp,
                          const SharedDataPtr<SlotObjects>& valid_slots_sharedData);

      void MessageToSharedData(const std::vector<jmc_auto::perception::ValidMapSlot> &valid_slots,
                                SharedDataPtr<SlotObjects> *slot_objects);
      std::string device_id_ = "surroundview_camera";
        // Shared Data
      SlotSharedData *slot_share_data_;
      //存储最后需要sharedata的数据
      std::vector<jmc_auto::perception::tslotPoint> slotPointInfos;
      //用来存储未匹配上地图库位id的库位信息
      std::vector<jmc_auto::perception::tslotPoint> store_slotPointInfos;
      int idcount = 0;
      //用来存储车辆定位信息和时间戳
      std::vector<jmc_auto::perception::tlocalizationTimeStamp> localizationTimeStampInfos;

      Eigen::Vector3d vehpose;
      jmc_auto::common::Quaternion quaternion;
      Eigen::Vector3d VehCoInvGeoCo(const jmc_auto::common::Quaternion &Quaternion, const Eigen::Vector3d &p, const Eigen::Vector3d &vehpose) 
      {
      //四元数：w+xi+yj+zk,表示3D方向和旋转
      Eigen::Quaternion<double> quaternion(Quaternion.qw(),Quaternion.qx(), Quaternion.qy(), Quaternion.qz());
      //return quaternion.toRotationMatrix().inverse() * p + vehpose;
      return quaternion.toRotationMatrix() * p + vehpose;
      } //车辆坐标系点直接转换成大地坐标系的坐标
      int count;
      bool inited_ = false;
      bool roundview_timestamp_judge = true; //判断是否需要屏蔽该帧数据（当环视宕机时，该参数为false）
      double roundview_timestamp_last ;
      std::shared_ptr<jmc_auto::perception::JmcAvm> avm_inst_{nullptr};
      std::shared_ptr<jmc_auto::perception::JmcPsDetector> psdet_inst_{nullptr};

      const jmc_auto::hdmap::HDMap *hdmap_ = nullptr; 
      jmc_auto::hdmap::ParkingSpaceInfoConstPtr parking_spaces;

      std::vector<jmc_auto::perception::ValidMapSlot> valid_slots; 
      jmc_auto::perception::ValidMapSlot slot; 
      //将环视时间戳写进txt进行分析
      std::ofstream OutFile;
    };

    REGISTER_SUBNODE(SlotDetProcessSubnode);
  } // namespace perception
} // namespace jmc_auto

#endif // MODULES_PERCEPTION_SLOT_CHECK_SLOTDET_PROCESS_SUBNODE_H_
