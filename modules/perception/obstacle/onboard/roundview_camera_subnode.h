#ifndef MODULES_PERCEPTION_OBSTACLE_ONBORAD_ROUNDVIEW_CAMERA_SUBNODE_H_
#define MODULES_PERCEPTION_OBSTACLE_ONBORAD_ROUNDVIEW_CAMERA_SUBNODE_H_

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"
#include "yaml-cpp/yaml.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/timer.h"
#include "modules/common/time/time_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/cuda_util/util.h"
#include "modules/perception/lib/base/singleton.h"
#include "modules/perception/lib/config_manager/calibration_config_manager.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/camera/converter/geometry_camera_converter.h"
//#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/yolo_camera_detector.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_caffe_detector.h"
#include "modules/perception/obstacle/camera/dummy/dummy_algorithms.h"
#include "modules/perception/obstacle/camera/filter/object_camera_filter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_converter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"
#include "modules/perception/obstacle/camera/interface/base_camera_filter.h"
#include "modules/perception/obstacle/camera/interface/base_camera_tracker.h"
#include "modules/perception/obstacle/camera/interface/base_camera_transformer.h"
#include "modules/perception/obstacle/camera/tracker/cascaded_camera_tracker.h"
#include "modules/perception/obstacle/camera/tracker/jmc_camera_tracker.h"
#include "modules/perception/obstacle/camera/transformer/flat_camera_transformer.h"
#include "modules/perception/obstacle/onboard/camera_shared_data.h"
#include "modules/perception/obstacle/onboard/object_shared_data.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/traffic_light/util/color_space.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_avm.h"

namespace jmc_auto {
namespace perception {

class RoundviewCameraSubnode : public Subnode {
 public:
  RoundviewCameraSubnode() = default;
  ~RoundviewCameraSubnode() = default;

  jmc_auto::common::Status ProcEvents() override {
    return jmc_auto::common::Status::OK();
  }

 private:
  bool InitInternal() override;
  bool InitCalibration();
  bool InitModules();

  void ImgCallback(const sensor_msgs::Image& message);
  void ChassisCallback(const jmc_auto::canbus::Chassis& message);
  //void OnTimer(const ros::TimerEvent&);
  
  bool MessageToMat(const sensor_msgs::Image& msg, cv::Mat* img);
  bool MatToMessage(const cv::Mat& img, sensor_msgs::Image *msg);

  void VisualObjToSensorObj(
      const std::vector<std::shared_ptr<VisualObject>>& objects,
      SharedDataPtr<SensorObjects>* sensor_objects);

  void PublishDataAndEvent(const double timestamp,
                           const SharedDataPtr<SensorObjects>& sensor_objects,
                           const SharedDataPtr<CameraItem>& camera_item);

  void PublishPerceptionPbObj(const SharedDataPtr<SensorObjects>&
                              sensor_objects);

  void RemoveObj(std::vector<std::shared_ptr<VisualObject>> &objects);

  // General
  std::string device_id_ = "camera";
  SeqId seq_num_ = 0;
  double timestamp_ns_ = 0.0;

  // Shared Data
  CameraObjectData* cam_obj_data_;
  //CameraSharedData* cam_shared_data_;

  // Calibration
  int32_t image_height_ = 1080;
  int32_t image_width_ = 1920;
  Eigen::Matrix4d camera_to_car_;

  Eigen::Matrix<double, 3, 4> intrinsics_;

  // Dynamic calibration based on objects
  Eigen::Matrix4d camera_to_car_adj_;
  Eigen::Matrix4d car_to_world_;
  Eigen::Matrix4d camera_to_world_;
  Eigen::Matrix4d camera_to_world_adj_;
  int count = 0;//add count number

  //chassis info
  ros::Timer timer_;
  jmc_auto::canbus::Chassis chassis_;

  // Modules
  std::unique_ptr<BaseCameraDetector> detector_;
  std::unique_ptr<BaseCameraTransformer> transformer_;
  std::unique_ptr<jmc_auto::perception::JmcAvm> avm_inst_;
  std::unique_ptr<BaseCameraTracker> tracker_;
  std::unique_ptr<BaseCameraFilter> filter_;
};

REGISTER_SUBNODE(RoundviewCameraSubnode);

}  // namespace perception
}  // namespace jmc_auto

#endif  // MODULES_PERCEPTION_OBSTACLE_ONBORAD_ROUNDVIEW_CAMERA_SUBNODE_H_