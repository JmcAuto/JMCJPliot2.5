/******************************************************************************
 * Copyright 2018 The JmcAuto Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_CAFFE_DETECTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_CAFFE_DETECTOR_H_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "caffe/caffe.hpp"

//#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/proto/yolo.pb.h"
//#include "modules/perception/proto/yolo_camera_detector_config.pb.h"

#include "modules/perception/cuda_util/network.h"
#include "modules/perception/cuda_util/region_output.h"
#include "modules/perception/cuda_util/util.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/camera/common/cnn_adapter.h"
#include "modules/perception/obstacle/camera/detector/common/feature_extractor.h"
#include "modules/perception/obstacle/camera/dummy/dummy_algorithms.h"
#include "modules/perception/obstacle/camera/interface/base_camera_detector.h"

#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_object_detector.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_objdet_3d.h"

namespace jmc_auto {
namespace perception {
#define ENABLE_3D_BBOX 0

class JmcCaffeDetector : public BaseCameraDetector {
  public:
  JmcCaffeDetector() : BaseCameraDetector() 
  {
    #if ENABLE_3D_BBOX
    detect_inst_ = std::make_shared<jmc_auto::perception::JmcObjdet3D>();
    ADEBUG << "USE_3D";
    #else
    detect_inst_ = std::make_shared<jmc_auto::perception::JmcObjectDetector>();
    #endif
  }

  virtual ~JmcCaffeDetector() 
  {
    
  }

  bool Init(const CameraDetectorInitOptions &options =
                CameraDetectorInitOptions()) override;

  bool Detect(const cv::Mat &frame, const CameraDetectorOptions &options,
              std::vector<std::shared_ptr<VisualObject>> *objects) override;

  bool Multitask(const cv::Mat &frame, const CameraDetectorOptions &options,
                std::vector<std::shared_ptr<VisualObject>> *objects,
                cv::Mat *mask);

  bool Lanetask(const cv::Mat &frame, cv::Mat *mask);

  bool Extract(std::vector<std::shared_ptr<VisualObject>> *objects) {
    for (auto &extractor : extractors_) {
      extractor->extract(objects);
    }
    return true;
  }

  void RenderVisualObject(cv::Mat &image, std::vector<std::shared_ptr<VisualObject>> *objects);

  std::string Name() const override;

  private:

  std::vector<boost::shared_ptr<BaseFeatureExtractor>> extractors_;

#if ENABLE_3D_BBOX
    std::shared_ptr<jmc_auto::perception::JmcObjdet3D> detect_inst_{nullptr};
#else
    std::shared_ptr<jmc_auto::perception::JmcObjectDetector> detect_inst_{nullptr};
#endif
  
};

REGISTER_CAMERA_DETECTOR(JmcCaffeDetector);

}  // namespace perception
}  // namespace jmc_auto
#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_CAFFE_DETECTOR_H_
