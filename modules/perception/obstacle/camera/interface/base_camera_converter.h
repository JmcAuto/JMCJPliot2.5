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

// The base class of converting 2D detections into 3D objects

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_CONVERTER_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_CONVERTER_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"

namespace jmc_auto {
namespace perception {

class BaseCameraConverter {
 public:
  BaseCameraConverter() {}
  virtual ~BaseCameraConverter() {}

  virtual bool Init() = 0;

  // @brief: Convert 2D detected objects into physical 3D objects
  // @param [in/out]: detected object lists, added 3D position and orientation
  virtual bool Convert(std::vector<std::shared_ptr<VisualObject>>* objects) = 0;

  virtual std::string Name() const = 0;
  //20210408 Add return distance function
  virtual float get_real_distance_x(int i) = 0;
  virtual float get_real_distance_y(int i) = 0;
  //20210408
  
  //20210414 Add return corners
  virtual Eigen::Vector3f get_corners(int i) = 0;
  //20210414
  private:
  DISALLOW_COPY_AND_ASSIGN(BaseCameraConverter);
};

REGISTER_REGISTERER(BaseCameraConverter);
#define REGISTER_CAMERA_CONVERTER(name) \
  REGISTER_CLASS(BaseCameraConverter, name)

}  // namespace perception
}  // namespace jmc_auto

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_CONVERTER_H_
