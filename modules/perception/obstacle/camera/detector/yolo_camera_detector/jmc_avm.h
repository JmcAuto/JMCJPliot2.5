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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_AVM_H_H
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_AVM_H_H

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include "Eigen/Core"
//#include <opencv/cxcore.hpp>
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "caffe/caffe.hpp"

#include "third_party/avm/include/LuaContext.h"
#include "third_party/avm/include/AdjustUtilities.h"
#include "third_party/avm/include/DefineCommon.h"

namespace jmc_auto {
namespace perception {

    enum class CamIndex{
        Front = 0,
        Right = 1,
        Rear =  2,
        Left = 3,
        Max = 4,
    };

    class JmcAvm{
    public:
    JmcAvm()
    {
        #if 1
        cam_offset_[(int)CamIndex::Front].x = 0.0;
        cam_offset_[(int)CamIndex::Front].y = 0.5;

        cam_offset_[(int)CamIndex::Right].x = 0.5;
        cam_offset_[(int)CamIndex::Right].y = 0.0;

        cam_offset_[(int)CamIndex::Rear].x = 0.5;
        cam_offset_[(int)CamIndex::Rear].y = 0.5;

        cam_offset_[(int)CamIndex::Left].x = 0.0;
        cam_offset_[(int)CamIndex::Left].y = 0.0;

        #else

        #endif
        output_width_ = (int)(avm_w_ / mm_per_pix_ + 0.5);
        output_height_ = (int)(avm_h_ / mm_per_pix_ + 0.5);
        world_ = new Vector3f[output_width_ * output_height_];
        pixel_ = new Vector2f[output_width_ * output_height_];
        offset_ = new Vector2f[output_width_ * output_height_];
        out_ = cv::Mat(cv::Size(output_width_,output_height_),CV_8UC3);
    }

    virtual ~JmcAvm()
    {

        if(pixel_)
        {
            delete[] pixel_;
            pixel_ = nullptr;
        }

        if(world_)
        {
            delete[] world_;
            world_ = nullptr;
        }

        if(offset_)
        {
            delete[] offset_;
            offset_ = nullptr;
        }
    }

    bool SetConfigFilePath(const std::string &prototxt_file, const std::string &weight_file,
                           const std::string &calib_file, bool is_user = true);
    bool Init();
    
    bool Run(cv::Mat &frame, long long timestamp, cv::Mat &dst);

    void img2ground(const Eigen::Vector2f& cameraPixel,Eigen::Vector3f& ground_position);

    private:
    void GenPixelCor();

    protected:

    float *data_{nullptr};
    float *ltb_{nullptr};
    Vector3f *world_{nullptr};
    Vector2f *pixel_{nullptr};
    Vector2f *offset_{nullptr};

    std::string lens_file_;
    std::string sensor_file_;
    std::string calib_file_;

    int config_cout_{0};

    int output_width_{512};
    int output_height_{512};

    private:
    CameraInternal cam_inter[4];
    CameraExternal cam_ext[4];
    Vector2f cam_offset_[4];

    float mm_per_pix_{10.0}; // pix/mm
    float avm_w_{12000.0}; // mm
    float avm_h_{12000.0}; // mm
    float car_w_{2000.0}; // mm
    float car_h_{5200.0}; // mm
    cv::Mat out_;

  };
}  // namespace perception
}  // namespace jmc_auto
#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_AVM_H_
