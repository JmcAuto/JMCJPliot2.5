/******************************************************************************
 * Copyright 2021 The JmcAuto Authors. All Rights Reserved.
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

#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_avm.h"
#include "modules/common/log.h"

#include <cmath>
#include <unordered_map>
#include <utility>
#include <ctime>


namespace jmc_auto {
namespace perception {

    static std::string lens_file = "modules/perception/data/params/lens.lua";
    static std::string calib_file = "modules/perception/data/params/calibinfo.lua";
    static std::string sensor_file = "modules/perception/data/params/sensor.lua";

    bool JmcAvm::SetConfigFilePath(const std::string &lens_file, const std::string &sensor_file,
                                          const std::string &calib_file, bool is_user)
    {
        lens_file_ = lens_file;
        sensor_file_ = sensor_file;
        calib_file_ = calib_file;
        if(is_user)
        {
            config_cout_++;
        }

        return true;
    }

    bool JmcAvm::Init()
    {
        bool ret = true;
        if(config_cout_ == 0)
        {
            // use default file 
            SetConfigFilePath(lens_file,sensor_file,calib_file,false);
            //std::cout << config_cout_ << std::endl;
        }
        AdjustUtilities::Initialize(lens_file_.c_str(),sensor_file_.c_str());

        auto lua_calib_param = std::make_shared<LuaContext>();
        if(lua_calib_param)
        {
            lua_calib_param->DoFile(calib_file.c_str());
        }

        for(int index = 0; index < (int)CamIndex::Max; index++)
        {
            cam_inter[index] = AdjustUtilities::GetCameraInternal(index,lua_calib_param.get());
            cam_ext[index] = AdjustUtilities::GetCameraExternal(index,lua_calib_param.get());
        }

        GenPixelCor();
        return ret;
    }

    void JmcAvm::GenPixelCor()
    {
        int pos = 0;
        for(int x = 0; x < output_height_; ++x)
        {
            for(int z = 0; z < output_width_; ++z)
            {
                world_[pos].x = avm_h_ / 2.0 - x * mm_per_pix_;
                world_[pos].y = 0;
                world_[pos].z = z * mm_per_pix_ - avm_w_ / 2.0;
                pos++;
            }
        }

        int side_w = (int)((avm_w_ / 2 - car_w_ / 2) / mm_per_pix_ + 0.5);
        int side_h = output_height_;
        int mid_w = (int)(car_w_ / mm_per_pix_ + 0.5);
        int mid_h = (int)((avm_h_ / 2 - car_h_ / 2) / mm_per_pix_ + 0.5);

        // left
        for(int y = 0; y < side_h; ++y)
        {
            for(int x = 0; x < side_w; ++x)
            {
                int index = (int)CamIndex::Left;
                pos = y * output_width_ + x;
                Vector3f wd = world_[pos];
                pixel_[pos] = AdjustUtilities::GetCameraPixel(&cam_inter[index], &cam_ext[index], wd);
                offset_[pos] = cam_offset_[index];
            }
        }

        // front
        for(int y = 0; y < mid_h; ++y)
        {
            for(int x = side_w; x < (side_w + mid_w); ++x)
            {
                int index = (int)CamIndex::Front;
                pos = y * output_width_ + x;
                Vector3f wd = world_[pos];
                pixel_[pos] = AdjustUtilities::GetCameraPixel(&cam_inter[index], &cam_ext[index], wd);
                offset_[pos] = cam_offset_[index];
            }
        }

        // right
        for(int y = 0; y < side_h; ++y)
        {
            for(int x = (side_w + mid_w); x < output_width_; ++x)
            {
                int index = (int)CamIndex::Right;
                pos = y * output_width_ + x;
                Vector3f wd = world_[pos];
                pixel_[pos] = AdjustUtilities::GetCameraPixel(&cam_inter[index], &cam_ext[index], wd);
                offset_[pos] = cam_offset_[index];
            }
        }

        // rear
        for(int y = (output_height_ - mid_h); y < output_height_; ++y)
        {
            for(int x = side_w; x < (side_w + mid_w); ++x)
            {
                int index = (int)CamIndex::Rear;
                pos = y * output_width_ + x;
                Vector3f wd = world_[pos];
                pixel_[pos] = AdjustUtilities::GetCameraPixel(&cam_inter[index], &cam_ext[index], wd);
                offset_[pos] = cam_offset_[index];
            }
        }
    }

    bool JmcAvm::Run(cv::Mat &frame, long long timestamp, cv::Mat &dst)
    {
        bool ret = true;     
        unsigned char* ptr = (unsigned char *)out_.data;
        int w = frame.cols;
        int h = frame.rows;
        caffe::Timer det_time;
        det_time.Start();
        for(int y = 0; y < output_height_; ++y)
        {
            for(int x = 0; x < output_width_; ++x)
            {
                int pos = y * output_width_ + x;
                float cx = pixel_[pos].x + offset_[pos].x * w;
                float cy = pixel_[pos].y + offset_[pos].y * h;
                //float offset_x = offset_[pos].x;
                //float offset_y = offset_[pos].y;
                int offset = (int)(cx + 0.5) + ((int)(cy + 0.5)) * w;
                unsigned char* addr = (unsigned char *)frame.data + 3 * offset;
                ptr[0] = addr[0];
                ptr[1] = addr[1];
                ptr[2] = addr[2];
                ptr += 3;
            }
        }
        dst = out_;
        det_time.Stop();
        //std::cout << "Avm cost: " << det_time.MilliSeconds() << " ms"<< std::endl;
        return ret;
    }


    void JmcAvm::img2ground(const Eigen::Vector2f& cameraPixel,Eigen::Vector3f& ground_position){
        float img_height = 1080;
        float img_width = 1920;
        int cam_index;
        Vector2f img_pix;
        Vector3f ground_tmp_;
        
        if (cameraPixel.x() < (img_width/2) && cameraPixel.y() < (img_height/2)){
            cam_index = (int)CamIndex::Left;
            img_pix.x = cameraPixel.x();
            img_pix.y = cameraPixel.y();
        }
        else if (cameraPixel.x() > (img_width/2) && cameraPixel.y() < (img_height/2)){
            cam_index = (int)CamIndex::Right;
            img_pix.x = cameraPixel.x() - img_width/2;
            img_pix.y = cameraPixel.y();
        }
        else if (cameraPixel.x() < (img_width/2) && cameraPixel.y() > (img_height/2)){
            cam_index = (int)CamIndex::Front;
            img_pix.x = cameraPixel.x();
            img_pix.y = cameraPixel.y() - img_height/2;
        }
        else if (cameraPixel.x() > (img_width/2) && cameraPixel.y() > (img_height/2)){
            cam_index = (int)CamIndex::Rear;
            img_pix.x = cameraPixel.x() - img_width/2;
            img_pix.y = cameraPixel.y() - img_height/2;
        }

        ADEBUG <<"CAMERA_INDEX:" <<cam_index <<";pix.x:" <<img_pix.x <<";pix.y:" <<img_pix.y;
        ground_tmp_ = AdjustUtilities::GetPosition(&cam_inter[cam_index],&cam_ext[cam_index],img_pix,0);
        
        ground_position.x() = ground_tmp_.x;
        ground_position.y() = ground_tmp_.y;
        ground_position.z() = ground_tmp_.z;
        
        ADEBUG <<"OUTPUT_POSITION:" << ground_tmp_.x <<"/"<<ground_tmp_.y<<"/"<<ground_tmp_.z;
    }
}
}
