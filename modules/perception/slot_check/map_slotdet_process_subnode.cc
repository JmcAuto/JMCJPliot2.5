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
//0915-变更点1: 修改原来VPA内四个角点坐标跳变, 先固化出上一帧的大地坐标系，就不会产生变化。
//0915-变更点2: 滑窗距离设置缩短至4m, 保证M1车位及时输出。 
//0915-变更点3: 去掉M1车位检索的转角要求, 已匹配入弯车位的识别。

#include <math.h>
#include <algorithm>
#include <string>
#include <unordered_map>
#include "Eigen/Core"
#include "ros/include/ros/ros.h"
#include "modules/common/log.h"
#include "modules/common/time/time_util.h"
#include "modules/common/time/timer.h"
#include "modules/perception/onboard/subnode_helper.h"
#include "modules/perception/slot_check/map_slotdet_process_subnode.h"

namespace jmc_auto
{
  namespace perception
  {
    using jmc_auto::common::adapter::AdapterManager;
    using std::string;
    using std::unordered_map;

    bool MapSlotDetProcessSubnode::InitInternal()
    {
      // std::cout.setf(ios::fixed,ios::floatfield);
      ADEBUG << "slot_det_init";
      if (inited_)
      {
        return true;
      }
      count =0;
      #if 1
      //ADEBUG << " Get hdmap";
      //hdmap_ = jmc_auto::hdmap::HDMapUtil::BaseMapPtr();
      //CHECK(hdmap_) << "Failed to load map file:" << jmc_auto::hdmap::BaseMapFile();

      AdapterManager::AddImageRoundviewCallback(&MapSlotDetProcessSubnode::OnAvmProcess,
                                       this);
      ADEBUG << " Get hdmap";
      hdmap_ = jmc_auto::hdmap::HDMapUtil::BaseMapPtr();
      CHECK(hdmap_) << "Failed to load map file:" << jmc_auto::hdmap::BaseMapFile();

      ADEBUG << " GetLocalization";
      CHECK(AdapterManager::GetLocalization()) << "Localiztion is not initialized.";
      AdapterManager::AddLocalizationCallback(&MapSlotDetProcessSubnode::OnLocalization, this);

      if(avm_inst_ == nullptr)
      {
        avm_inst_ = std::make_shared<jmc_auto::perception::JmcAvm>();
      }
      avm_inst_->Init();

      if(psdet_inst_ == nullptr)
      {
        psdet_inst_ = std::make_shared<jmc_auto::perception::JmcPsDetector>();
      }
      psdet_inst_->Init();
      #endif

      inited_ = true;

      return true;
      ADEBUG << "slot_det_init ok";
    }

    bool MapSlotDetProcessSubnode::MessageToMat(const sensor_msgs::Image &msg, cv::Mat *img) 
    {
      *img = cv::Mat(msg.height, msg.width, CV_8UC3);
      int pixel_num = msg.width * msg.height;
      if (msg.encoding.compare("yuyv") == 0) {
        unsigned char *yuv = (unsigned char *)&(msg.data[0]);
        cv::Mat yuyv = cv::Mat(cv::Size(msg.width, msg.height * 2),CV_8UC1, yuv);
        cv::cvtColor(yuyv, *img ,cv::COLOR_YUV2BGR_YUYV);
      } else {
        cv_bridge::CvImagePtr cv_ptr =
            cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        *img = cv_ptr->image;
      }
      return true;
    }
  
    void MapSlotDetProcessSubnode::OnAvmProcess(const sensor_msgs::Image& msg)
    {
      #if 1
      cv::Mat dst;
      cv::Mat frame;
      MessageToMat(msg, &frame);
      if(frame.data)
      {
        avm_inst_->Run(frame,0,dst);

        std::vector<jmc_auto::perception::tJmcPsinfo> psinfos;
        psdet_inst_->Detect(dst,0,psinfos);
        ADEBUG<<"psinfo size:"<<psinfos.size();
      }

	if (dst.data != NULL){
        //show the image
        ADEBUG << "start_image_show!";
        cv::imshow("slot_detector", dst);
        // imshow之后必须有waitKey函数，否则显示窗内将一闪而过，不会驻留屏幕
        cv::waitKey(1);
        }
        cv::string out_path;
        out_path = "pstest/result_vis_"+std::to_string(count)+".jpg";
        cv::imwrite(out_path,dst);
        count++;
      #endif
    }

    void MapSlotDetProcessSubnode::OnLocalization(const jmc_auto::localization::LocalizationEstimate &localization)
    {
      double timestamp = localization.header().timestamp_sec();
      ADEBUG << "localization timestamp:" << GLOG_TIMESTAMP(timestamp);
      ADEBUG << "localization message" << localization.ShortDebugString();
      vehpose << (double)localization.pose().position().x(), (double)localization.pose().position().y(), (double)localization.pose().position().z();
      quaternion = localization.pose().orientation();
      ADEBUG << "start  Localization";
    }

    void MapSlotDetProcessSubnode::PublishValidSlot()
    {
      MulvalidSlot mulvalidslot;
      ADEBUG << "valid_slots.size:" << valid_slots.size();

      for (size_t i = 0; i < valid_slots.size(); i++)
      {
        auto *a = mulvalidslot.add_slotsum();
        a->CopyFrom(valid_slots[i]);
      }
      ADEBUG << "PublishValidSlots";
      AdapterManager::FillValidSlotHeader("ValidSlot", &mulvalidslot);
      AdapterManager::PublishValidSlot(mulvalidslot);
    }

  } // namespace perception
} // namespace jmc_auto
