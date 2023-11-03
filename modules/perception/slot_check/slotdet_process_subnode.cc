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
#include "modules/perception/slot_check/slotdet_process_subnode.h"
#include "modules/perception/slot_check/slot_shared_data.h"
#include "modules/perception/onboard/subnode.h"
#include "modules/perception/onboard/common_shared_data.h"
#include <ctime>
#include <iomanip>
namespace jmc_auto
{
  namespace perception
  {
    using jmc_auto::common::adapter::AdapterManager;
    using std::string;
    // using std::unordered_map;
    bool SlotDetProcessSubnode::InitInternal()
    {
      
      std::unordered_map<std::string, std::string> fields;
      SubnodeHelper::ParseReserveField(reserve_, &fields);
      device_id_ = fields["device_id"];
        // Shared Data
      slot_share_data_ = static_cast<SlotSharedData *>(
      shared_data_manager_->GetSharedData("SlotSharedData"));

      if (slot_share_data_ == nullptr) {
      AERROR << "Failed to get shared data instance ";
      return false;
      }
      AINFO << "Init shared data successfully, data: " << slot_share_data_->name();
      OutFile.open("/data/csv/roundview_timestamp.txt");
      // slot_share_data_ = dynamic_cast<SlotSharedData *>(
      // shared_data_manager_->GetSharedData("SlotSharedData"));
      // if (slot_share_data_ == nullptr) {
      //   AERROR << "failed to get shared data instance: SlotSharedData ";
      //   return false;
      // }

      ADEBUG << "Det GetPad";
      CHECK(AdapterManager::GetPad()) << "Det padmsg is not initialized.";
      // AdapterManager::AddPadCallback(&SlotCheckProcessSubnode::OnPad, this);
      AdapterManager::AddPlanningPadCallback(&SlotDetProcessSubnode::OnPad, this);

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

      AdapterManager::AddImageRoundviewCallback(&SlotDetProcessSubnode::OnAvmProcess,
                                       this);
      ADEBUG << " Get hdmap";
      hdmap_ = jmc_auto::hdmap::HDMapUtil::BaseMapPtr();
      CHECK(hdmap_) << "Failed to load map file:" << jmc_auto::hdmap::BaseMapFile();

      ADEBUG << " GetLocalization";
      CHECK(AdapterManager::GetLocalization()) << "Localiztion is not initialized.";
      AdapterManager::AddLocalizationCallback(&SlotDetProcessSubnode::OnLocalization, this);

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
      ADEBUG << "slot_det_init ok";
      return true;
    }

    void SlotDetProcessSubnode::OnPad(const jmc_auto::planning::PadMessage &pad_msg){
      if(pad_msg.has_point() && (pad_msg.appmode() == jmc_auto::planning::PARKING || pad_msg.appmode() == jmc_auto::planning::PARKING_NOID ||pad_msg.appmode() == jmc_auto::planning::PARKING_TO_PARKING)){
        slotPointInfos.clear();
        ADEBUG<<"slotPointInfos clean";
        }
    }

    bool SlotDetProcessSubnode::MessageToMat(const sensor_msgs::Image &msg, cv::Mat *img)
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

    void SlotDetProcessSubnode::OnAvmProcess(const sensor_msgs::Image& msg)
    {
      #if 1
      PERF_FUNCTION("SlotdetProcessSubnode");
      PERF_BLOCK_START();

      cv::Mat dst;
      cv::Mat frame;
      MessageToMat(msg, &frame);
      clock_t startTime,endTime;
      // clock_t startTime,endTime,startTime1,endTime1,startTime2,endTime2,startTime3,endTime3,startTime4,endTime4,startTime5,endTime5,startTime6,endTime6;
      clock_t run_startTime,run_endTime,det_startTime,det_endTime;
      //把环视时间戳存进txt
      double roundview_timestamp = msg.header.stamp.toSec();
      // std::ofstream OutFile;
      OutFile<<std::setprecision(20)<<roundview_timestamp<<"\n";
      if (count==0)
      {
        roundview_timestamp_judge =true;
      }else{
        double diff_time = roundview_timestamp-roundview_timestamp_last;
        if (diff_time>0.5)
        {
          roundview_timestamp_judge = false;
          ADEBUG<<"diff_time is "<<diff_time;
        }else
        {
          roundview_timestamp_judge = true;
        }
      }
      roundview_timestamp_last=roundview_timestamp;

      startTime=clock();
      //跳帧识别，每3帧跳2帧
      // if(frame.data && count%3==0)
      if(frame.data && count%3==0 && roundview_timestamp_judge)
      {

        run_startTime=clock();
        avm_inst_->Run(frame,0,dst);

        run_endTime=clock();
        OutFile<<"run_time:"<<(double)(run_endTime-run_startTime)/CLOCKS_PER_SEC<<"\n";

        std::vector<jmc_auto::perception::tJmcPsinfo> psinfos;

        det_startTime=clock();
        psdet_inst_->Detect(dst,0,psinfos);
        ADEBUG<<"psinfo size:"<<psinfos.size();//需要验证识别后的vector的y坐标是否和python一样由小增大
        det_endTime=clock();
        OutFile<<"det_time:"<<(double)(det_endTime-det_startTime)/CLOCKS_PER_SEC<<"\n";
        // ADEBUG<<"(Time1)The run time of Detect is "<<(double)(endTime1-startTime1)/CLOCKS_PER_SEC<<" s.";

        // std::vector<jmc_auto::perception::tslotPoint> slotPointInfos;

        double msg_timestamp = msg.header.stamp.toSec();

        // slotPointInfos.clear();
        const int slotNumId=5;//一次最多推送5个库位
        const int roundview_delay_time=0;//获取当前帧率往前roundview_delay_time个帧率，环视有0-200ms的不定延迟，环视帧率30 SPF

        // startTime2=clock();

        //找到和环视最接近的车辆定位时间戳
        bool localizationTimeStamp_bool=false;

        PERF_BLOCK_END("Definition");

        for(int i=0;i<(localizationTimeStampInfos.size()-1);i++)
        {
          if(localizationTimeStampInfos[i].locTimeStamp <= msg_timestamp && msg_timestamp<=localizationTimeStampInfos[i+1].locTimeStamp)
          {

            if (i-roundview_delay_time>=0)
              {
              vehpose = localizationTimeStampInfos[i-roundview_delay_time].locVehpose;
              quaternion =localizationTimeStampInfos[i-roundview_delay_time].locQuaternion;
              localizationTimeStamp_bool=true;
              ADEBUG<<"find the TimeStamp "<<i-roundview_delay_time;
              }
            else
              {
              vehpose = localizationTimeStampInfos[0].locVehpose;
              quaternion =localizationTimeStampInfos[0].locQuaternion;
              localizationTimeStamp_bool=true;
              ADEBUG<<"find the TimeStamp "<<0;
              }
            break;
          }
        }

        if (localizationTimeStamp_bool==false)
        {
          vehpose = localizationTimeStampInfos.back().locVehpose;
          quaternion =localizationTimeStampInfos.back().locQuaternion;
          ADEBUG<<"find the TimeStamp,the last.";
        }
        // endTime2=clock();
        // ADEBUG<<"(Time2)The run time of DlocalizationTimeStamp is "<<(double)(endTime2-startTime2)/CLOCKS_PER_SEC<<" s.";

        //for(int k=(slotPointInfos.size()-1);k>=0;k--)  
        // for (int i=(psinfos.size()-1); i>=0; i--) //倒序离车辆越近的库位先压入。即标号越小的图像y越大。
        
        int count_slot_y=0;

        for (int i=0; i<psinfos.size(); i++) //先压入y更大的。
        {
          //图像像素大小为1200*1200,实际尺寸为12米*12米
          //修正：以后轴中心为坐标原点，front_edge_to_center:3.886,back_edge_to_center:0.9
          //图像坐标转换成车辆坐标（x,y）=>(x/100-6,7.493-y/100);
          //车辆坐标系中前进方向为Y方向，右方为X方向，单位为米，中心点为原点
          tslotPoint slotPointInfo;
          // slotPointInfo.clear();//??????
          bool pair_bool=false;//是否成功匹配库位

          // Eigen::Vector3d p0((psinfos[i].p0x-40)/100-6, 7.493-psinfos[i].p0y/100, 0);
          // Eigen::Vector3d p1((psinfos[i].p1x-40)/100-6, 7.493-psinfos[i].p1y/100, 0);
          // Eigen::Vector3d p2((psinfos[i].p2x-40)/100-6, 7.493-psinfos[i].p2y/100, 0);
          // Eigen::Vector3d p3((psinfos[i].p3x-40)/100-6, 7.493-psinfos[i].p3y/100, 0);

          Eigen::Vector3d p0;
          Eigen::Vector3d p1;
          Eigen::Vector3d p2;
          Eigen::Vector3d p3;
          if (psinfos[i].p0x-40>0)
          {
            p0=Eigen::Vector3d((psinfos[i].p0x-40)/100-6, 7.493-psinfos[i].p0y/100, 0);
          }
          else{
            p0=Eigen::Vector3d(0, 7.493-psinfos[i].p0y/100, 0);
          }
          if (psinfos[i].p1x-40>0)
          {
            p1=Eigen::Vector3d((psinfos[i].p1x-40)/100-6, 7.493-psinfos[i].p1y/100, 0);
          }
          else{
            p1=Eigen::Vector3d(0, 7.493-psinfos[i].p1y/100, 0);
          }
          if (psinfos[i].p2x-40>0)
          {
            p2=Eigen::Vector3d((psinfos[i].p2x-40)/100-6, 7.493-psinfos[i].p2y/100, 0);
          }
          else{
            p2=Eigen::Vector3d(0, 7.493-psinfos[i].p2y/100, 0);
          }
          if (psinfos[i].p3x-40>0)
          {
            p3=Eigen::Vector3d((psinfos[i].p3x-40)/100-6, 7.493-psinfos[i].p3y/100, 0);
          }
          else{
            p3=Eigen::Vector3d(0, 7.493-psinfos[i].p3y/100, 0);
          }

          Eigen::Vector3d world_space_p0=VehCoInvGeoCo(quaternion, p0,vehpose);
          Eigen::Vector3d world_space_p1=VehCoInvGeoCo(quaternion, p1,vehpose);
          Eigen::Vector3d world_space_p2=VehCoInvGeoCo(quaternion, p2,vehpose);
          Eigen::Vector3d world_space_p3=VehCoInvGeoCo(quaternion, p3,vehpose);

          slotPointInfo.p0 = world_space_p1;
          slotPointInfo.p1 = world_space_p3;
          slotPointInfo.p2 = world_space_p2;
          slotPointInfo.p3 = world_space_p0;//p0-p3排序按左上，左下，右下，右上

          
          // ADEBUG<<"left_up world_space_p1[0]:"<<world_space_p1[0]<<"world_space_p0[1]:"<<world_space_p1[1];
          // ADEBUG<<"right_up world_space_p0[0]:"<<world_space_p0[0]<<"world_space_p0[1]:"<<world_space_p0[1];

          //过滤车辆左侧库位，过滤四个边角的库位(越靠边角正畸效果越不准)
          //if((psinfos[i].p0x>550)&&(psinfos[i].p3y>150)&&(psinfos[i].p0y<1050))
          if((psinfos[i].p0x>550)&&(200<psinfos[i].p0y)&&(psinfos[i].p1y<800))
          {
            //以单个库位中心点搜索地图车位
            jmc_auto::common::PointENU mpoint;
            float mpointX=(world_space_p0[0]+world_space_p1[0]+world_space_p2[0]+world_space_p3[0])/4;
            float mpointY=(world_space_p0[1]+world_space_p1[1]+world_space_p2[1]+world_space_p3[1])/4;
            mpoint.set_x(mpointX);
            mpoint.set_y(mpointY);
            mpoint.set_z(0);
            ADEBUG<<"mpointX:"<<mpointX;
            ADEBUG<<"mpointY:"<<mpointY;
            int l = 4; //地图搜寻半径为4m

            // startTime3=clock();
            std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> temp_parking_spaces; 
            if (hdmap_->GetParkingSpaces(mpoint, l, &temp_parking_spaces) == 0)
            {
              ADEBUG << "Get ParkingSpaces ok";
              for (const auto &parking : temp_parking_spaces)
              {
                //假设点0-3分别对应点ABCD，排序为顺时针（地图坐标轴东为X方向，北为Y方向），点O的坐标为（mpointX,mpointY）
                //当同时满足以下条件时，说明O点在四边形ABCD内部
                //向量AB x 向量AO > 0
                //向量BC x 向量BO > 0
                //向量CD x 向量CO > 0
                //向量DA x 向量DO > 0
                //原理：当O点在凸多边形内部时，O点一定在向量AB、BC、CD、DA的右侧
                double x0 = parking->polygon().points()[0].x(); //提取地图车位角点坐标,0-3分别对应左上，左下，右下，右上
                double y0 = parking->polygon().points()[0].y();
                double x1 = parking->polygon().points()[1].x();
                double y1 = parking->polygon().points()[1].y();
                double x2 = parking->polygon().points()[2].x();
                double y2 = parking->polygon().points()[2].y();
                double x3 = parking->polygon().points()[3].x();
                double y3 = parking->polygon().points()[3].y();

                float a, b, c, d;
                a=(x1-x0)*(mpointY-y0)-(y1-y0)*(mpointX-x0);
                b=(x2-x1)*(mpointY-y1)-(y2-y1)*(mpointX-x1);
                c=(x3-x2)*(mpointY-y2)-(y3-y2)*(mpointX-x2);
                d=(x0-x3)*(mpointY-y3)-(y0-y3)*(mpointX-x3);

                
                ADEBUG<<"a:"<<a<<"b:"<<b<<"c:"<<c<<"d:"<<d;

                if (a>0.001 && b>0.001 && c>0.001 && d>0.001) // Means the map slot 0&3 point are inside parking space.
                {
                  count_slot_y++;
                  ADEBUG<<"count_slot_y:"<<count_slot_y<<" image:"<<parking->id().id()<<"x0:"<<psinfos[i].p1x<<"y0:"<<psinfos[i].p1y<<"x1:"<<psinfos[i].p3x<<"y1:"<<psinfos[i].p3y<<"x2:"<<psinfos[i].p2x<<"y2:"<<psinfos[i].p2y<<"x3:"<<psinfos[i].p0x<<"y3:"<<psinfos[i].p0y;
                  ADEBUG<<std::setprecision(10)<<"det_id:"<<parking->id().id()<<"x0:"<<world_space_p1[0]<<"y0:"<<world_space_p1[1]<<"x1:"<<world_space_p3[0]<<"y1:"<<world_space_p3[1]<<"x2:"<<world_space_p2[0]<<"y2:"<<world_space_p2[1]<<"x3:"<<world_space_p0[0]<<"y3:"<<world_space_p0[1];
                  ADEBUG<<std::setprecision(10)<<"map_id:"<<parking->id().id()<<"x0:"<<x0<<"y0:"<<y0<<"x1:"<<x1<<"y1:"<<y1<<"x2:"<<x2<<"y2:"<<y2<<"x3:"<<x3<<"y3:"<<y3;
                  ADEBUG << "Det Point Inside Check OK！";
                  pair_bool = true; //说明成功匹配地图车位
                  slotPointInfo.soltId = parking->id().id();//添加车位ID
                  slotPointInfo.slotType = ValidMapSlot::VSLOT;//添加车位类型
                  break;
                }
              }
              PERF_BLOCK_END("MapSlot");
            }
            // endTime3=clock();
            // ADEBUG<<"(Time3)The run time of map slot is "<<(double)(endTime3-startTime3)/CLOCKS_PER_SEC<<" s.";

            // startTime4=clock();
            if (pair_bool==false)//没有匹配到地图库位,该库位是地图里没有的库位
            {
              ADEBUG << "Det Point Inside Check lose！";
              if (store_slotPointInfos.size()==0) //store_slotPointInfos为空，create一个id再存储
              {
                slotPointInfo.soltId = "not_map_parking_"+ std::to_string(idcount);//添加车位ID
                slotPointInfo.slotType = ValidMapSlot::VPA;//添加车位类型
                store_slotPointInfos.push_back(slotPointInfo);//存储
                idcount++;
              }
              else
              {
                for (int j=0; j<store_slotPointInfos.size();j++)
                {
                  double x0 = store_slotPointInfos[j].p0[0]; //提取存储的库位角点坐标,0-3分别对应左上，左下，右下，右上
                  double y0 = store_slotPointInfos[j].p0[1];
                  double x1 = store_slotPointInfos[j].p1[0];
                  double y1 = store_slotPointInfos[j].p1[1];
                  double x2 = store_slotPointInfos[j].p2[0];
                  double y2 = store_slotPointInfos[j].p2[1];
                  double x3 = store_slotPointInfos[j].p3[0];
                  double y3 = store_slotPointInfos[j].p3[1];

                  float a1, b1, c1, d1;
                  a1=(x1-x0)*(mpointY-y0)-(y1-y0)*(mpointX-x0);
                  b1=(x2-x1)*(mpointY-y1)-(y2-y1)*(mpointX-x1);
                  c1=(x3-x2)*(mpointY-y2)-(y3-y2)*(mpointX-x2);
                  d1=(x0-x3)*(mpointY-y3)-(y0-y3)*(mpointX-x3);

                  ADEBUG<<"a1:"<<a1<<"b1:"<<b1<<"c1:"<<c1<<"d1:"<<d1;

                  if (a1>0.001 && b1>0.001 && c1>0.001 && d1>0.001) // Means the map slot 0&3 point are inside parking space.
                  {
                    ADEBUG << "Det Point Inside store slot Check OK！"<<"This is a tmp parking";
                    count_slot_y++;
                    ADEBUG<<"count_slot_y:"<<count_slot_y<<" image:"<<store_slotPointInfos[j].soltId<<"x0:"<<psinfos[i].p1x<<"y0:"<<psinfos[i].p1y<<"x1:"<<psinfos[i].p3x<<"y1:"<<psinfos[i].p3y<<"x2:"<<psinfos[i].p2x<<"y2:"<<psinfos[i].p2y<<"x3:"<<psinfos[i].p0x<<"y3:"<<psinfos[i].p0y;
                    ADEBUG<<std::setprecision(10)<<"det_id:"<<store_slotPointInfos[j].soltId<<"x0:"<<world_space_p1[0]<<"y0:"<<world_space_p1[1]<<"x1:"<<world_space_p3[0]<<"y1:"<<world_space_p3[1]<<"x2:"<<world_space_p2[0]<<"y2:"<<world_space_p2[1]<<"x3:"<<world_space_p0[0]<<"y3:"<<world_space_p0[1];
                    ADEBUG<<std::setprecision(10)<<"not_map_parking_store:"<<"x0:"<<x0<<"y0:"<<y0<<"x1:"<<x1<<"y1:"<<y1<<"x2:"<<x2<<"y2:"<<y2<<"x3:"<<x3<<"y3:"<<y3;
                    pair_bool = true; //说明成功匹配存储的库位
                    slotPointInfo.soltId = store_slotPointInfos[j].soltId;//添加库位ID
                    slotPointInfo.slotType = store_slotPointInfos[j].slotType;//添加库位类型
                    break;
                  }
                }
                if (pair_bool==false)
                {
                  ADEBUG << "Det Point slot can not pair！";
                  slotPointInfo.soltId = "not_map_parking_"+ std::to_string(idcount);//添加库位ID

                  ADEBUG<<"slotPointInfo.soltId:"<<slotPointInfo.soltId<<"idcount:"<<idcount;

                  slotPointInfo.slotType = ValidMapSlot::VPA;//添加库位类型
                  store_slotPointInfos.push_back(slotPointInfo);//存储
                  idcount++;
                }
                PERF_BLOCK_END("MapNotSlot");
              }
            }           
            // endTime4=clock();
            // ADEBUG<<"(Time4)The run time of storage slot is "<<(double)(endTime4-startTime4)/CLOCKS_PER_SEC<<" s.";

            // startTime5=clock();
            //判断id是否在slotPointInfos内,如果在则直接
            bool judge_sameid=false;
            for(int k=0;k<slotPointInfos.size();k++)   
            {
              if (slotPointInfos[k].soltId==slotPointInfo.soltId)
              {
                // slotPointInfos.erase(slotPointInfos.begin()+k);//删除已有的id坐标
                slotPointInfos[k]=slotPointInfo;
                judge_sameid=true;
                break;
              }
            }
            //加进vector
            if (judge_sameid==false)
            {
              slotPointInfos.push_back(slotPointInfo);
            }


            // endTime5=clock();
            // ADEBUG<<"(Time5)The run time of delete slot is "<<(double)(endTime5-startTime5)/CLOCKS_PER_SEC<<" s.";

          }
        }

        

        //留slotNumId个id，从前往后删除
        if (slotPointInfos.size()>slotNumId)
        {
          slotPointInfos.erase(slotPointInfos.begin(),slotPointInfos.begin()+slotPointInfos.size()-slotNumId);
        }
        if (store_slotPointInfos.size()>slotNumId)
        {
          store_slotPointInfos.erase(store_slotPointInfos.begin(),store_slotPointInfos.begin()+store_slotPointInfos.size()-slotNumId);
        }
        

        // startTime6=clock();
        //SharedDataPtr<std::vector<jmc_auto::perception::ValidMapSlot>> valid_slots_ptr(new std::vector<jmc_auto::perception::ValidMapSlot>);
        //将slotPointInfos转成ValidMapSlot类的数据
        valid_slots.clear();
        ADEBUG<<"start_det1"<<slotPointInfos.size();
        for (int i=0; i<slotPointInfos.size();i++)
        {
          ADEBUG<<"det_check_"<<i;
          slot.clear_polygon_v(); //清除slot内polygon添加元素值, 以便新的数值添加
          slot.mutable_id_v()->set_id(slotPointInfos[i].soltId);       //提取匹配地图车位到slot内
          auto *point0 = slot.mutable_polygon_v()->add_point(); //将地图车位角点坐标重新压入到slot polygon内
          point0->set_x(slotPointInfos[i].p0[0]);
          point0->set_y(slotPointInfos[i].p0[1]);
          auto *point1 = slot.mutable_polygon_v()->add_point();
          point1->set_x(slotPointInfos[i].p1[0]);
          point1->set_y(slotPointInfos[i].p1[1]);
          auto *point2 = slot.mutable_polygon_v()->add_point();
          point2->set_x(slotPointInfos[i].p2[0]);
          point2->set_y(slotPointInfos[i].p2[1]);
          auto *point3 = slot.mutable_polygon_v()->add_point();
          point3->set_x(slotPointInfos[i].p3[0]);
          point3->set_y(slotPointInfos[i].p3[1]);
          slot.set_slottype(slotPointInfos[i].slotType); //车位类型 0-vertical slot; 1-horizontal slot;2-oblique slot; 3- Valid Parking Area.
          valid_slots.push_back(slot);
        }     
        
        std::shared_ptr<SlotObjects> pub_objects(new SlotObjects);  
        MessageToSharedData(valid_slots,&pub_objects);
        
        // PublishValidSlot();
        PublishDataAndEvent(msg_timestamp, pub_objects);
        // endTime6=clock();
        // ADEBUG<<"(Time6)The run time of valid_slots is "<<(double)(endTime6-startTime6)/CLOCKS_PER_SEC<<" s.";

        //cv::imshow("dst",dst);
        //cv::waitKey(0);

      }

      endTime=clock();
      OutFile<<"all_time:"<<(double)(endTime-startTime)/CLOCKS_PER_SEC<<"\n";
      //ADEBUG<<"The run time is "<<(double)(endTime-startTime)/CLOCKS_PER_SEC<<" s.";
      PERF_BLOCK_END("OnAVMProcess End");


      /*
	    if (dst.data != NULL){
        //show the image
        ADEBUG << "start_image_show!";
        cv::imshow("slot_detector", dst);
        // imshow之后必须有waitKey函数，否则显示窗内将一闪而过，不会驻留屏幕
        cv::waitKey(1);
        }*/
        
        // cv::string out_path;
        // out_path = "pstest/result_vis_"+std::to_string(count)+".jpg";
        // cv::imwrite(out_path,dst);
        count++;
      #endif
    }

    void SlotDetProcessSubnode::OnLocalization(const jmc_auto::localization::LocalizationEstimate &localization)
    {
      double timestamp = localization.header().timestamp_sec();
      ADEBUG << "localization timestamp:" << GLOG_TIMESTAMP(timestamp);
      ADEBUG << "localization message" << localization.ShortDebugString();
      vehpose << (double)localization.pose().position().x(), (double)localization.pose().position().y(), (double)localization.pose().position().z();
      quaternion = localization.pose().orientation();

      if (localizationTimeStampInfos.size()>99)
      {
        localizationTimeStampInfos.erase(localizationTimeStampInfos.begin());
      }

      tlocalizationTimeStamp localizationTimeStampInfo;
      localizationTimeStampInfo.locTimeStamp=timestamp;
      localizationTimeStampInfo.locVehpose=vehpose;
      localizationTimeStampInfo.locQuaternion=quaternion;
      localizationTimeStampInfos.push_back(localizationTimeStampInfo);
      ADEBUG << "start  Localization";
    }

    void SlotDetProcessSubnode::MessageToSharedData(const std::vector<jmc_auto::perception::ValidMapSlot> &valid_slots,
                                SharedDataPtr<SlotObjects> *slot_objects){
      for (auto slot : valid_slots) {
      ((*slot_objects)->objects).emplace_back(slot);
  }
    }

    void SlotDetProcessSubnode::PublishDataAndEvent(
      const double timestamp, const SharedDataPtr<SlotObjects>& valid_slots_sharedData) 
      {
      CommonSharedDataKey key(timestamp, device_id_);
      slot_share_data_->Add(key, valid_slots_sharedData);
      
      for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
        const EventMeta &event_meta = pub_meta_events_[idx];
        Event event;
        event.event_id = event_meta.event_id;
        event.timestamp = timestamp;
        event.reserve = device_id_;
        event_manager_->Publish(event);
        ADEBUG<<"pub_slot_event_num:"<<pub_meta_events_.size();
    }
}

    void SlotDetProcessSubnode::PublishValidSlot()
    {
      MulvalidSlot mulvalidslot;
      ADEBUG << "valid_slots.size:" << valid_slots.size();

     /* for (size_t i = 0; i < valid_slots.size(); i++)
      {
        auto *a = mulvalidslot.add_slotsum();
        a->CopyFrom(valid_slots[i]);
      }*/
      ADEBUG << "PublishValidSlots";
      AdapterManager::FillValidSlotHeader("ValidSlot", &mulvalidslot);
      AdapterManager::PublishValidSlot(mulvalidslot);
    }

  } // namespace perception
} // namespace jmc_auto
