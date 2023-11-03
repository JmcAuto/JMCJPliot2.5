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
//SN7-Slotcheck
//20211130-变更点1: 修改判断车位0,1,2,3四个角点在方框内的算法。

#include "modules/perception/slot_check/map_slotcheck_process_subnode.h"

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

namespace jmc_auto
{
  namespace perception
  {
    using jmc_auto::common::adapter::AdapterManager;
    using std::string;
    using std::unordered_map;
    using jmc_auto::perception::ValidMapSlot;

    bool MapSlotCheckProcessSubnode::InitInternal(void)
    {
      // std::cout.setf(ios::fixed,ios::floatfield);
      ADEBUG << "slot_check_init";
      // if (initialized_)
      // {
      //   return true;
      // }

      // ADEBUG << " Get hdmap";
      hdmap_ = jmc_auto::hdmap::HDMapUtil::BaseMapPtr();
      CHECK(hdmap_) << "Failed to load map file:" << jmc_auto::hdmap::BaseMapFile();

      ADEBUG << " GetPad";
      CHECK(AdapterManager::GetPad()) << "padmsg is not initialized.";
      AdapterManager::AddPlanningPadCallback(&MapSlotCheckProcessSubnode::OnPad, this);

      timer_ = AdapterManager::CreateTimer(ros::Duration(1.0 / 20.0), &MapSlotCheckProcessSubnode::OnTimer, this);


//      Start_Check_ = true;

      ADEBUG << "slot_check_init ok";

      return true;
      
    }


    // void MapSlotCheckProcessSubnode::OnPad(const jmc_auto::control::PadMessage &pad_msg){
    //   if(pad_msg.has_point() && (pad_msg.appmode() == jmc_auto::control::PARKING || pad_msg.appmode() == jmc_auto::control::PARKING_NOID)){
    void MapSlotCheckProcessSubnode::OnPad(const jmc_auto::planning::PadMessage &pad_msg){
      if(pad_msg.has_point() && (pad_msg.appmode() == jmc_auto::planning::PARKING || pad_msg.appmode() == jmc_auto::planning::PARKING_NOID ||pad_msg.appmode() == jmc_auto::planning::PARKING_TO_PARKING)){

        slots_.clear();
        unvalid_slots_.clear();

        //初始化大地坐标系下的B，C点坐标
        jmc_auto::common::Quaternion quaternion = localization_.pose().orientation();
        Eigen::Vector3d vehicle_pose(localization_.pose().position().x(), localization_.pose().position().y(), localization_.pose().position().z());
        Eigen::Vector3d m1_pointB_rfu(m1_b_x_rfu_, m1_b_y_rfu_, 0); //cjx20211123 m1_b_x_rfu_ = 1.0; m1_b_y_rfu_ = -0.5;
        m1_pointB_enu_ = CoordinateTrans(quaternion, m1_pointB_rfu, vehicle_pose);   //cjx20211123 Eigen::Vector3d m1_pointB_enu_; 将b点转化到大地坐标系中
        Eigen::Vector3d m1_pointC_rfu(m1_c_x_rfu_, m1_c_y_rfu_, 0); //cjx20211124 m1_c_x_rfu_ = 6.5; m1_c_y_rfu_ = -0.5;
        m1_pointC_enu_ = CoordinateTrans(quaternion, m1_pointC_rfu, vehicle_pose);
        
        Need_Check_ = true;
        ADEBUG << "slots  is clear";
      }else{
        Need_Check_ = false;
      }
    }

    bool MapSlotCheckProcessSubnode::Judge_vector_pair_id(std::vector<std::pair<std::string, jmc_auto::perception::ValidMapSlot>> vector_pair,std::string find_id){
      for (auto iter = vector_pair.begin();iter !=vector_pair.end();++iter){
        if (iter->first==find_id){
          return true;
        }
      }
      return false;
    }

    void MapSlotCheckProcessSubnode::OnTimer(const ros::TimerEvent&)
    {

      //获取底盘，定位，TTE雷达信息
      AdapterManager::Observe();

      auto chassis_adapter = AdapterManager::GetChassis();  
      if (chassis_adapter->Empty()){
        AINFO<< "not chassis msg";
        return;
      }  
      chassis_ = chassis_adapter->GetLatestObserved();

      auto localization_adapter = AdapterManager::GetLocalization();
      if (localization_adapter->Empty()){
        AINFO<< "not localization msg";
        return;
      }
      localization_ = localization_adapter->GetLatestObserved();

      auto tte_adapter = AdapterManager::GetTteContiRadar();
      if (tte_adapter->Empty()){
        AINFO << "not tte msg";
        return;
      }
      tte_ = tte_adapter->GetLatestObserved();

      // if(!Need_Check_){
      //   AINFO << "not parking mode, not need check parking";
      //   return;
      // }


      auto perception_obstacles_adapter = AdapterManager::GetPerceptionRoundviewObstacles();
      if (!perception_obstacles_adapter->Empty()){
        perception_obstacles_ = perception_obstacles_adapter->GetLatestObserved();
        SearchUnvalidParkingSpace();
      }else{
        ADEBUG << "not tte obstacles msg";
      }


      jmc_auto::common::Quaternion quaternion = localization_.pose().orientation();
      Eigen::Vector3d vehicle_pose(localization_.pose().position().x(), localization_.pose().position().y(), localization_.pose().position().z());
      ADEBUG << "Vehicele position:" << localization_.pose().position().x() << " " << localization_.pose().position().y();
      //RRS sensor detect object distance- M1
      double m1_rrs_distance = tte_.debug_apareardistanceinfo_457().aparrs_distance();
//      double m1_frs_distance = tte_.debug_apafrontdistanceinfo_458().apafrs_distance(); //cjx20211124 用在过滤算法中

      //USS Right Slot Debug Information - M2
      // double m2_a_x = tte_.debug_rightusslot_ptab_467().rightusslot_pta_x();  //cjx20211124 雷达给出的车位坐标点（A点，B点）
      // double m2_a_y = tte_.debug_rightusslot_ptab_467().rightusslot_pta_y();
      // double m2_b_x = tte_.debug_rightusslot_ptab_467().rightusslot_ptb_x();
      // double m2_b_y = tte_.debug_rightusslot_ptab_467().rightusslot_ptb_y();

      // //(VPL)Fusion Right Slot Debug Information -M3
      // double m3_a_x = tte_.debug_rightvplslot_ptab_463().rightvplslot_pta_x();
      // double m3_a_y = tte_.debug_rightvplslot_ptab_463().rightvplslot_pta_y();
      // double m3_b_x = tte_.debug_rightvplslot_ptab_463().rightvplslot_ptb_x();
      // double m3_b_y = tte_.debug_rightvplslot_ptab_463().rightvplslot_ptb_y();
      // double m3_c_x = tte_.debug_rightvplslot_ptcd_464().rightvplslot_ptc_x();
      // double m3_c_y = tte_.debug_rightvplslot_ptcd_464().rightvplslot_ptc_y();
      // double m3_d_x = tte_.debug_rightvplslot_ptcd_464().rightvplslot_ptd_x();
      // double m3_d_y = tte_.debug_rightvplslot_ptcd_464().rightvplslot_ptd_y();

      // if(need_refresh_pointB_pointC_)
      // {
      //   //初始化大地坐标系下的B，C点坐标
      //   Eigen::Vector3d m1_pointB_rfu(m1_b_x_rfu_, m1_b_y_rfu_, 0); //cjx20211123 m1_b_x_rfu_ = 1.0; m1_b_y_rfu_ = -0.5;
      //   m1_pointB_enu_ = CoordinateTrans(quaternion, m1_pointB_rfu, vehicle_pose);   //cjx20211123 Eigen::Vector3d m1_pointB_enu_; 将b点转化到大地坐标系中
      //   Eigen::Vector3d m1_pointC_rfu(m1_c_x_rfu_, m1_c_y_rfu_, 0); //cjx20211124 m1_c_x_rfu_ = 6.5; m1_c_y_rfu_ = -0.5;
      //   m1_pointC_enu_ = CoordinateTrans(quaternion, m1_pointC_rfu, vehicle_pose);

      //   need_refresh_pointB_pointC_ = false;
      // }

      SlotCheckObstacleStatus obstacle_status;
      
      //M1
      bool has_obstacle_now = CheckRrsObstacle(m1_rrs_distance); //判断当前有无障碍物
      if(has_obstacle_now && !has_obstacle_last_)          //障碍物由无到有
      {
        obstacle_status = OBS_APPEAR;
        //obs_count_ = 1;
      }else if (!has_obstacle_now && has_obstacle_last_)   //障碍物由有到无
      {
        obstacle_status = OBS_DISAPPEAR;
      }else if (!has_obstacle_now && !has_obstacle_last_)  //无障碍物
      {
        obstacle_status = NO_OBS;
      }else                                               //有障碍物    
      {
        //obs_count_++;
        obstacle_status = HAS_OBS;
      }
      has_obstacle_last_ = has_obstacle_now;


      double parking_search_r = 5;
      //int filtering_num = 2;
      
      switch (obstacle_status)
      {
      case SlotCheckObstacleStatus::OBS_APPEAR:
      {
        ADEBUG << "OBS_APPEAR";
        Eigen::Vector3d point_a_estimate_rfu = PointAPositionEstimate(m1_rrs_distance, true); //获取A点估算坐标，车辆坐标系
        Eigen::Vector3d point_a_estimate_enu = CoordinateTrans(quaternion, point_a_estimate_rfu, vehicle_pose); //将A点坐标由车辆坐标系转到大地坐标系
        if(CalculateaAbDistance(point_a_estimate_enu, m1_pointB_enu_) >= 6.8)   //计算AB点的距离，并判断是否大于3米
        {
          jmc_auto::common::PointENU search_point_enu;
          search_point_enu.set_x(point_a_estimate_enu[0]);
          search_point_enu.set_y(point_a_estimate_enu[1] - 1.5);
          //search_point_enu.set_z(point_a_estimate_enu[2]);

          std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> search_parking_spaces;
          if (hdmap_->GetParkingSpaces(search_point_enu, parking_search_r, &search_parking_spaces) == 0) //返回0表示搜索成功
          {
            ADEBUG << "M1 Get ParkingSpaces ok";
            //确定大地坐标系下的D点估算坐标
            Eigen::Vector3d m1_pointD_estimate_rfu(point_a_estimate_rfu[0] + 5.5, point_a_estimate_rfu[1], 0); 
            Eigen::Vector3d m1_pointD_estimate_enu = CoordinateTrans(quaternion, m1_pointD_estimate_rfu, vehicle_pose);
            
            SlotJudge(search_parking_spaces, point_a_estimate_enu, m1_pointB_enu_, m1_pointC_enu_, m1_pointD_estimate_enu);

            m1_pointB_enu_ = point_a_estimate_enu; //  更新B，C点坐标
            m1_pointC_enu_ = m1_pointD_estimate_enu;
          }else
          {
            ADEBUG << "No ParkingSpaces Searched!";
          }
        }
        break;
      }

      case SlotCheckObstacleStatus::OBS_DISAPPEAR:
      {
        ADEBUG << "OBS_DISAPPEAR";
       //if(obs_count_ >= filtering_num){
         // ADEBUG << "OBS_DISAPPEAR COMPUTER";
          Eigen::Vector3d m1_pointB_estimate_rfu = PointAPositionEstimate(m1_rrs_distance, false);; //cjx20211123 m1_b_x_rfu_ = 1.0; m1_b_y_rfu_ = -0.5;
          Eigen::Vector3d m1_pointC_estimate_rfu(m1_pointB_estimate_rfu[0] + 5.5, m1_pointB_estimate_rfu[1], 0); //m1_c_x_rfu_ = 6.5;  m1_c_y_rfu_ = -0.5;
          m1_pointB_enu_ = CoordinateTrans(quaternion, m1_pointB_estimate_rfu, vehicle_pose);
          m1_pointC_enu_ = CoordinateTrans(quaternion, m1_pointC_estimate_rfu, vehicle_pose);
        //}

        //m1_pointB_enu_ = m1_pointB_estimate_enu; //  更新B，C点坐标
        //m1_pointC_enu_ = m1_pointC_estimate_enu;
        break;
      }

      case SlotCheckObstacleStatus::NO_OBS:
      {
        Eigen::Vector3d m1_pointA_rfu(m1_b_x_rfu_, m1_b_y_rfu_, 0); //cjx20211123 m1_b_x_rfu_ = 1.0; m1_b_y_rfu_ = -0.5;
        Eigen::Vector3d m1_pointA_enu = CoordinateTrans(quaternion, m1_pointA_rfu, vehicle_pose);   //将A点转化到大地坐标系中
        if(CalculateaAbDistance(m1_pointA_enu, m1_pointB_enu_) >= 9 && CalculateaAbDistance(m1_pointA_enu, m1_pointB_enu_) < 38.44)  //AB距离大于3米,且小于6.16米，则设AB距离等于3米
        {
          // Eigen::Vector3d m1_pointB_rfu(m1_b_x_rfu_, m1_b_y_rfu_ - 3.0, 0);  //B点在车辆坐标系下的坐标
          // Eigen::Vector3d m1_pointC_rfu(m1_b_x_rfu_ + 5.5, m1_b_y_rfu_ - 3.0, 0);  //B点在车辆坐标系下的坐标
          // m1_pointB_enu_ = CoordinateTrans(quaternion, m1_pointB_rfu, vehicle_pose);   //更新B点在大地坐标系中的坐标
          // m1_pointC_enu_ = CoordinateTrans(quaternion, m1_pointC_rfu, vehicle_pose);   //更新C点在大地坐标系中的坐标

          jmc_auto::common::PointENU search_point;
          search_point.set_x(m1_pointA_enu[0]);
          search_point.set_y(m1_pointA_enu[1] - 1.5);
          //search_point.set_z(m1_pointA_enu[2]);

          std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> search_parking_spaces;
          if (0 == hdmap_->GetParkingSpaces(search_point, parking_search_r, &search_parking_spaces)) //返回0表示搜索成功
          {
            Eigen::Vector3d m1_pointD_rfu(m1_b_x_rfu_ + 5.5, m1_b_y_rfu_, 0);  //B点在车辆坐标系下的坐标
            Eigen::Vector3d m1_pointD_enu = CoordinateTrans(quaternion, m1_pointD_rfu, vehicle_pose);
            SlotJudge(search_parking_spaces, m1_pointA_enu, m1_pointB_enu_, m1_pointC_enu_, m1_pointD_enu);
          }else
          {
            ADEBUG << "No ParkingSpaces Searched!";
          }
        }
        if(CalculateaAbDistance(m1_pointA_enu, m1_pointB_enu_) >= 38.44)  //AB距离大于3米，则设AB距离等于3米
        {
          Eigen::Vector3d m1_pointB_rfu(m1_b_x_rfu_, m1_b_y_rfu_ - 6.2, 0);  //B点在车辆坐标系下的坐标
          Eigen::Vector3d m1_pointC_rfu(m1_b_x_rfu_ + 5.5, m1_b_y_rfu_ - 6.2, 0);  //B点在车辆坐标系下的坐标
          m1_pointB_enu_ = CoordinateTrans(quaternion, m1_pointB_rfu, vehicle_pose);   //更新B点在大地坐标系中的坐标
          m1_pointC_enu_ = CoordinateTrans(quaternion, m1_pointC_rfu, vehicle_pose);   //更新C点在大地坐标系中的坐标

          jmc_auto::common::PointENU search_point;
          search_point.set_x(m1_pointA_enu[0]);
          search_point.set_y(m1_pointA_enu[1] - 1.5);
          //search_point.set_z(m1_pointA_enu[2]);

          std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> search_parking_spaces;
          if (0 == hdmap_->GetParkingSpaces(search_point, parking_search_r, &search_parking_spaces)) //返回0表示搜索成功
          {
            Eigen::Vector3d m1_pointD_rfu(m1_b_x_rfu_ + 5.5, m1_b_y_rfu_, 0);  //B点在车辆坐标系下的坐标
            Eigen::Vector3d m1_pointD_enu = CoordinateTrans(quaternion, m1_pointD_rfu, vehicle_pose);
            SlotJudge(search_parking_spaces, m1_pointA_enu, m1_pointB_enu_, m1_pointC_enu_, m1_pointD_enu);
          }else
          {
            ADEBUG << "No ParkingSpaces Searched!";
          }
        }
        break;
      }
      // case SlotCheckObstacleStatus::HAS_OBS:
      // {
      //   if(obs_count_ == filtering_num){
      //     ADEBUG << "OBS_HAS";
      //     //Eigen::Vector3d point_a_estimate_rfu = PointAPositionEstimate(m1_rrs_distance, true); //获取A点估算坐标，车辆坐标系
      //     //Eigen::Vector3d point_a_estimate_enu = CoordinateTrans(quaternion, point_a_estimate_rfu, vehicle_pose); //将A点坐标由车辆坐标系转到大地坐标系
      //     if(CalculateaAbDistance(m1_pointA_enu_, m1_pointB_enu_) >= 6.8)   //计算AB点的距离，并判断是否大于3米
      //     {
      //       jmc_auto::common::PointENU search_point_enu;
      //       search_point_enu.set_x(m1_pointA_enu_[0]);
      //       search_point_enu.set_y(m1_pointA_enu_[1] - 1.5);
      //       //search_point_enu.set_z(point_a_estimate_enu[2]);

      //       std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> search_parking_spaces;
      //       if (hdmap_->GetParkingSpaces(search_point_enu, parking_search_r, &search_parking_spaces) == 0) //返回0表示搜索成功
      //       {
      //         ADEBUG << "M1 Get ParkingSpaces ok";
      //         //确定大地坐标系下的D点估算坐标
      //         //Eigen::Vector3d m1_pointD_estimate_rfu(point_a_estimate_rfu[0] + 5.5, point_a_estimate_rfu[1], 0); 
      //         //Eigen::Vector3d m1_pointD_estimate_enu = CoordinateTrans(quaternion, m1_pointD_estimate_rfu, vehicle_pose);
            
      //         SlotJudge(search_parking_spaces, m1_pointA_enu_, m1_pointB_enu_, m1_pointC_enu_, m1_pointD_enu_);

      //         m1_pointB_enu_ = m1_pointA_enu_; //  更新B，C点坐标
      //         m1_pointC_enu_ = m1_pointD_enu_;
      //       }else
      //       {
      //         ADEBUG << "No ParkingSpaces Searched!";
      //       }
      //     }
      //   }
      //   break;
      // }
      default:
        break;
      } //switch(obstacle_status_)

      //M2
      // Eigen::Vector3d m2_point_b_rfu(m2_b_x - 1, m2_b_y, 0); //TTE ABCD match with map slot 0123 points. A-3, B-0, C-1, D-2
      // Eigen::Vector3d m2_point_b_enu = CoordinateTrans(quaternion, m2_point_b_rfu, vehicle_pose);
      // Eigen::Vector3d m2_point_a_rfu(m2_a_x - 1, m2_a_y, 0);
      // Eigen::Vector3d m2_point_a_enu = CoordinateTrans(quaternion, m2_point_a_rfu, vehicle_pose);

      // //根据TTE给出A，B点坐标点, 计算C，D点坐标
      // Eigen::Vector3d m2_point_c_rfu(m2_b_x + 5.5, m2_b_y, 0);
      // Eigen::Vector3d m2_point_c_enu = CoordinateTrans(quaternion, m2_point_c_rfu, vehicle_pose);
      // Eigen::Vector3d m2_point_d_rfu(m2_a_x + 5.5, m2_a_y, 0);
      // Eigen::Vector3d m2_point_d_enu = CoordinateTrans(quaternion, m2_point_d_rfu, vehicle_pose);

      // //确定车位搜索点
      // Eigen::Vector3d m2_search_point_rfu_temp(m2_a_x, m2_a_y - 1.5, 0);
      // Eigen::Vector3d m2_search_point_enu_temp = CoordinateTrans(quaternion, m2_search_point_rfu_temp, vehicle_pose);

      // if (m2_a_x + m2_a_y + m2_b_x + m2_b_y != 0) //当车位坐标点AB非0,意味着存在右侧超声波车位输出
      // {
      //   ADEBUG << "M2 trigger Sucessfully";
      //   jmc_auto::common::PointENU search_point_enu;
      //   search_point_enu.set_x(m2_search_point_enu_temp[0]);
      //   search_point_enu.set_y(m2_search_point_enu_temp[1]);
      //   //search_point_enu.set_z(m2_search_point_enu_temp[2]);
      //   std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> search_parking_spaces;
      //   if (hdmap_->GetParkingSpaces(search_point_enu, parking_search_r, &search_parking_spaces) == 0) //搜寻到附近地图车位
      //   {
          
      //     SlotJudge(search_parking_spaces, m2_point_a_enu, m2_point_b_enu, m2_point_c_enu, m2_point_d_enu);
      //   }else
      //   {
      //     ADEBUG << "No ParkingSpaces Searched!";
      //   }
      // }

      // //M3-采用融合车位信息来进行地图车位匹配
      // //将A，B,C，D点由车辆坐标系转换为大地坐标系
      // Eigen::Vector3d m3_point_b_rfu(m3_b_x - 1, m3_b_y, 0);
      // Eigen::Vector3d m3_point_b_enu = CoordinateTrans(quaternion, m3_point_b_rfu, vehicle_pose);
      // Eigen::Vector3d m3_point_a_rfu(m3_a_x - 1, m3_a_y, 0);
      // Eigen::Vector3d m3_point_a_enu = CoordinateTrans(quaternion, m3_point_a_rfu, vehicle_pose);
      // Eigen::Vector3d m3_point_c_rfu(m3_c_x, m3_c_y, 0);
      // Eigen::Vector3d m3_point_c_enu = CoordinateTrans(quaternion, m3_point_c_rfu, vehicle_pose);
      // Eigen::Vector3d m3_point_d_rfu(m3_d_x, m3_d_y, 0);
      // Eigen::Vector3d m3_point_d_enu = CoordinateTrans(quaternion, m3_point_d_rfu, vehicle_pose);

      // //确定车位搜索点
      // Eigen::Vector3d m3_search_point_rfu_temp(m3_a_x, m3_a_y - 1.5, 0);
      // Eigen::Vector3d m3_search_point_enu_temp = CoordinateTrans(quaternion, m3_search_point_rfu_temp, vehicle_pose);

      // if (m3_a_x + m3_a_y + m3_b_x + m3_b_y + m3_c_x + m3_c_y + m3_d_x + m3_d_y != 0) // It means USS right slot detected by ultrasonic sensor.
      // {
      //   ADEBUG << "M3 trigger Sucessfully";
      //   jmc_auto::common::PointENU search_point_enu;
      //   search_point_enu.set_x(m3_search_point_enu_temp[0]);
      //   search_point_enu.set_y(m3_search_point_enu_temp[1]);
      //   //search_point_enu.set_z(m3_search_point_enu_temp[2]);
      //   std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> search_parking_spaces;
      //   if (hdmap_->GetParkingSpaces(search_point_enu, parking_search_r, &search_parking_spaces) == 0)
      //   {
      //     SlotJudge(search_parking_spaces, m3_point_a_enu, m3_point_b_enu, m3_point_c_enu, m3_point_d_enu);
      //   }else
      //   {
      //     ADEBUG << "No ParkingSpaces Searched!";
      //   }
      // }
      last_distance_ = m1_rrs_distance;
      PublishValidSlot();
    } //void SlotCheckProcessSubnode::OnTimer(const ros::TimerEvent&)

    //根据RRS的值rrs_distance判断有没有障碍物，返回值true:有障碍物，false:无障碍物
    bool MapSlotCheckProcessSubnode::CheckRrsObstacle(double rrs_distance)
    {
      return rrs_distance < 32;
    }

    //计算两个点在x,y平面内的距离，不开根号
    double MapSlotCheckProcessSubnode::CalculateaAbDistance(const Eigen::Vector3d &point_a, const Eigen::Vector3d &point_b)
    { 
      double distance = (pow(point_a[0] - point_b[0], 2) + pow(point_a[1] - point_b[1], 2));
      ADEBUG << "distance:" << distance;
      return distance;
    }

    //障碍物出现或者消失时估算A点的位置，返回A点在车辆坐标系下的坐标，obstacle_appear = true表示障碍物出现时，false表示障碍物消失时
    Eigen::Vector3d MapSlotCheckProcessSubnode::PointAPositionEstimate(double distance, bool obstacle_appear)
    {
      //double delta_y = std::sin(7.5 * M_PI / 180.0) * distance;
      double buff = 0.2;
      if(obstacle_appear)
      {
        double delta_y = std::sin(7.5 * M_PI / 180.0) * distance;
        Eigen::Vector3d point_a_rfu(1.0, -0.5 + delta_y + buff, 0);
        ADEBUG << "delta_dis:" << point_a_rfu[1];
        return point_a_rfu;
      }else
      {
        double delta_y = std::sin(7.5 * M_PI / 180.0) * last_distance_;
        Eigen::Vector3d point_a_rfu(1.0, -0.5 - delta_y - buff, 0);
        ADEBUG << "delta_dis:" << point_a_rfu[1];
        return point_a_rfu;
      }
      //return point_a_rfu;
  }


  void MapSlotCheckProcessSubnode::SearchUnvalidParkingSpace(){
    if(!perception_obstacles_.has_header()){
      ADEBUG << "tte obstacles is NULL";
      return;
    }

    for(auto& obstacle : perception_obstacles_.perception_obstacle()){
        jmc_auto::common::PointENU search_point;
        search_point.set_x(obstacle.position().x());
        search_point.set_y(obstacle.position().y());

        ADEBUG << "TTE obstacle id:" << obstacle.id();
        //滤除行走的行人
        if(obstacle.type() == perception::PerceptionObstacle::PEDESTRIAN){
          double dis = (pow(obstacle.position().x() - localization_.pose().position().x(), 2) + pow(obstacle.position().y() - localization_.pose().position().y(), 2));
          if(dis < 9){
            ADEBUG << "PEDESTRIAN is too close, skip";
            continue;
          }
        }

        std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> search_parking_spaces;
        if (hdmap_->GetParkingSpaces(search_point, obstacle.width()/2, &search_parking_spaces) == 0) //返回0表示搜索成功
        {
          //ADEBUG << "TTE obstacle id:" << obstacle.id();
          //确定大地坐标系下的D点估算坐标
          //Eigen::Vector3d m1_pointD_estimate_rfu(point_a_estimate_rfu[0] + 5.5, point_a_estimate_rfu[1], 0); 
          //Eigen::Vector3d m1_pointD_estimate_enu = CoordinateTrans(quaternion, m1_pointD_estimate_rfu, vehicle_pose);
          jmc_auto::common::math::Vec2d center_point(obstacle.position().x(), obstacle.position().y());
//          jmc_auto::common::math::Vec2d point_1(obstacle.polygon_point()[1].x(), obstacle.polygon_point()[1].y());
//          jmc_auto::common::math::Vec2d point_2(obstacle.polygon_point()[2].x(), obstacle.polygon_point()[2].y());
//          jmc_auto::common::math::Vec2d point_3(obstacle.polygon_point()[3].x(), obstacle.polygon_point()[3].y());

          for(auto& parking : search_parking_spaces){
            auto id = std::find(unvalid_slots_.begin(), unvalid_slots_.end(), parking->id().id());
            if(id != unvalid_slots_.end()){
              continue;
            }
            //ADEBUG << "TTE obstacle search parking id:" << parking->id().id();
            jmc_auto::common::math::Vec2d search_box_corner(parking->polygon().points()[0].x(), parking->polygon().points()[0].y());  //构建搜索框
            jmc_auto::common::math::Vec2d search_box_opposite_corner(parking->polygon().points()[2].x(), parking->polygon().points()[2].y());
            jmc_auto::common::math::Box2d search_box = jmc_auto::common::math::Box2d::CreateAABox(search_box_corner, search_box_opposite_corner);
            if(search_box.IsPointIn(center_point)){
              if(unvalid_slots_.size() > 10){
                // auto unvalid_slots_end = unvalid_slots_.end();
                // unvalid_slots_end--;
              // std::string delete_unvalid_slots_id;
              // for (auto iter = unvalid_slots_.begin();iter!=unvalid_slots_.end();++iter){
              //   delete_unvalid_slots_id = iter->first;
              // }

                unvalid_slots_.erase(unvalid_slots_.begin());
              }
              ADEBUG << "TTE obstacle search unvalid parking id:" << parking->id().id();
              unvalid_slots_.push_back(parking->id().id());
            }
          }

        }else
        {
          ADEBUG << "No ParkingSpaces Searched by obstacles!";
        }
    }
  }

    //判断哪些车位在vectors包含的矩形框中
    void MapSlotCheckProcessSubnode::SlotJudge(const std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> &parking_spaces, const Eigen::Vector3d &point_a, const Eigen::Vector3d &point_b, const Eigen::Vector3d &point_c, const Eigen::Vector3d &point_d)
    {
      for (auto& parking : parking_spaces)
      {
        //跳过有检测结果的车位
        auto it = std::find(unvalid_slots_.begin(), unvalid_slots_.end(), parking->id().id());
        // if((slots_.find(parking->id().id()) != slots_.end()) || it != unvalid_slots_.end()){
        if((Judge_vector_pair_id(slots_,parking->id().id())) || it != unvalid_slots_.end()){
          continue;
        }

        //提取车位0,1,2,3角点坐标 
        double x0 = parking->polygon().points()[0].x(); 
        double y0 = parking->polygon().points()[0].y();
        double x1 = parking->polygon().points()[1].x();
        double y1 = parking->polygon().points()[1].y();
        double x2 = parking->polygon().points()[2].x();
        double y2 = parking->polygon().points()[2].y();
        double x3 = parking->polygon().points()[3].x();
        double y3 = parking->polygon().points()[3].y();

        //构建车位0,1,2,3四个点
        jmc_auto::common::math::Vec2d point_0(x0, y0);
        // jmc_auto::common::math::Vec2d point_1(x1, y1);
        // jmc_auto::common::math::Vec2d point_2(x2, y2);
        jmc_auto::common::math::Vec2d point_3(x3, y3);
        jmc_auto::common::math::Vec2d search_box_corner(point_b[0], point_b[1]);  //构建搜索框
        jmc_auto::common::math::Vec2d search_box_opposite_corner(point_d[0], point_d[1]);

        jmc_auto::common::math::Box2d search_box = jmc_auto::common::math::Box2d::CreateAABox(search_box_corner, search_box_opposite_corner);
        bool is_point_0_in = search_box.IsPointIn(point_0);
        // bool is_point_1_in = search_box.IsPointIn(point_1);
        // bool is_point_2_in = search_box.IsPointIn(point_2);
        bool is_point_3_in = search_box.IsPointIn(point_3);
        
        ADEBUG << "B-x:" << point_b[0] << " A-x:" << point_a[0] << " D-xy:" << point_d[0] << " " << point_d[1];
        ADEBUG << "parking0-x:" << x0 << " parking3-x:" << x3; 

        if(is_point_0_in && is_point_3_in)
        {
          ADEBUG << "M1-small Point Inside Check OK";
          ValidMapSlot slot;
          //slot_.clear_polygon_v(); //清除slot内polygon添加元素值, 以便新的数值添加

          slot.mutable_id_v()->set_id(parking->id().id());       //提取匹配地图车位到slot内
          ADEBUG << "id:" << parking->id().id();
          auto *point0 = slot.mutable_polygon_v()->add_point(); //将地图车位角点坐标重新压入到slot polygon内
          point0->set_x(x0);
          point0->set_y(y0);
          auto *point1 = slot.mutable_polygon_v()->add_point();
          point1->set_x(x1);
          point1->set_y(y1);
          auto *point2 = slot.mutable_polygon_v()->add_point();
          point2->set_x(x2);
          point2->set_y(y2);
          auto *point3 = slot.mutable_polygon_v()->add_point();
          point3->set_x(x3);
          point3->set_y(y3);
          slot.set_slottype(ValidMapSlot::VSLOT); //车位类型 0-vertical slot; 1-horizontal slot;2-oblique slot; 3- Valid Parking Area.

          //auto id = std::find(unvalid_slots_.begin(), unvalid_slots_.end(), parking->id().id());
          // if((slots_.find(parking->id().id()) == slots_.end()) && 
          //    (std::find(unvalid_slots_.begin(), unvalid_slots_.end(), parking->id().id()) == unvalid_slots_.end())){
            if(slots_.size() > 10){
              // std::unordered_map<std::string, jmc_auto::perception::ValidMapSlot>::iterator slots_end = slots_.end();
              // slots_end--;
              // slots_.erase(slots_end);

              // std::string delete_slots_id;
              // for (auto iter = slots_.begin();iter!=slots_.end();++iter){
              //   delete_slots_id = iter->first;
              // }
              // slots_.erase(delete_slots_id);
              slots_.pop_back();

            }
            // slots_.emplace(parking->id().id(), slot);
            slots_.insert(slots_.begin(),std::make_pair(parking->id().id(), slot));
//          }
          // if (slots.size() == 0) //vector数组为0 需要size前面程序就进行压入设置
          // {
          //   slots_.push_back(slot);
          //   ADEBUG << "M1 Intilize push Ok";
          //   continue;
          // }



          // for (size_t i = 0; i < slots_.size(); i++) //为防止存在检测出地图ID和原有q1内ID存在重复, 进行对比查看。
          // {
          //   if (slots_[i].id_v().id() == parking->id())  //parking->id().id()
          //   {
          //     break;
          //   }
          //   else if (i == slots_.size() - 1)
          //   {
          //     ADEBUG << "M1 push Ok";
          //     slots_.push_back(slot);
          //       //M = true; //true 说明存在有效地图车位压入vector数组内, 无需进行VPA存储。
          //     if (slots_.size() > 10)
          //     {
          //       slots_.erase(slots_.begin());
          //     }
          //   }
          // }
        }
      }
    }

    void MapSlotCheckProcessSubnode::PublishValidSlot(void)
    {
      MulvalidSlot mulvalidslot;
      ADEBUG << "public slots_.size  " << slots_.size();

      // for (size_t i = 0; i < slots_.size(); i++)
      // {
      //   auto *a = mulvalidslot.add_slotsum();
      //   a->CopyFrom(slots_[i].second);
      // }

      for(auto& slot : slots_){
        ADEBUG<<slot.first;
      }


      for(auto& slot : slots_){
        auto *a = mulvalidslot.add_slotsum();
        a->CopyFrom(slot.second);
      }

      ADEBUG << "B-x:" << m1_pointB_enu_[0] << " B-y:" << m1_pointB_enu_[1];
      ADEBUG << "C-x:" << m1_pointC_enu_[0] << " C-y:" << m1_pointC_enu_[1];
      ADEBUG << "PublishValidSlot";
      AdapterManager::FillValidSlotHeader("ValidSlot", &mulvalidslot);
      AdapterManager::PublishValidSlot(mulvalidslot);
    }

  } // namespace perception
} // namespace jmc_auto
