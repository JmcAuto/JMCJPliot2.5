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

#include "modules/perception/slot_check/slotcheck_process_subnode.h"

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
#include "modules/perception/onboard/shared_data_manager.h"
#include "modules/perception/onboard/event_manager.h"
#define M_PI 3.1415926535897
namespace jmc_auto
{
  namespace perception
  {
    using jmc_auto::common::adapter::AdapterManager;
    using std::string;
    using std::unordered_map;
    using jmc_auto::common::ErrorCode;
    using jmc_auto::common::Status;
    using jmc_auto::perception::ValidMapSlot;

    bool SlotCheckProcessSubnode::InitInternal(void)
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
      // AdapterManager::AddPadCallback(&SlotCheckProcessSubnode::OnPad, this);
      AdapterManager::AddPlanningPadCallback(&SlotCheckProcessSubnode::OnPad, this);


      timer_ = AdapterManager::CreateTimer(ros::Duration(1.0 / 20.0), &SlotCheckProcessSubnode::OnTimer, this);


//      Start_Check_ = true;
      
      CHECK(shared_data_manager_ != nullptr);
      slot_shared_data_ = dynamic_cast<SlotSharedData *>(shared_data_manager_->GetSharedData("SlotSharedData"));
      if(slot_shared_data_ == nullptr)
      {
        AWARN << "failed to get shared data instance : SlotSharedData.";
      }
      
      if(!InitOutputStream())
      {
        AERROR << "Failed to init output stream.";
        return false;
      }

      ADEBUG << "slot_check_init ok";

      return true;
      
    }


    // void SlotCheckProcessSubnode::OnPad(const jmc_auto::control::PadMessage &pad_msg){
    //   if(pad_msg.has_point() && (pad_msg.appmode() == jmc_auto::control::PARKING || pad_msg.appmode() == jmc_auto::control::PARKING_NOID)){
    void SlotCheckProcessSubnode::OnPad(const jmc_auto::planning::PadMessage &pad_msg){
      if(pad_msg.has_point() && (pad_msg.appmode() == jmc_auto::planning::PARKING || pad_msg.appmode() == jmc_auto::planning::PARKING_NOID||pad_msg.appmode() == jmc_auto::planning::PARKING_TO_PARKING)){
        slots_.clear();
        unvalid_slots_.clear();
        slots_to_check_.clear();
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

    std::vector<std::pair<std::string, jmc_auto::perception::ValidMapSlot>> SlotCheckProcessSubnode::Delete_vector_pair_id(std::vector<std::pair<std::string, jmc_auto::perception::ValidMapSlot>> vector_pair,std::string delete_id){
      //删除指定id
      for (auto iter = vector_pair.begin();iter !=vector_pair.end();++iter){
        if (iter->first==delete_id){
          vector_pair.erase(iter);
          break;
        }
      }
      return vector_pair;
    }

    bool SlotCheckProcessSubnode::Judge_vector_pair_id(std::vector<std::pair<std::string, jmc_auto::perception::ValidMapSlot>> vector_pair,std::string find_id){
      for (auto iter = vector_pair.begin();iter !=vector_pair.end();++iter){
        if (iter->first==find_id){
          return true;
        }
      }
      return false;
    }

      

    void SlotCheckProcessSubnode::OnTimer(const ros::TimerEvent&)
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
        if (perception_obstacles_vector_.size()>=100 )
        {
          perception_obstacles_vector_.erase(perception_obstacles_vector_.begin());
        }
        if (perception_obstacles_count_%5==0)
        {
          perception_obstacles_vector_.push_back(perception_obstacles_);
        }
        perception_obstacles_count_++;

        SearchUnvalidParkingSpace();
      }else{
        ADEBUG << "not perception obstacles msg";
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
        if(CalculateaAbDistance(point_a_estimate_enu, m1_pointB_enu_) >= 6.8)   //计算AB点的距离，并判断是否大于2.6米
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

  //20220210 环视会给出可用车位vector，当车辆行驶到某个可用车位正前方时，此时该车位四个角点坐标的精度最高，记下该车位的信息，并判断该车位内有无障碍物
	
	jmc_auto::perception::MulvalidSlot perception_slots;  
	if(roundview_slots_ && (roundview_slots_->objects.size() > 0))
	{
		// MutexLock lock(&slot_mutex_);
		for(auto& slot : roundview_slots_->objects)
		{	
			auto *s = perception_slots.add_slotsum();
			s->CopyFrom(slot);
		}
    ADEBUG <<" perception_slots.slotsum_size(): "<<perception_slots.slotsum_size();
	}else
  {
    ADEBUG << "slotcheck : ----------roundview_slot size <= 0.";
  }

    for (int i=0; i<perception_slots.slotsum_size();i++)
    {
      auto slot_to_check_id = perception_slots.slotsum(i).id_v().id(); 
      auto iter = std::find(unvalid_slots_.begin(), unvalid_slots_.end(), slot_to_check_id);//返回一个迭代指针
      //车位id不在不可用车位，且不在待检测的序列，且不在可用车位里，如果iter == unvalid_slots_.end(),说明没找到
      // if((iter == unvalid_slots_.end()) && (slots_to_check_.find(slot_to_check_id) == slots_to_check_.end()) && (slots_.find(slot_to_check_id) == slots_.end()))
      if((iter == unvalid_slots_.end()) && (!Judge_vector_pair_id(slots_to_check_,slot_to_check_id)) && (!Judge_vector_pair_id(slots_,slot_to_check_id)))
      { 
        //直接存待检测的车位信息
        // slots_to_check_.emplace(slot_to_check_id, perception_slots.slotsum(i));
        slots_to_check_.push_back(std::make_pair(slot_to_check_id, perception_slots.slotsum(i)));

      }
      //车位id不在不可用车位，且在待检测的序列，且不在可用车位里
      // else if ((iter == unvalid_slots_.end()) && (slots_to_check_.find(slot_to_check_id) != slots_to_check_.end()) && (slots_.find(slot_to_check_id) == slots_.end()))
      else if ((iter == unvalid_slots_.end()) && (Judge_vector_pair_id(slots_to_check_,slot_to_check_id)) && (!Judge_vector_pair_id(slots_,slot_to_check_id)))

      {
        //删除原来的，更新成现在的新的
        // slots_to_check_.erase(slot_to_check_id);
        slots_to_check_=Delete_vector_pair_id(slots_to_check_,slot_to_check_id);
        // slots_to_check_.emplace(slot_to_check_id, perception_slots.slotsum(i));
        slots_to_check_.push_back(std::make_pair(slot_to_check_id, perception_slots.slotsum(i)));

      }
    }
    ADEBUG<<"slots_to_check_.size():"<<slots_to_check_.size();
    ADEBUG << "unvalid_slots_.size(): " << unvalid_slots_.size();
    ADEBUG << "slots_.size(): " << slots_.size();
    for(auto& slot : slots_to_check_){
        ADEBUG<<"slots_to_check_:"<<slot.first;
      }


	// unsigned int right_side_slot_index;
	// if((perception_slots.slotsum_size() > 0) && FindRightSideSlot(perception_slots, &right_side_slot_index))
	// {
	// 	auto slot_to_check_id = perception_slots.slotsum(right_side_slot_index).id_v().id(); 
	// 	auto iter = std::find(unvalid_slots_.begin(), unvalid_slots_.end(), slot_to_check_id);	
	// 	//车位id不在不可用车位里且不在待检测的序列中且不在可用车位里
	// 	if((iter == unvalid_slots_.end()) && (slots_to_check_.find(slot_to_check_id) == slots_to_check_.end()) && (slots_.find(slot_to_check_id) == slots_.end()))
	// 	{ 
	// 		//保存待检测的车位信息
	// 		slots_to_check_.emplace(slot_to_check_id, perception_slots.slotsum(right_side_slot_index));
	// 	}
	// }else
  // {
  //   ADEBUG << "slotcheck : ----------findRightSideSlot not started. ";
  // }
	if(slots_to_check_.size() > 0)
	{ int slots_to_check_size=slots_to_check_.size();
		auto slot_to_check = slots_to_check_.begin()->second;
    // auto slot_to_check_id = slots_to_check_.begin()->first;
		double point_0_x = slot_to_check.polygon_v().point(0).x();
		double point_0_y = slot_to_check.polygon_v().point(0).y();
		double point_3_x = slot_to_check.polygon_v().point(3).x();
		double point_3_y = slot_to_check.polygon_v().point(3).y();
		common::math::Vec2d point0_vec(point_0_x, point_0_y);
		common::math::Vec2d point3_vec(point_3_x, point_3_y);

		Eigen::Vector3d point_a_rfu(m1_b_x_rfu_, m1_b_y_rfu_, 0);
		Eigen::Vector3d point_a_enu = CoordinateTrans(quaternion, point_a_rfu, vehicle_pose);
		double point_a_enu_x = point_a_enu.x();
		double point_a_enu_y = point_a_enu.y();
		common::math::Vec2d point_a_vec(point_a_enu_x, point_a_enu_y);
		
		common::math::LineSegment2d slot_linesegment(point0_vec, point3_vec);
		double proj = slot_linesegment.ProjectOntoUnit(point_a_vec);

		common::math::Box2d slot_box(slot_linesegment, 6.0);
		double point_b_x = m1_pointB_enu_.x();
		double point_b_y = m1_pointB_enu_.y();
		common::math::Vec2d point_b_vec(point_b_x, point_b_y);

		double slot_len = slot_linesegment.length();
		if((proj > 0) && (proj < slot_len + 0.1))
		{
			if(slot_box.IsPointIn(point_b_vec))	//说明车位内有障碍物
			{
				if(unvalid_slots_.size() > 15)
				{
          // auto unvalid_slots_end = unvalid_slots_.end();
          // unvalid_slots_end--;
          // std::string delete_unvalid_slots_id;
          // for (auto iter = unvalid_slots_.begin();iter!=unvalid_slots_.end();++iter){
          //   delete_unvalid_slots_id = iter->first;
          // }

					unvalid_slots_.erase(unvalid_slots_.begin());
				}
        ADEBUG << "the unvalidunvalid_slots_ id : " << slot_to_check.id_v().id();
        ADEBUG << "unvalid_slots_.size(): " << unvalid_slots_.size();
				unvalid_slots_.push_back(slot_to_check.id_v().id());

				// slots_to_check_.erase(slot_to_check.id_v().id());
        slots_to_check_=Delete_vector_pair_id(slots_to_check_,slot_to_check.id_v().id());
			}
		}else if(proj >= slot_len + 0.1 &&(slot_to_check.id_v().id().find("not_map_parking_")!=0))//而且是地图ID
		{ //TTE认为该库位有效
			if(!slot_box.IsPointIn(point_b_vec))
			{     
        // ADEBUG<<"Perception add slots_(det):"<<slot_to_check.id_v().id();
        auto iter = std::find(unvalid_slots_.begin(), unvalid_slots_.end(), slot_to_check.id_v().id());
        //环视障碍物认为该车位无效(TTE认为有效)
        if (iter != unvalid_slots_.end())
        {
          ADEBUG<<"TTE yes ,roundview Obstacles no,id: "<<slot_to_check.id_v().id();
          // slots_to_check_.erase(slot_to_check.id_v().id());
          slots_to_check_=Delete_vector_pair_id(slots_to_check_,slot_to_check.id_v().id());
        }else//环视障碍物不认为该车位无效(TTE认为有效)，说明真的有效
        {
          double point_1_x = slot_to_check.polygon_v().point(1).x();				
          double point_1_y = slot_to_check.polygon_v().point(1).y();
          double point_2_x = slot_to_check.polygon_v().point(2).x();				
          double point_2_y = slot_to_check.polygon_v().point(2).y();
          ValidMapSlot slot;
          slot.mutable_id_v()->set_id(slot_to_check.id_v().id());
          auto *point0 = slot.mutable_polygon_v()->add_point();
          point0->set_x(point_0_x);
          point0->set_y(point_0_y);
          auto *point1 = slot.mutable_polygon_v()->add_point();
          point1->set_x(point_1_x);
          point1->set_y(point_1_y);
          auto *point2 = slot.mutable_polygon_v()->add_point();
          point2->set_x(point_2_x);
          point2->set_y(point_2_y);
          auto *point3 = slot.mutable_polygon_v()->add_point();
          point3->set_x(point_3_x);
          point3->set_y(point_3_y);
          slot.set_slottype(ValidMapSlot::VSLOT);

          if(slots_.size() >= 10)
          {
            // std::string delete_slots_id;
            // for (auto iter = slots_.begin();iter!=slots_.end();++iter){
            //   delete_slots_id = iter->first;
            // }
            // slots_.erase(delete_slots_id);
            slots_.pop_back();
          }
          // slots_.emplace(slot_to_check.id_v().id(), slot);
          slots_.insert(slots_.begin(),std::make_pair(slot_to_check.id_v().id(), slot));
          // slots_to_check_.erase(slot_to_check.id_v().id());
          slots_to_check_ = Delete_vector_pair_id(slots_to_check_,slot_to_check.id_v().id());
          ADEBUG<<"TTE yes ,roundview Obstacles yes,id: "<<slot_to_check.id_v().id();
        }
			}

    }else if((proj >= slot_len + 0.1) &&(!slot_box.IsPointIn(point_b_vec))&&(slot_to_check.id_v().id().find("not_map_parking_")==0))//非地图ID的库位
      {
        double point_1_x = slot_to_check.polygon_v().point(1).x();				
        double point_1_y = slot_to_check.polygon_v().point(1).y();
        double point_2_x = slot_to_check.polygon_v().point(2).x();				
        double point_2_y = slot_to_check.polygon_v().point(2).y();
        bool perception_obstacles_judge=true;
        for(int j=0;j<perception_obstacles_vector_.size();j++){
          for(auto& obstacle : perception_obstacles_vector_[j].perception_obstacle()){
            double obstacle_center_x = obstacle.position().x();
            double obstacle_center_y = obstacle.position().y();
            float e, f, g, h;
            e=(point_1_x-point_0_x)*(obstacle_center_y-point_0_y)-(point_1_y-point_0_y)*(obstacle_center_x-point_0_x);
            f=(point_2_x-point_1_x)*(obstacle_center_y-point_1_y)-(point_2_y-point_1_y)*(obstacle_center_x-point_1_x);
            g=(point_3_x-point_2_x)*(obstacle_center_y-point_2_y)-(point_3_y-point_2_y)*(obstacle_center_x-point_2_x);
            h=(point_0_x-point_3_x)*(obstacle_center_y-point_3_y)-(point_0_y-point_3_y)*(obstacle_center_x-point_3_x);
            if (e>0.001 && f>0.001 && g>0.001 && h>0.001) // 该非地图库位有环视障碍物
            {//将ID加入到无效车位unvalid_slots_中
              if(unvalid_slots_.size() > 15)
              {
                // auto unvalid_slots_end = unvalid_slots_.end();
                // unvalid_slots_end--;
                // std::string delete_unvalid_slots_id;
                // for (auto iter = unvalid_slots_.begin();iter!=unvalid_slots_.end();++iter){
                //   delete_unvalid_slots_id = iter->first;
                // }

                unvalid_slots_.erase(unvalid_slots_.begin());
              }
              ADEBUG << "the not map slot is unvalid , id : " << slot_to_check.id_v().id();
              ADEBUG << "unvalid_slots_.size(): " << unvalid_slots_.size();
              unvalid_slots_.push_back(slot_to_check.id_v().id());

				      // slots_to_check_.erase(slot_to_check.id_v().id());
              slots_to_check_ = Delete_vector_pair_id(slots_to_check_,slot_to_check.id_v().id());
              perception_obstacles_judge=false;
              break;
            }        
          }
          if (perception_obstacles_judge==false)
          {
            break;
          }
        }
        if(perception_obstacles_judge)
        {//将ID加入到有效库位slots_中
          double point_1_x = slot_to_check.polygon_v().point(1).x();				
          double point_1_y = slot_to_check.polygon_v().point(1).y();
          double point_2_x = slot_to_check.polygon_v().point(2).x();				
          double point_2_y = slot_to_check.polygon_v().point(2).y();
          ValidMapSlot slot;
          slot.mutable_id_v()->set_id(slot_to_check.id_v().id());
          auto *point0 = slot.mutable_polygon_v()->add_point();
          point0->set_x(point_0_x);
          point0->set_y(point_0_y);
          auto *point1 = slot.mutable_polygon_v()->add_point();
          point1->set_x(point_1_x);
          point1->set_y(point_1_y);
          auto *point2 = slot.mutable_polygon_v()->add_point();
          point2->set_x(point_2_x);
          point2->set_y(point_2_y);
          auto *point3 = slot.mutable_polygon_v()->add_point();
          point3->set_x(point_3_x);
          point3->set_y(point_3_y);
          slot.set_slottype(ValidMapSlot::VSLOT);
          if(slots_.size() >= 10)
          {
            // auto slots_end = slots_.end();
            // slots_end--;
            // std::string delete_slots_id;
            // for (auto iter = slots_.begin();iter!=slots_.end();++iter){
            //   delete_slots_id = iter->first;
            // }

            // slots_.erase(delete_slots_id);
            slots_.pop_back();
          }
          // slots_.emplace(slot_to_check.id_v().id(), slot);
          slots_.insert(slots_.begin(),std::make_pair(slot_to_check.id_v().id(), slot));
          ADEBUG<<"Perception add slots_(det):"<<slot_to_check.id_v().id();
          ADEBUG<<std::setprecision(10)<<"valid_slot_id:"<<slot_to_check.id_v().id()<<"x0:"<<point_0_x<<"y0:"<<point_0_y<<"x1:"<<point_1_x<<"y1:"<<point_1_y<<"x2:"<<point_2_x<<"y2:"<<point_2_y<<"x3:"<<point_3_x<<"y3:"<<point_3_y;
          // slots_to_check_.erase(slot_to_check.id_v().id());
          slots_to_check_ = Delete_vector_pair_id(slots_to_check_,slot_to_check.id_v().id());
        }
      }
    
    if(slots_to_check_size==slots_to_check_.size())//如果障碍物完成检查后，第一个待检车位还在(还不能确定是否有障碍物)，则将该车位放到最后一个
    {
      // auto slot_to_check_begin = slots_to_check_.begin();
      // auto slot_to_check = slots_to_check_.begin()->first;

      // slots_to_check_.erase(slot_to_check.id_v().id());
      slots_to_check_ = Delete_vector_pair_id(slots_to_check_,slot_to_check.id_v().id());
      // slots_to_check_.emplace(slot_to_check.id_v().id(),slot_to_check);
      slots_to_check_.push_back(std::make_pair(slot_to_check.id_v().id(), slot_to_check));

    }
    
  }else
  {
    ADEBUG << "slotcheck : ----------slot_to_check_ size <= 0.";
  }

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
    bool SlotCheckProcessSubnode::CheckRrsObstacle(double rrs_distance)
    {
      return rrs_distance < 32;
    }

    //计算两个点在x,y平面内的距离，不开根号
    double SlotCheckProcessSubnode::CalculateaAbDistance(const Eigen::Vector3d &point_a, const Eigen::Vector3d &point_b)
    { 
      double distance = (pow(point_a[0] - point_b[0], 2) + pow(point_a[1] - point_b[1], 2));
      ADEBUG << "distance:" << distance;
      return distance;
    }

    //障碍物出现或者消失时估算A点的位置，返回A点在车辆坐标系下的坐标，obstacle_appear = true表示障碍物出现时，false表示障碍物消失时
    Eigen::Vector3d SlotCheckProcessSubnode::PointAPositionEstimate(double distance, bool obstacle_appear)
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

/*
  void SlotCheckProcessSubnode::SearchUnvalidParkingSpace(){
    if(!perception_obstacles_.has_header()){
      ADEBUG << "PerceptionRoundview obstacles is NULL";
      return;
    }

    for(auto& obstacle : perception_obstacles_.perception_obstacle()){
        jmc_auto::common::PointENU search_point;
        search_point.set_x(obstacle.position().x());
        search_point.set_y(obstacle.position().y());

        ADEBUG << "PerceptionRoundview obstacle id:" << obstacle.id();
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
              if(unvalid_slots_.size() > 15){
                unvalid_slots_.erase(unvalid_slots_.begin());
              }
              ADEBUG << "Perception obstacle search unvalid parking id:" << parking->id().id();
              unvalid_slots_.push_back(parking->id().id());
            }
          }

        }else
        {
          ADEBUG << "No ParkingSpaces Searched by perception obstacles!";
        }
    }
  }*/


  //zzhou算法优化20220525:
  void SlotCheckProcessSubnode::SearchUnvalidParkingSpace(){
    if(!perception_obstacles_.has_header()){
      ADEBUG << "PerceptionRoundview obstacles is NULL";
      return;
    }

    for(auto& obstacle : perception_obstacles_.perception_obstacle()){
        jmc_auto::common::PointENU search_point;
        double mpointX = obstacle.position().x();
        double mpointY = obstacle.position().y();
        search_point.set_x(mpointX);
        search_point.set_y(mpointY);

        ADEBUG << "PerceptionRoundview obstacle id:" << obstacle.id();
        //滤除行走的行人
        if(obstacle.type() == perception::PerceptionObstacle::PEDESTRIAN){
          double dis = (pow(obstacle.position().x() - localization_.pose().position().x(), 2) + pow(obstacle.position().y() - localization_.pose().position().y(), 2));
          if(dis < 9){
            ADEBUG << "PEDESTRIAN is too close, skip";
            continue;
          }
        }

        std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> search_parking_spaces;
        double search_r_unvaildsolts;//障碍物的长不一定比宽的值大
        if (obstacle.width()>obstacle.length())
        {
          search_r_unvaildsolts=obstacle.length();
        }else{
          search_r_unvaildsolts=obstacle.width();
        }

        if (hdmap_->GetParkingSpaces(search_point, search_r_unvaildsolts/2, &search_parking_spaces) == 0) //返回0表示搜索成功
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
            // jmc_auto::common::math::Vec2d search_box_corner(parking->polygon().points()[0].x(), parking->polygon().points()[0].y());  //构建搜索框
            // jmc_auto::common::math::Vec2d search_box_opposite_corner(parking->polygon().points()[2].x(), parking->polygon().points()[2].y());
            // jmc_auto::common::math::Box2d search_box = jmc_auto::common::math::Box2d::CreateAABox(search_box_corner, search_box_opposite_corner);
            

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
            
            if(a>0.0001 && b>0.0001 && c>0.0001 && d>0.0001)// 满足则说明该障碍物在该库位里
            {
              if(unvalid_slots_.size() >=15){
                // auto unvalid_slots_end = unvalid_slots_.end();
                // unvalid_slots_end--;
                // std::string delete_unvalid_slots_id;
                // for (auto iter = unvalid_slots_.begin();iter!=unvalid_slots_.end();++iter){
                //   delete_unvalid_slots_id = iter->first;
                // }
                unvalid_slots_.erase(unvalid_slots_.begin());
              }
              ADEBUG << "Map-perception obstacle search unvalid parking id:" << parking->id().id();
              unvalid_slots_.push_back(parking->id().id());
            }
          }

        }else
        {
          ADEBUG << "No ParkingSpaces Searched by perception obstacles!";
        }
    }
  }





    //判断哪些车位在vectors包含的矩形框中
    void SlotCheckProcessSubnode::SlotJudge(const std::vector<jmc_auto::hdmap::ParkingSpaceInfoConstPtr> &parking_spaces, const Eigen::Vector3d &point_a, const Eigen::Vector3d &point_b, const Eigen::Vector3d &point_c, const Eigen::Vector3d &point_d)
    {
      //The heading is zero when the car is facing East and positive when facing North.注意是弧度
      double vehicle_heading=localization_.pose().heading();
      for (auto& parking : parking_spaces)
      {
        //跳过有检测结果的车位
        auto it = std::find(unvalid_slots_.begin(), unvalid_slots_.end(), parking->id().id());
        // if((slots_.find(parking->id().id()) != slots_.end()) || it != unvalid_slots_.end()){
        if((Judge_vector_pair_id(slots_,parking->id().id())) || it != unvalid_slots_.end()){
          continue;
        }
        double x0,y0,x1,y1,x2,y2,x3,y3;				

        bool slot_to_check_bool = false;//后面为true了说明用的是环视的
        
        // auto iter=slots_to_check_.find(parking->id().id());

        for (auto iter = slots_to_check_.begin();iter != slots_to_check_.end();++iter){
          if (iter->first==parking->id().id()){ //说明感知里有待识别的库位，优先用感知的，感知没有再用地图
            auto slot_to_check = iter->second;
            x0 = slot_to_check.polygon_v().point(0).x();				
            y0 = slot_to_check.polygon_v().point(0).y();
            x1 = slot_to_check.polygon_v().point(1).x();				
            y1 = slot_to_check.polygon_v().point(1).y();
            x2 = slot_to_check.polygon_v().point(2).x();				
            y2 = slot_to_check.polygon_v().point(2).y();
            x3 = slot_to_check.polygon_v().point(3).x();				
            y3 = slot_to_check.polygon_v().point(3).y();
            slot_to_check_bool = true;
            break;
          }
        }

        if(!slot_to_check_bool){//用地图
          x0 = parking->polygon().points()[0].x(); 
          y0 = parking->polygon().points()[0].y();
          x1 = parking->polygon().points()[1].x();
          y1 = parking->polygon().points()[1].y();
          x2 = parking->polygon().points()[2].x();
          y2 = parking->polygon().points()[2].y();
          x3 = parking->polygon().points()[3].x();
          y3 = parking->polygon().points()[3].y();

        }


        // if (iter!=slots_to_check_.end())//说明感知里有待识别的库位，优先用感知的，感知没有再用地图
        // {
          
        //   auto slot_to_check = iter->second; 
        //   x0 = slot_to_check.polygon_v().point(0).x();				
        //   y0 = slot_to_check.polygon_v().point(0).y();
        //   x1 = slot_to_check.polygon_v().point(1).x();				
        //   y1 = slot_to_check.polygon_v().point(1).y();
        //   x2 = slot_to_check.polygon_v().point(2).x();				
        //   y2 = slot_to_check.polygon_v().point(2).y();
        //   x3 = slot_to_check.polygon_v().point(3).x();				
        //   y3 = slot_to_check.polygon_v().point(3).y();
        //   slot_to_check_bool = true;
        // }else{
        //   //提取车位0,1,2,3角点坐标
        //   x0 = parking->polygon().points()[0].x(); 
        //   y0 = parking->polygon().points()[0].y();
        //   x1 = parking->polygon().points()[1].x();
        //   y1 = parking->polygon().points()[1].y();
        //   x2 = parking->polygon().points()[2].x();
        //   y2 = parking->polygon().points()[2].y();
        //   x3 = parking->polygon().points()[3].x();
        //   y3 = parking->polygon().points()[3].y();
        // }

        //构建车位0,1,2,3四个点
        jmc_auto::common::math::Vec2d point_0(x0, y0);
        jmc_auto::common::math::Vec2d point_1(x1, y1);
        // jmc_auto::common::math::Vec2d point_2(x2, y2);
        jmc_auto::common::math::Vec2d point_3(x3, y3);
        jmc_auto::common::math::Vec2d search_box_corner(point_b[0], point_b[1]);  //构建搜索框
        jmc_auto::common::math::Vec2d search_box_opposite_corner(point_d[0], point_d[1]);

        jmc_auto::common::math::Box2d search_box = jmc_auto::common::math::Box2d::CreateAABox(search_box_corner, search_box_opposite_corner);
        //将库位的0号和3号点组成一条线段，求这条线段与车辆航向角的夹角
        //如果夹角接近垂直，则是水平库位,用来做search_box.IsPointIn的是0号点和1号点()
        
        common::math::LineSegment2d slot_linesegment(point_0, point_3);
        double slot_0_3_heading = slot_linesegment.heading();
        double heading_judge;
        if(slot_0_3_heading*vehicle_heading >0)
        {
          heading_judge = std::abs(slot_0_3_heading-vehicle_heading);
        }else
        {
            if(slot_0_3_heading <0)
          {
            heading_judge = std::abs(slot_0_3_heading+M_PI-vehicle_heading);
          }else
          {
            heading_judge = std::abs(vehicle_heading+M_PI-slot_0_3_heading);
          }
        }
        if(heading_judge>M_PI/2)
        {
          heading_judge=M_PI-heading_judge;
        }
        //如果夹角很小，则说明车辆航向和0-3号线段接近平行，说明是垂直库位/斜向库位
        //如果夹角很大（接近Π/2），则说明车辆航向和0-3号线段接近垂直，说明是平行库位,平行库位靠近车辆的两个点是0号和1号点
        ADEBUG <<std::setprecision(10)<<"heading_judge: "<<heading_judge;
        ADEBUG << "id: " << parking->id().id();
        bool is_point_0_in;
        bool is_point_3_in;
        if (heading_judge >1)   //1大概是57度
        {
          is_point_0_in = search_box.IsPointIn(point_0);
          is_point_3_in = search_box.IsPointIn(point_1);
        }else
        {
          is_point_0_in = search_box.IsPointIn(point_0);
          // bool is_point_1_in = search_box.IsPointIn(point_1);
          // bool is_point_2_in = search_box.IsPointIn(point_2);
          is_point_3_in = search_box.IsPointIn(point_3);
        }
        
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
          if(slots_.size() >= 10){
            // auto slots_end = slots_.end();
            // slots_end--;
            // std::string delete_slots_id;
            // for (auto iter = slots_.begin();iter!=slots_.end();++iter){
            //   delete_slots_id = iter->first;
            // }
            // slots_.erase(delete_slots_id);
            slots_.pop_back();
          }
          // slots_.emplace(parking->id().id(), slot);
          slots_.insert(slots_.begin(),std::make_pair(parking->id().id(),slot));
          if(slot_to_check_bool)//用的是感知的坐标
          {
            ADEBUG<<"SlotJudge add slots_(det):"<<parking->id().id();
          }else//用的是地图的坐标
          {
            ADEBUG<<"SlotJudge add slots_(map):"<<parking->id().id();
          }

          //如果用的是环视的库位，则
          if (slot_to_check_bool)
          {
            // slots_to_check_.erase(iter);
            slots_to_check_ = Delete_vector_pair_id(slots_to_check_,parking->id().id());

          }
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

	//从环视给出的车位vector中，选出一个在车辆正右方的车位，该车位的坐标精度最高
	// bool SlotCheckProcessSubnode::FindRightSideSlot(const jmc_auto::perception::MulvalidSlot& mulvalidslot, unsigned int* choosen_index)
	// {
  //   ADEBUG << "slotcheck : FindRightSideSlot...";
	// 	if(mulvalidslot.slotsum_size() < 1)
	// 	{
	// 		ADEBUG << "perception slots num < 1";
	// 		return false;
	// 	}
	// 	double distance_to_center = 1.543;	
	// 	common::math::Vec2d vehicle_center_point(localization_.pose().position().x() + distance_to_center * std::cos(localization_.pose().heading()), 
	// 					         localization_.pose().position().y() + distance_to_center * std::sin(localization_.pose().heading()));
	// 	unsigned int index = 0;
	// 	for(auto slot : mulvalidslot.slotsum())
	// 	{
	// 		//构建从车位角点0指向角点3的线段
	// 		common::math::Vec2d point0(slot.polygon_v().point(0).x(), slot.polygon_v().point(0).y());
	// 		common::math::Vec2d point3(slot.polygon_v().point(3).x(), slot.polygon_v().point(3).y());
	// 		common::math::Vec2d point_center = (point0 + point3) / 2.0;		
	// 		common::math::LineSegment2d slot_linesegment(point0, point3);
	// 		common::math::LineSegment2d center_to_center_linesegment(vehicle_center_point, point_center);
			
	// 		//间接求出两个线段的夹角，当该值小于0.3时，可以认为车辆在车位的正前方
	// 		double segment_angle = std::fabs(slot_linesegment.unit_direction().InnerProd(center_to_center_linesegment.unit_direction()));
	// 		if(segment_angle < 0.3) 
	// 		{
	// 			*choosen_index = index;	
	// 			return true;
	// 		}
	// 		++index;
	// 	}
	// 	return false;
	// }

  bool SlotCheckProcessSubnode::GetSharedData(const Event &event, std::shared_ptr<SlotObjects> *slots)
  {
    ADEBUG << "slotcheck : GetSharedData...";
    double timestamp = event.timestamp;
    const std::string &device_id = event.reserve;
    std::string data_key;
    if(!SubnodeHelper::ProduceSharedDataKey(timestamp, device_id, &data_key))
    {
      AERROR << "Failed to produce shared data key. EventID : " << event.event_id 
              << " timestamp : " << timestamp << " device_id : " << device_id;
    }
    bool get_data_succ = false;
    if(event.event_id == slot_event_id_ && slot_shared_data_ != nullptr)
    {
      get_data_succ = slot_shared_data_->Get(data_key, slots);
      if(!get_data_succ)
      {
        ADEBUG << "slotcheck : ----------Failed to get shared data. event : " << event.to_string();
        return false;
      }else
      {
        ADEBUG << "slotcheck : get shared data size : ";
        int count = 0;
        for(auto& s : (*slots)->objects)
        {
          ADEBUG << "slotcheck : get shared data size : " << (*slots)->objects.size() << " no:" << count << " slot id : " <<  s.id_v().id();
          ++count; 
        }
      }
    }else
    {
      ADEBUG << "slotcheck : ----------event id not matched!";
    }
    return true;
  }

  bool SlotCheckProcessSubnode::InitOutputStream()
  {
    ADEBUG << "slotcheck : InitOutputStream...";
    std::unordered_map<std::string, std::string> reserve_field_map;
    if(!SubnodeHelper::ParseReserveField(reserve_, &reserve_field_map))
    {
      AERROR << "Failed to parse reserve string : " << reserve_;
      return false;
    }
    auto slot_iter = reserve_field_map.find("slot_event_id");
    if(slot_iter == reserve_field_map.end())
    {
      AWARN << "Failed to find slot_event_id : " << reserve_;
      AINFO << "slot_event_id will be set -1";
      slot_event_id_ = -1;
    }else
    {
      slot_event_id_ = static_cast<EventID>(atoi((slot_iter->second).c_str()));
    }
    return true;
  }

  Status SlotCheckProcessSubnode::ProcEvents()
  {
    //ADEBUG << "slotcheck : ProcEvents...";
    for(auto event_meta : sub_meta_events_)
    {
      std::vector<Event> events;
      if(!SubscribeEvents(event_meta, &events))
      {
        AERROR << "event meta id : " << event_meta.event_id << " from : " << event_meta.from_node << " to : " << event_meta.to_node;
        return Status(ErrorCode::PERCEPTION_ERROR, "Subscribe event fail.");
      }
      if(events.empty())
      {
        continue;
      }
      for(auto event : events)
      {
        
        if(!GetSharedData(event, &roundview_slots_))
        {
          ADEBUG << "slotcheck : ----------failed to get shared data.";
          return Status(ErrorCode::PERCEPTION_ERROR, "failed to get shared data!");
        }
      }
    }
    return Status::OK();
  }

  bool SlotCheckProcessSubnode::SubscribeEvents(const EventMeta &event_meta, std::vector<Event> *events) const
  {
    //ADEBUG << "slotcheck : SubscribeEvents...";
    Event event;
    //non blocking
    while(event_manager_->Subscribe(event_meta.event_id, &event, true))
    {
      events->push_back(event);
    }
    return true;
  }

    void SlotCheckProcessSubnode::PublishValidSlot(void)
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
