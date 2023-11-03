/******************************************************************************
 * Copyright 2019 The jmc_auto Authors. All Rights Reserved.
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

/**
 * @file
 **/

#include "modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_provider.h"

#include <string>

//#include "cyber/task/task.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/common/trajectory_stitcher.h"

namespace jmc_auto {
namespace planning {

using jmc_auto::common::ErrorCode;
using jmc_auto::common::Status;
using jmc_auto::common::TrajectoryPoint;
using jmc_auto::common::math::Vec2d;
using jmc_auto::common::time::Clock;

OpenSpaceTrajectoryProvider::OpenSpaceTrajectoryProvider(
    const TaskConfig& config)
    : TrajectoryOptimizer(config) {
  open_space_trajectory_optimizer_.reset(new OpenSpaceTrajectoryOptimizer(
      config.open_space_trajectory_provider_config()
          .open_space_trajectory_optimizer_config()));
}

OpenSpaceTrajectoryProvider::~OpenSpaceTrajectoryProvider() {
  if (FLAGS_enable_open_space_planner_thread) {
    Stop();
  }
}

void OpenSpaceTrajectoryProvider::Stop() {
  if (FLAGS_enable_open_space_planner_thread) {
    is_generation_thread_stop_.store(true);
    if (thread_init_flag_) {
      thread_->joinable();
      thread_->join();
      //task_future_.get();
    }
    trajectory_updated_.store(false);
    trajectory_error_.store(false);
    trajectory_skipped_.store(false);
    optimizer_thread_counter = 0;
  }
}

void OpenSpaceTrajectoryProvider::Restart() {
  if (FLAGS_enable_open_space_planner_thread) {
    is_generation_thread_stop_.store(true);
    if (thread_init_flag_) {
      thread_->joinable();
      thread_->join();
      //task_future_.get();
    }
    is_generation_thread_stop_.store(false);
    thread_init_flag_ = false;
    trajectory_updated_.store(false);
    trajectory_error_.store(false);
    trajectory_skipped_.store(false);
    optimizer_thread_counter = 0;
  }
}

Status OpenSpaceTrajectoryProvider::Process() {
  ADEBUG << "trajectory provider";
  auto trajectory_data =
      frame_->mutable_open_space_info()->mutable_stitched_trajectory_result();

  // generate stop trajectory at park_and_go check_stage
  if (PlanningContext::Instance()
          ->mutable_planning_status()
          ->mutable_park_and_go()
          ->in_check_stage()) {
    ADEBUG << "ParkAndGo Stage Check.";
    GenerateStopTrajectory(trajectory_data);
    return Status::OK();
  }
  // Start thread when getting in Process() for the first time
  if (FLAGS_enable_open_space_planner_thread && !thread_init_flag_) {
    thread_.reset(
        new std::thread(&OpenSpaceTrajectoryProvider::GenerateTrajectoryThread, this));
    // task_future_ = cyber::Async(
    //     &OpenSpaceTrajectoryProvider::GenerateTrajectoryThread, this);
     thread_init_flag_ = true;
  }
  // Get stitching trajectory from last frame
  const common::VehicleState vehicle_state = frame_->vehicle_state();
  auto* previous_frame = FrameHistory::Instance()->Latest();
  frame_->set_current_partitioned(previous_frame->current_partitioned().first, previous_frame->current_partitioned().second);
  //NEW add
//  if(vehicle_state.gear() == 2){
//    current_gear = false;
//  }
  //判断是否是泊出模式
  bool current_mode = (frame_->local_view().pad_msg->appmode() == 4);
  // Use complete raw trajectory from last frame for stitching purpose
  std::vector<TrajectoryPoint> stitching_trajectory;
  if (!IsVehicleStopDueToFallBack(
          previous_frame->open_space_info().fallback_flag(), vehicle_state)){
// && frame_->second_parking()) {
    const auto& previous_planning =
        previous_frame->open_space_info().stitched_trajectory_result();
    const auto& previous_planning_header =
        previous_frame->current_frame_planned_trajectory()
            .header()
            .timestamp_sec();
    const double planning_cycle_time = FLAGS_open_space_planning_period;
    PublishableTrajectory last_frame_complete_trajectory(
        previous_planning_header, previous_planning);
    std::string replan_reason;
    const double start_timestamp = Clock::NowInSeconds();
    stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
        vehicle_state, start_timestamp, planning_cycle_time,
        FLAGS_open_space_trajectory_stitching_preserved_length, false,
        &last_frame_complete_trajectory, &replan_reason);
  } else {
    ADEBUG << "Replan due to fallback stop";
    const double planning_cycle_time =
        1.0 / static_cast<double>(FLAGS_planning_loop_rate);
    stitching_trajectory = TrajectoryStitcher::ComputeReinitStitchingTrajectory(
        planning_cycle_time, vehicle_state);
    auto* open_space_status = PlanningContext::Instance()
                                  ->mutable_planning_status()
                                  ->mutable_open_space();
    open_space_status->set_position_init(false);
//    frame_->mutable_open_space_info()->set_open_space_provider_success(false);
  }
  // Get open_space_info from current frame
  const auto& open_space_info = frame_->open_space_info();

    //NEW ADD
     std::vector<double> end_pose_to_world;
    // double parking_depth = std::abs(frame_->local_view().pad_msg->parking_space_info().point_left_up().y() - frame_->local_view().pad_msg->parking_space_info().point_left_down().y());


  if (FLAGS_enable_open_space_planner_thread) {
    ADEBUG << "Open space plan in multi-threads mode";

    if (is_generation_thread_stop_) {
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK, "Parking finished");
    }
   //else{
   //   is_generation_thread_stop_.store(false);
   // }

    {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      thread_data_.stitching_trajectory = stitching_trajectory;
      thread_data_.end_pose = open_space_info.open_space_end_pose();
      thread_data_.rotate_angle = open_space_info.origin_heading();
      thread_data_.translate_origin = open_space_info.origin_point();
      thread_data_.obstacles_edges_num = open_space_info.obstacles_edges_num();
      thread_data_.obstacles_A = open_space_info.obstacles_A();
      thread_data_.obstacles_b = open_space_info.obstacles_b();
      thread_data_.obstacles_vertices_vec =
          open_space_info.obstacles_vertices_vec();
      thread_data_.XYbounds = open_space_info.ROI_xy_boundary();
      thread_data_.mode = current_mode;
      data_ready_.store(true);
    }

//    if(is_generation_thread_stop_ && IsDestination(vehicle_state, open_space_info.open_space_end_pose(),
//            open_space_info.origin_heading())) {
//      GenerateStopTrajectory(trajectory_data);
//      return Status(ErrorCode::OK, "Parking finished");
//    }else{
//      is_generation_thread_stop_.store(false);
//    }

    // Check vehicle state
    if (IsVehicleNearDestination(
            vehicle_state, open_space_info.open_space_end_pose(),
            open_space_info.origin_heading(), open_space_info.origin_point(),
            &end_pose_to_world)) {
      GenerateStopTrajectory(trajectory_data);
      if(frame_->local_view().pad_msg->appmode() != planning::SUMMON_INPARKING &&vehicle_state.linear_velocity() < 0.01){
        frame_->set_parking_status(planning::SUCCEED);
        ADEBUG << "SUCCEED";
      }
      //is_generation_thread_stop_.store(true);
      frame_->mutable_open_space_info()->set_open_space_provider_success(false);
      return Status(ErrorCode::OK, "Vehicle is near to destination");
    }

    //NEW ADD  添加剩余直线轨迹
    if(is_straightline_trajectory_){
      GenerateStraightLineTrajectory(trajectory_data, end_pose_to_world);
      return Status::OK();
    }

   // if(is_secondend_trajectory_){
   //   GenerateStopTrajectory(trajectory_data);
   //   return Status(ErrorCode::OK, "Second the First End");
   // }


    // Check if trajectory updated
    if (trajectory_updated_) {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      LoadResult(trajectory_data);
      
      //新增打印
      for(const auto& p : *trajectory_data)
      {
        ADEBUG << "test DiscretizedTrajectory : ";
        ADEBUG << p.DebugString();
      }
      //处理短距离泊出


      if (FLAGS_enable_record_debug) {
        // call merge debug ptr, open_space_trajectory_optimizer_
        auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
        open_space_trajectory_optimizer_->UpdateDebugInfo(
            ptr_debug->mutable_planning_data()->mutable_open_space());

        // sync debug instance
        frame_->mutable_open_space_info()->sync_debug_instance();
      }
      data_ready_.store(false);
      trajectory_updated_.store(false);
      return Status::OK();
    }

    if (trajectory_error_) {
      ++optimizer_thread_counter;
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      trajectory_error_.store(false);
      // TODO(Jinyun) Use other fallback mechanism when last iteration smoothing
      // result has out of bound pathpoint which is not allowed for next
      // iteration hybrid astar algorithm which requires start position to be
      // strictly in bound
      if (optimizer_thread_counter > 1000) {
        frame_->set_parking_status(planning::FAILED);
        ADEBUG << "PARKING FAILED";
        return Status(ErrorCode::PLANNING_ERROR,
                      "open_space_optimizer failed too many times");
      }
    }

    if (previous_frame->open_space_info().open_space_provider_success()) {
      ReuseLastFrameResult(previous_frame, trajectory_data);
      if (FLAGS_enable_record_debug) {
        // copy previous debug to current frame
        ReuseLastFrameDebug(previous_frame);
      }
      // reuse last frame debug when use last frame traj
      return Status(ErrorCode::OK,
                    "Waiting for open_space_trajectory_optimizer in "
                    "open_space_trajectory_provider");
    } else {
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK, "Stop due to computation not finished");
    }
  } else {
    const auto& end_pose = open_space_info.open_space_end_pose();
    const auto& rotate_angle = open_space_info.origin_heading();
    const auto& translate_origin = open_space_info.origin_point();
    const auto& obstacles_edges_num = open_space_info.obstacles_edges_num();
    const auto& obstacles_A = open_space_info.obstacles_A();
    const auto& obstacles_b = open_space_info.obstacles_b();
    const auto& obstacles_vertices_vec =
        open_space_info.obstacles_vertices_vec();
    const auto& XYbounds = open_space_info.ROI_xy_boundary();

    // Check vehicle state
    if (IsVehicleNearDestination(vehicle_state, end_pose, rotate_angle,
                                 translate_origin, &end_pose_to_world)) {
      GenerateStopTrajectory(trajectory_data);
      ADEBUG << "dest replan: vehicle reach destination.";
      if(frame_->local_view().pad_msg->appmode() != planning::SUMMON_INPARKING && vehicle_state.linear_velocity() < 0.01){
        frame_->set_parking_status(planning::SUCCEED);
        ADEBUG << "SUCCEED";
      }else if((frame_->local_view().pad_msg->appmode() == planning::SUMMON_INPARKING) && (vehicle_state.linear_velocity() < 0.01))
      {
        frame_->set_parking_status(planning::SUCCEED);
        ADEBUG << "short summon. SUCCEED";
      }else{}
      return Status(ErrorCode::OK, "Vehicle is near to destination");
    }

    // Generate Trajectory;
    double time_latency;
    //cjx New add
    if((frame_->borrow_parking() == false) && 
        ((frame_->local_view().pad_msg->appmode() == planning::PARKING) || (frame_->local_view().pad_msg->appmode() == planning::PARKING_NOID))){
        open_space_trajectory_optimizer_->set_need_change_end_node(frame_->parking_type() == planning::Parking_Type::VERTICAL);
    }else{
        open_space_trajectory_optimizer_->set_need_change_end_node(false);
    }

    Status status = open_space_trajectory_optimizer_->Plan(
        stitching_trajectory, end_pose, XYbounds, rotate_angle,
        translate_origin, obstacles_edges_num, obstacles_A, obstacles_b,
        obstacles_vertices_vec, current_mode,  &time_latency);
    frame_->mutable_open_space_info()->set_time_latency(time_latency);

    // If status is OK, update vehicle trajectory;
    if (status == Status::OK()) {
      LoadResult(trajectory_data);

      //处理短距离泊出
      DealWithShortSummon(trajectory_data);
      //新增短距离泊出轨迹点打印
      /*for(const auto& p : *trajectory_data)
      {
        ADEBUG << "test short_summon. DiscretizedTrajectory after cut : ";
        ADEBUG << p.DebugString();
      }*/
      ADEBUG << "short summon. end pose : " << end_pose[0] << " " << end_pose[1] << " " << end_pose[2] << " " << end_pose[3];
      return status;
    } else {
//      if(open_space_trajectory_optimizer_->IsStartNodeCollison()){
//        frame_->set_parking_start_collision(true);
//      }
      ADEBUG << "short summon. else branch.";
      LoadResult(trajectory_data);
      return status;
    }
  }
  return Status(ErrorCode::PLANNING_ERROR);
}

void OpenSpaceTrajectoryProvider::GenerateTrajectoryThread() {
  while (!is_generation_thread_stop_) {
    if (!trajectory_updated_ && data_ready_) {
      OpenSpaceTrajectoryThreadData thread_data;
      {
        std::lock_guard<std::mutex> lock(open_space_mutex_);
        thread_data = thread_data_;
      }
      double time_latency;
      Status status = open_space_trajectory_optimizer_->Plan(
          thread_data.stitching_trajectory, thread_data.end_pose,
          thread_data.XYbounds, thread_data.rotate_angle,
          thread_data.translate_origin, thread_data.obstacles_edges_num,
          thread_data.obstacles_A, thread_data.obstacles_b,
          thread_data.obstacles_vertices_vec, thread_data.mode,  &time_latency);
      frame_->mutable_open_space_info()->set_time_latency(time_latency);
      if (status == Status::OK()) {
        std::lock_guard<std::mutex> lock(open_space_mutex_);
        trajectory_updated_.store(true);
      } else {
        if (status.ok()) {
          std::lock_guard<std::mutex> lock(open_space_mutex_);
          trajectory_skipped_.store(true);
        } else {
//          if(open_space_trajectory_optimizer_->IsStartNodeCollison()){
//            frame_->set_parking_start_collision(true);
//          }
          std::lock_guard<std::mutex> lock(open_space_mutex_);
          trajectory_error_.store(true);
        }
      }
    }
  }
}


//bool OpenSpaceTrajectoryProvider::IsDestination(
//    const common::VehicleState& vehicle_state,
//    const std::vector<double>& end_pose, double rotate_angle) {
//  CHECK_EQ(end_pose.size(), 4);
//  Vec2d end_pose_to_world_frame = Vec2d(end_pose[0], end_pose[1]);
//
//  end_pose_to_world_frame.SelfRotate(rotate_angle);
//  end_pose_to_world_frame += translate_origin;

//  double end_theta_to_world_frame = end_pose[2];
//  end_theta_to_world_frame += rotate_angle;

//  double distance_to_vehicle =
//      std::sqrt((vehicle_state.x() - end_pose_to_world_frame.x()) *
//                    (vehicle_state.x() - end_pose_to_world_frame.x()) +
//                (vehicle_state.y() - end_pose_to_world_frame.y()) *
//                    (vehicle_state.y() - end_pose_to_world_frame.y()));
//  return distance_to_vehicle < 1 ? true : false; 
//
//}

bool OpenSpaceTrajectoryProvider::IsVehicleNearDestination(
    const common::VehicleState& vehicle_state,
    const std::vector<double>& end_pose, double rotate_angle,
    const Vec2d& translate_origin, std::vector<double>* end_pose_to_world) {
  CHECK_EQ(end_pose.size(), 4);
  if(frame_->local_view().pad_msg->appmode() == planning::SUMMON_INPARKING)
  {
    auto summon_x = frame_->local_view().pad_msg->point().end().x();
    auto summon_y = frame_->local_view().pad_msg->point().end().y();
    
    //Vec2d summon_point_ref = Vec2d(summon_x, summon_y);
    //summon_point_ref -= translate_origin;
    //summon_point_ref.SelfRotate(-rotate_angle);

    //Vec2d vehicle_point_ref = Vec2d(vehicle_state.x(), vehicle_state.y());
    //vehicle_point_ref -= translate_origin;
    //vehicle_point_ref.SelfRotate(-rotate_angle);
 
    //bool is_horizontal = (std::abs(rotate_angle) > (M_PI / 2.5)) && (std::abs(rotate_angle) < (M_PI / 1.8));
    //double short_summon_distance_to_vehicle =
    //    std::sqrt((vehicle_state.x() - summon_x) * (vehicle_state.x() - summon_x) +
    //            (vehicle_state.y() - summon_y) * (vehicle_state.y() - summon_y));
    //double rest_len = 0.0;
    //if(is_horizontal)
    //{
      //rest_len = std::fabs(summon_point_ref.y() - vehicle_point_ref.y() - 3.88);
    //}else
    //{
      //rest_len = std::fabs(summon_point_ref.x() - vehicle_point_ref.x());
    //}

    auto* previous_frame = FrameHistory::Instance()->Latest();
    if(!previous_frame)
    {
      return false;
    }
    auto previous_trajectory = previous_frame->open_space_info().stitched_trajectory_result();
    if(previous_trajectory.size() == 0)
    {
      return false;
    }
    auto trajectory_last_point = previous_trajectory.back();
    auto last_point_x = trajectory_last_point.path_point().x();
    auto last_point_y = trajectory_last_point.path_point().y();
    double short_summon_distance_to_vehicle =
                 std::sqrt((vehicle_state.x() - last_point_x) * (vehicle_state.x() - last_point_x) +
                (vehicle_state.y() - last_point_y) * (vehicle_state.y() - last_point_y));

    ADEBUG << "short summon. summon_x : " << std::setprecision(9) << summon_x << " summon_y : " << summon_y;
    ADEBUG << "short summon. vehicle_x : " << std::setprecision(9) << vehicle_state.x() << " vehicle_y : " << vehicle_state.y();
    //ADEBUG << "short summon. ABS delx: " << std::setprecision(9) << summon_x - vehicle_state.x() << " dely : " << summon_y - vehicle_state.y();
    //ADEBUG << "short summon. REF delx: " << std::setprecision(9) << summon_point_ref.x() - vehicle_point_ref.x() << " dely : " << summon_point_ref.y() - vehicle_point_ref.y();
    ADEBUG << "short summon. trajectory last x : " << std::setprecision(9) << last_point_x << " last y : " << last_point_y << " distance : " << short_summon_distance_to_vehicle;
    //ADEBUG << "short summon. short_summon_distance_to_vehicle: " << short_summon_distance_to_vehicle;
    ADEBUG << "short summon. vehicle speed : " << vehicle_state.linear_velocity() << " truncated : " << is_trajectory_truncated_;
   
    /*
    if(std::fabs(summon_point_ref.x() - vehicle_point_ref.x()) < config_.open_space_trajectory_provider_config()
                                .open_space_trajectory_optimizer_config()
                                .planner_open_space_config()
                                .is_near_destination_threshold())
    */
    if(short_summon_distance_to_vehicle < config_.open_space_trajectory_provider_config()
                  .open_space_trajectory_optimizer_config()
                  .planner_open_space_config()
                  .is_near_destination_threshold() && vehicle_state.linear_velocity() < 0.01 && is_trajectory_truncated_)
    
    {
      ADEBUG << "short summon. vehicle reach end point.";
      frame_->mutable_open_space_info()->set_destination_reached(true);
      return true;
    }
  }

  Vec2d end_pose_to_world_frame = Vec2d(end_pose[0], end_pose[1]);

  end_pose_to_world_frame.SelfRotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;

  double end_theta_to_world_frame = end_pose[2];
  end_theta_to_world_frame += rotate_angle;

  double distance_to_vehicle =
      std::sqrt((vehicle_state.x() - end_pose_to_world_frame.x()) *
                    (vehicle_state.x() - end_pose_to_world_frame.x()) +
                (vehicle_state.y() - end_pose_to_world_frame.y()) *
                    (vehicle_state.y() - end_pose_to_world_frame.y()));

ADEBUG << "end_pose_to_world_frame.(x): " << std::setprecision(9) << end_pose_to_world_frame.x();
ADEBUG << "end_pose_to_world_frame.(y): " << std::setprecision(9) << end_pose_to_world_frame.y();
ADEBUG << "distance_to_vehicle: " << distance_to_vehicle;

  double theta_to_vehicle = std::abs(common::math::AngleDiff(
      vehicle_state.heading(), end_theta_to_world_frame));
  ADEBUG << "theta_to_vehicle" << theta_to_vehicle << "end_theta_to_world_frame"
         << end_theta_to_world_frame << "rotate_angle" << rotate_angle;
  ADEBUG << "is_near_destination_threshold"
         << config_.open_space_trajectory_provider_config()
                .open_space_trajectory_optimizer_config()
                .planner_open_space_config()
                .is_near_destination_threshold();  // which config file
  ADEBUG << "is_near_destination_theta_threshold"
         << config_.open_space_trajectory_provider_config()
                .open_space_trajectory_optimizer_config()
                .planner_open_space_config()
                .is_near_destination_theta_threshold();
  
  //NEW ADD 增加下面这个if  第一段泊车时不需要判断终点
  //if(frame_->second_parking()){
  //   if(theta_to_vehicle < 0.2 && distance_to_vehicle < 0.5){
  //    frame_->set_second_parking(false);
  //    //frame_->mutabqse_open_space_info()->set_fallback_flag(true);
  //    is_secondend_trajectory_ = true;
  //    ADEBUG << "Complete first trajectory";
  //   }
  //}
  //else{

   if (distance_to_vehicle < config_.open_space_trajectory_provider_config()
                                .open_space_trajectory_optimizer_config()
                                .planner_open_space_config()
                                .is_near_destination_threshold() &&
      theta_to_vehicle < config_.open_space_trajectory_provider_config()
                             .open_space_trajectory_optimizer_config()
                             .planner_open_space_config()
                             .is_near_destination_theta_threshold() && vehicle_state.linear_velocity() < 0.01) {
    ADEBUG << "vehicle reach end_pose";
    frame_->mutable_open_space_info()->set_destination_reached(true);
 //   frame_->set_parking_callback(false);
    frame_->set_second_parking(true);

    if(frame_->destination_replan()){
      frame_->set_destination_replan(false);
    }
    return true;
  }
//NEW ADD
//  else if(theta_to_vehicle < 0.03 &&
//          std::abs(vehicle_state.x() - end_pose_to_world_frame.x()) < 0.3){
//        end_pose_to_world->clear();
//        end_pose_to_world->push_back(end_pose_to_world_frame.x());
//        end_pose_to_world->push_back(end_pose_to_world_frame.y());
//        end_pose_to_world->push_back(end_theta_to_world_frame);
//        is_straightline_trajectory_ = true;
//        ADEBUG << "is_straightline_trajectory:" << is_straightline_trajectory_;
//  }
 //} 
  return false;
}

bool OpenSpaceTrajectoryProvider::IsVehicleStopDueToFallBack(
    const bool is_on_fallback, const common::VehicleState& vehicle_state) {
  if (!is_on_fallback) {
    return false;
  }
  static constexpr double kEpsilon = 1.0e-1;// new add -1
  const double adc_speed = vehicle_state.linear_velocity();
 // const double adc_acceleration = vehicle_state.linear_acceleration();
  if (std::abs(adc_speed) < kEpsilon) {
    ADEBUG << "ADC stops due to fallback trajectory";
    return true;
  }
  return false;
}

void OpenSpaceTrajectoryProvider::GenerateStopTrajectory(
    DiscretizedTrajectory* const trajectory_data) {
  double relative_time = 0.0;
  // TODO(Jinyun) Move to conf
  static constexpr int stop_trajectory_length = 10;
  static constexpr double relative_stop_time = 0.1;
//  static constexpr double vEpsilon = 0.00001;
  double standstill_acceleration =
        frame_->vehicle_state().gear() == canbus::Chassis::GEAR_REVERSE
//      frame_->vehicle_state().linear_velocity() >= -vEpsilon
          ? FLAGS_open_space_standstill_acceleration
          : -FLAGS_open_space_standstill_acceleration;
  trajectory_data->clear();
  for (size_t i = 0; i < stop_trajectory_length; i++) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(relative_time);
    point.set_v(0.0);
    point.set_a(standstill_acceleration);
    trajectory_data->emplace_back(point);
    relative_time += relative_stop_time;
  }
}


//NEW ADD
void OpenSpaceTrajectoryProvider::GenerateStraightLineTrajectory(
    DiscretizedTrajectory* const trajectory_data,
    const std::vector<double>& end_pose_to_world) {
  double relative_time = 0.0;
  // TODO(Jinyun) Move to conf
  double trajectory_length = std::abs(frame_->vehicle_state().y() - end_pose_to_world[1]);
  ADEBUG << "straight line length:" << trajectory_length;
  relative_time = trajectory_length / frame_->vehicle_state().linear_velocity();
  double heading_angle = frame_->vehicle_state().heading();
  const Vec2d init_tracking_vector(
      end_pose_to_world[0] - frame_->vehicle_state().x(),
      end_pose_to_world[1] - frame_->vehicle_state().y());
  double tracking_angle = init_tracking_vector.Angle();
  auto gear =
      std::abs(common::math::NormalizeAngle(tracking_angle - heading_angle)) <
              (M_PI_2)
          ? canbus::Chassis::GEAR_DRIVE
          : canbus::Chassis::GEAR_REVERSE;
  //static constexpr double vEpsilon = 0.00001;
  double standstill_acceleration =
      gear == canbus::Chassis::GEAR_REVERSE
//      frame_->vehicle_state().linear_velocity() >= -vEpsilon
          ? -FLAGS_open_space_standstill_acceleration
          : FLAGS_open_space_standstill_acceleration;
  trajectory_data->clear();
  TrajectoryPoint point;
  point.mutable_path_point()->set_x(frame_->vehicle_state().x());
  point.mutable_path_point()->set_y(frame_->vehicle_state().y());
  point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
  point.mutable_path_point()->set_s(0.0);
  point.mutable_path_point()->set_kappa(0.0);
  point.set_relative_time(0.0);
  point.set_v(gear == canbus::Chassis::GEAR_REVERSE ? -frame_->vehicle_state().linear_velocity() : frame_->vehicle_state().linear_velocity());
  point.set_a(standstill_acceleration);
  trajectory_data->emplace_back(point);

  // for (size_t i = 0; i < stop_trajectory_length; i++) {
  //   TrajectoryPoint point;
  //   point.mutable_path_point()->set_x(frame_->vehicle_state().x());
  //   point.mutable_path_point()->set_y(frame_->vehicle_state().y());
  //   point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
  //   point.mutable_path_point()->set_s(0.0);
  //   point.mutable_path_point()->set_kappa(0.0);
  //   point.set_relative_time(relative_time);
  //   point.set_v(0.0);
  //   point.set_a(standstill_acceleration);
  //   trajectory_data->emplace_back(point);
  //   relative_time += relative_stop_time;
  // }


  TrajectoryPoint point_end;
  point_end.mutable_path_point()->set_x(end_pose_to_world[0]);
  point_end.mutable_path_point()->set_y(end_pose_to_world[1]);
  point_end.mutable_path_point()->set_theta(end_pose_to_world[2]);
  point_end.mutable_path_point()->set_s(gear == canbus::Chassis::GEAR_REVERSE ? -trajectory_length : trajectory_length);
  point_end.mutable_path_point()->set_kappa(0.0);
  point_end.set_relative_time(relative_time);
  point_end.set_v(0.0);
  point_end.set_a(0.0);
  trajectory_data->emplace_back(point_end);
}

void OpenSpaceTrajectoryProvider::LoadResult(
    DiscretizedTrajectory* const trajectory_data) {
  // Load unstitched two trajectories into frame for debug
  trajectory_data->clear();
  auto optimizer_trajectory_ptr =
      frame_->mutable_open_space_info()->mutable_optimizer_trajectory_data();
  auto stitching_trajectory_ptr =
      frame_->mutable_open_space_info()->mutable_stitching_trajectory_data();
  open_space_trajectory_optimizer_->GetOptimizedTrajectory(
      optimizer_trajectory_ptr);
  open_space_trajectory_optimizer_->GetStitchingTrajectory(
      stitching_trajectory_ptr);
  // Stitch two trajectories and load back to trajectory_data from frame
  size_t optimizer_trajectory_size = optimizer_trajectory_ptr->size();
  double stitching_point_relative_time =
      stitching_trajectory_ptr->back().relative_time();
  double stitching_point_relative_s =
      stitching_trajectory_ptr->back().path_point().s();
  for (size_t i = 0; i < optimizer_trajectory_size; ++i) {
    optimizer_trajectory_ptr->at(i).set_relative_time(
        optimizer_trajectory_ptr->at(i).relative_time() +
        stitching_point_relative_time);
    optimizer_trajectory_ptr->at(i).mutable_path_point()->set_s(
        optimizer_trajectory_ptr->at(i).path_point().s() +
        stitching_point_relative_s);
  }
  *(trajectory_data) = *(optimizer_trajectory_ptr);

  // Last point in stitching trajectory is already in optimized trajectory, so
  // it is deleted
  frame_->mutable_open_space_info()
      ->mutable_stitching_trajectory_data()
      ->pop_back();
  trajectory_data->PrependTrajectoryPoints(
      frame_->open_space_info().stitching_trajectory_data());
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
  //frame_->set_is_last(open_space_trajectory_optimizer_->is_last());
  if(open_space_trajectory_optimizer_->current_partition().second){
    frame_->set_current_partitioned(open_space_trajectory_optimizer_->current_partition().first, open_space_trajectory_optimizer_->current_partition().second);
  }
  if(open_space_trajectory_optimizer_->destination_replan()){
    frame_->set_destination_replan(true);
  }
  //frame_->set_current_partitioned(open_space_trajectory_optimizer_->current_partition().first, open_space_trajectory_optimizer_->current_partition().second);
  frame_->set_check_end(false);
}

void OpenSpaceTrajectoryProvider::ReuseLastFrameResult(
    const Frame* last_frame, DiscretizedTrajectory* const trajectory_data) {
  *(trajectory_data) =
      last_frame->open_space_info().stitched_trajectory_result();
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
}

void OpenSpaceTrajectoryProvider::ReuseLastFrameDebug(const Frame* last_frame) {
  // reuse last frame's instance
  auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug_instance();
  ptr_debug->mutable_planning_data()->mutable_open_space()->MergeFrom(
      last_frame->open_space_info()
          .debug_instance()
          .planning_data()
          .open_space());
}

void OpenSpaceTrajectoryProvider::DealWithShortSummon(DiscretizedTrajectory* const trajectory_data)
{
  is_trajectory_truncated_ = false;
  if(frame_->local_view().pad_msg->appmode() != planning::SUMMON_INPARKING)
  {
    ADEBUG << "vehicle not in SUMMON_INPARKING.";
    return;
  }
  if(!frame_->open_space_info().need_truncate()){
    return;
  }

  auto vehicle_x = frame_->vehicle_state().x();
  auto vehicle_y = frame_->vehicle_state().y();
  auto last_point_x = trajectory_data->back().path_point().x();
  auto last_point_y = trajectory_data->back().path_point().y();
  if(((vehicle_x - last_point_x) * (vehicle_x - last_point_x) + (vehicle_y - last_point_y) * (vehicle_y - last_point_y)) < 15.2)
  {
    ADEBUG << "short summon. trajectory is too short. skip.";
    return;
  }

  //const auto& open_space_info = frame_->open_space_info();
  //确定左右泊出
  //double end_pose_theta = open_space_info.open_space_end_pose().at(2);
  //bool is_right_summon = std::abs(end_pose_theta) < (M_PI / 2.0);
  //bool is_horizontal = (std::abs(open_space_info.origin_heading()) > (M_PI / 2.5)) && (std::abs(open_space_info.origin_heading()) < (M_PI / 1.8));
  //std::string summon_direction = is_right_summon ? "right summon." : "left summon.";
  //std::string slot_type = is_horizontal ? "horizontal type." : "vertical type or oblique type.";
  //ADEBUG << "short summon. direction : " << summon_direction << " slot type : " << slot_type << " end pose theta : " << end_pose_theta;
  //jmc_auto::common::math::Box2d judge_box;
  
  jmc_auto::common::math::Vec2d summon_point_enu;
  auto end_x_enu = frame_->local_view().pad_msg->point().end().x();
  auto end_y_enu = frame_->local_view().pad_msg->point().end().y();
  summon_point_enu.set_x(end_x_enu);
  summon_point_enu.set_y(end_y_enu);

  auto min_comp = [&summon_point_enu](const TrajectoryPoint& tp1, const TrajectoryPoint& tp2){
    return ((summon_point_enu.x() - tp1.path_point().x()) * (summon_point_enu.x() - tp1.path_point().x()) + 
            (summon_point_enu.y() - tp1.path_point().y()) * (summon_point_enu.y() - tp1.path_point().y())) < 
            ((summon_point_enu.x() - tp2.path_point().x()) * (summon_point_enu.x() - tp2.path_point().x()) +
            (summon_point_enu.y() - tp2.path_point().y()) * (summon_point_enu.y() - tp2.path_point().y())); 
  };
  auto min_iter = std::min_element(trajectory_data->begin(), trajectory_data->end(), min_comp);
  
  jmc_auto::common::math::Vec2d target_point_enu;
  target_point_enu.set_x(min_iter->path_point().x());
  target_point_enu.set_y(min_iter->path_point().y());
  ADEBUG << std::setprecision(9) << "short summon. target point x = " << target_point_enu.x() << " y = " << target_point_enu.y();

  auto find_func = [&target_point_enu](const TrajectoryPoint& p){
    double x = p.path_point().x();
    double y = p.path_point().y();
    double theta = p.path_point().theta();
    jmc_auto::common::math::Vec2d center(x + 1.44 * std::cos(theta), y + 1.44 * std::sin(theta));
    jmc_auto::common::math::Box2d vehicle_box(center, theta, 4.89, 1.93);
    return vehicle_box.IsPointIn(target_point_enu);
  };
  auto bound_iter = std::find_if(trajectory_data->begin(), trajectory_data->end(), find_func);
  ADEBUG << "short summon. trajectory size : " << trajectory_data->size() << " erase from : " << bound_iter - trajectory_data->begin() << " to end.";

  is_trajectory_truncated_ = true;
  if((bound_iter - trajectory_data->begin()) < 2)
  {
    trajectory_data->erase(trajectory_data->begin() + 2, trajectory_data->end());
  }else
  {
    trajectory_data->erase(bound_iter, trajectory_data->end()); 
  }

/*
  if(is_right_summon)
  {
    if(is_horizontal)
    {
      judge_box = jmc_auto::common::math::Box2d::CreateAABox({0.0, -5.5}, {-4.0, 5.0});
    }else
    {
      judge_box = jmc_auto::common::math::Box2d::CreateAABox({-0.2, 0.0}, {6.0, 5.0});
    }
  }else
  {
    judge_box = jmc_auto::common::math::Box2d::CreateAABox({-6.0, 5.0}, {2.8, 0.0});
  }
  jmc_auto::common::math::Vec2d summon_point_ref;

  auto end_x = frame_->local_view().pad_msg->point().end().x();
  auto origin_x = open_space_info.origin_point().x();
  auto end_y = frame_->local_view().pad_msg->point().end().y();
  auto origin_y = open_space_info.origin_point().y();
  summon_point_ref.set_x(end_x - origin_x);
  summon_point_ref.set_y(end_y - origin_y);
  summon_point_ref.SelfRotate(-open_space_info.origin_heading());

  bool is_point_in = judge_box.IsPointIn(summon_point_ref);
  ADEBUG << "short summon. point in : " << is_point_in << " x=" << summon_point_ref.x() << " y=" << summon_point_ref.y() << " origin heading=" << open_space_info.origin_heading();
  
  if(is_point_in)
  {
    
    //auto comp = [&origin_x](const TrajectoryPoint& tp, const double x){
    //  return tp.path_point().x() < (x + origin_x);
    //};
    

    common::math::Vec2d origin_point(origin_x, origin_y);
    unsigned int count = 0;
    for(const auto& p : *trajectory_data)
    {
      common::math::Vec2d raw_p(p.path_point().x(), p.path_point().y());
      raw_p -= origin_point;
      raw_p.SelfRotate(-open_space_info.origin_heading());
      if(is_right_summon)
      {      
        if(is_horizontal)
        {
          if(raw_p.y() + 3.88 > summon_point_ref.y())
          {
            break;
          }
        }else
        {
          if(raw_p.x() > summon_point_ref.x())
          {
            break;
          }
        }
      }else
      {
        if(raw_p.x() < summon_point_ref.x())
        {
          break;
        }
      }
      ++count;
    }
    //auto it_lower = std::lower_bound(trajectory_data->begin(), trajectory_data->end(), summon_point_ref.x(), comp);
    ADEBUG << "short summon. trajectory size : " << trajectory_data->size() << " erase from : " << count << " to end.";
    if(0 == count)
    {
      trajectory_data->erase(trajectory_data->begin() + 1, trajectory_data->end());
    }else
    {
      trajectory_data->erase(trajectory_data->begin() + count, trajectory_data->end());
    }
    trajectory_data->back().set_v(0.0);
    trajectory_data->back().set_a(0.0);
  }*/
}

}  // namespace planning
}  // namespace jmc_auto
