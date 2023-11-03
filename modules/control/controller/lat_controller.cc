/*****************************************************************************
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

#include "modules/control/controller/lat_controller.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/LU"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/linear_quadratic_regulator.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/util/string_util.h"
#include "modules/control/common/control_gflags.h"
#include "modules/common/configs/config_gflags.h"

namespace jmc_auto {
namespace control {

using jmc_auto::common::ErrorCode;
using jmc_auto::common::Point3D;
using jmc_auto::common::Status;
using jmc_auto::common::TrajectoryPoint;
using jmc_auto::common::VehicleStateProvider;
using jmc_auto::common::util::StrCat;
using Matrix = Eigen::MatrixXd;
using jmc_auto::common::time::Clock;
using jmc_auto::common::PathPoint;

LatController::LatController() : name_("LQR-based Lateral Controller") {
  if (FLAGS_enable_csv_debug) {
  ADEBUG << "printf lat csv log";
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  strftime(name_buffer, 80, "data/csv/steer_log_%F_%H%M%S.csv",
           localtime(&raw_time));
  steer_log_file_ = fopen(name_buffer,"w");
  if (steer_log_file_ == nullptr)
  {
    AERROR << "Fail to open file:" << name_buffer ;
    FLAGS_enable_csv_debug = false ;
  }else
  {
    ADEBUG << "steer_log_file not empty" ;
   // steer_log_file_ << std::fixed;
   // steer_log_file_ << std::setprecision(6);
    fprintf(steer_log_file_,
	    "timestramp,"
            "lateral_error,"
             "heading_error,"
             "x_error,"
             "y_error,"
             "theta_error,"
             "steering_angle_cmd,"
             "curvature,"
             "gear,"
             "steer_angle_feedforward,"
             "lqr_lateral_contribution,"
             "lqr_lateral_rate_contribution,"
             "lqr_heading_contribution,"
             "lqr_heading_rate_contribution,"
             "steer_angle_feedback,"
             "steering_position,"
             "v,"
             "match_x,"
             "match_y,"
             "match_theta,"
             "vehicle_x,"
             "vehicle_y,"
             "heading,"
             "\r\n");
    fflush(steer_log_file_);
  }
  }
  AINFO << "Using " << name_;
}

void LatController::ProcessLogs(const SimpleLateralDebug *debug,
                                const canbus::Chassis *chassis) {
  double current_control_time = Clock::NowInSeconds();
  fprintf(steer_log_file_,
         "%.6f, %.6f , %.6f , %.6f , %.6f , %.6f , %.6f , %.6f , %d ,"
         " %.6f , %.6f , %.6f , %.6f , %.6f , %.6f , %.6f , %.6f ,%.6f,%.6f,"
         " %.6f , %.6f , %.6f , %.6f , \r\n",
	  current_control_time ,
          debug->lateral_error() , debug->heading_error(),
          debug->x_error() , debug->y_error(),
          debug->theta_error(),debug->steering_angle_cmd(),
          debug->match_curvature(),chassis->gear_location(),
          debug->steering_angle_feedforward(),
          debug->steering_angle_lateral_contribution(),
          debug->steering_angle_lateral_rate_contribution(),
          debug->steering_angle_heading_contribution(),
          debug->steering_angle_heading_rate_contribution(),
          debug->steering_angle_feedback(),
          chassis->steering_percentage(),
          VehicleStateProvider::instance()->linear_velocity(),
          debug->match_x(),debug->match_y(),debug->match_theta(),
          debug->vehicle_x(),debug->vehicle_y(),debug->heading());
  return ;
  //ADEBUG << "Steer_Control_Detail: " << log_str;
}

void LatController::CloseLogFile() {
  if (FLAGS_enable_csv_debug ) {
    if (steer_log_file_ != nullptr)
    {
      fclose(steer_log_file_);
      steer_log_file_ = nullptr;
    }
  }
}

LatController::~LatController() { CloseLogFile(); }

void LatController::Stop() { CloseLogFile(); }

std::string LatController::Name() const { return name_; }

bool LatController::LoadControlConf(const ControlConf *control_conf) {
  const auto &vehicle_param_ =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  ts_ = control_conf->lat_controller_conf().ts();
  CHECK_GT(ts_, 0.0) << "[LatController] Invalid control update interval.";
  cf_ = control_conf->lat_controller_conf().cf();
  cr_ = control_conf->lat_controller_conf().cr();
  preview_window_ = control_conf->lat_controller_conf().preview_window();
//  lookahead_station_ = control_conf->lat_controller_conf().lookahead_station();
//  lookback_station_ = control_conf->lat_controller_conf().lookback_station();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_transmission_ratio_ = vehicle_param_.steer_ratio();
  steering_max_degree_ =
  vehicle_param_.max_steer_angle() / M_PI * 180;
  max_lat_acc_ = control_conf->lat_controller_conf().max_lateral_acceleration();

  const double mass_fl = control_conf->lat_controller_conf().mass_fl();
  const double mass_fr = control_conf->lat_controller_conf().mass_fr();
  const double mass_rl = control_conf->lat_controller_conf().mass_rl();
  const double mass_rr = control_conf->lat_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;//
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;
   //车辆前端距车辆质心距离
  lf_ = wheelbase_ * (1.0 - mass_front / mass_);// mass_front越大，Lf越小
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);//后端距质心距离
  // moment of inertia
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
  lqr_eps_ = control_conf->lat_controller_conf().eps();
  lqr_max_iteration_ = control_conf->lat_controller_conf().max_iteration();
  minimum_speed_protection_ = control_conf->minimum_speed_protection();
  AINFO << "Load lat control conf file succeed!" ;
  return true;
}



void LatController::LogInitParameters() {
  AINFO << name_ << " begin.";
  AINFO << "[LatController parameters]"
        << " mass_: " << mass_ << ","
        << " iz_: " << iz_ << ","
        << " lf_: " << lf_ << ","
        << " lr_: " << lr_;
}

void LatController::InitializeFilters(const ControlConf *control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(
      ts_, control_conf->lat_controller_conf().cutoff_freq(), &den, &num);
  digital_filter_.set_coefficients(den, num);
  curvature_mean_filter_ = common::MeanFilter(
      control_conf->lat_controller_conf().mean_filter_window_size());
  steering_mean_filter_ = common::MeanFilter(
      control_conf->lat_controller_conf().mean_filter_window_size());
  AINFO << "LAT Filter init succeed!" ;
}

Status LatController::Init(const ControlConf *control_conf) {
  control_conf_ = control_conf;
  if (!LoadControlConf(control_conf)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }
  // Matrix init operations.
  const int matrix_size = basic_state_size_;
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_); //矩阵A初始化为全零矩阵
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
  //矩阵A中的常数项
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  //矩阵A中和速度V相关的变量
  matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  matrix_b_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bdc_ = Matrix::Zero(matrix_size, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  matrix_state_ = Matrix::Zero(matrix_size, 1);
  matrix_k_ = Matrix::Zero(1, matrix_size);
  matrix_r_ = Matrix::Identity(1, 1);
  matrix_r_(0,0) = 0.1 ;
  ADEBUG << "matrix_r " << matrix_r_ ;
  matrix_q_ = Matrix::Zero(matrix_size, matrix_size);
  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();

  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
  }
  matrix_q_updated_ = matrix_q_;
  InitializeFilters(control_conf_);
  //PID初始化
  lateral_pid_controller_.Init(control_conf_->lat_controller_conf().lateral_pid_conf());
//  auto &lat_controller_conf = control_conf->lat_controller_conf();
//  LoadLatGainScheduler(lat_controller_conf);
//  LoadSteerCalibrationTable(lat_controller_conf);
//  steering_pid_controller_.Init(lat_controller_conf.steering_pid_conf()) ;
//  AINFO << "LAT PID coeff " << lat_controller_conf.steering_pid_conf().kp() ;
  LogInitParameters();
  AINFO << "Lat init succeed!";
//  use_dymethed = false ;
  return Status::OK();
}


void LatController::LoadLatGainScheduler(
    const LatControllerConf &lat_controller_conf) {
  const auto &lat_err_gain_scheduler =
      lat_controller_conf.lat_err_gain_scheduler();
  const auto &heading_err_gain_scheduler =
      lat_controller_conf.heading_err_gain_scheduler();
  AINFO << "Lateral control gain scheduler loaded";
  Interpolation1D::DataType xy1, xy2;
  for (const auto &scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto &scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  lat_err_interpolation_.reset(new Interpolation1D);
  CHECK(lat_err_interpolation_->Init(xy1))
      << "Fail to load lateral error gain scheduler";

  heading_err_interpolation_.reset(new Interpolation1D);
  CHECK(heading_err_interpolation_->Init(xy2))
      << "Fail to load heading error gain scheduler";
  AINFO << "Load Lateral control gain scheduler succeed!";
}


Status LatController::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {

    auto vehicle_state = VehicleStateProvider::instance() ;
    chassis_ = chassis;
    trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(planning_published_trajectory));
    trajectory_message_ = planning_published_trajectory ;
    SimpleLateralDebug *debug = cmd->mutable_debug()->mutable_simple_lat_debug();
    debug->Clear();
    int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
    debug->set_is_parking_mode(planning_published_trajectory->decision().main_decision().has_parking());
    debug->set_is_summon_mode(planning_published_trajectory->decision().main_decision().has_summon());
    double steering_angle_feedback = 0.0 ;
    double steering_angle_feedforward = 0.0 ;
    matrix_r_(0,0) = 0.1 ;
    if (debug->is_parking_mode())
    {
      preview_window_ = control_conf_->lat_controller_conf().parking_preview_window() ;
    }else if(debug->is_summon_mode())
    {
      preview_window_ = control_conf_->lat_controller_conf().summon_preview_window() ;
    }else
    {
      preview_window_ = control_conf_->lat_controller_conf().driving_preview_window() ;
    }

    if (debug->is_parking_mode())
    {
        ADEBUG << "Parking mode" ;
        double feedforward_coeff = 1.0;   //------------曲率加权系数
          //读取参数
        if(vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
        {
          cf_ = -control_conf_->lat_controller_conf().cf();
          cr_ = -control_conf_->lat_controller_conf().cr();
	 // feedforward_coeff = 0.872118;
          matrix_a_(0,1) = 0.0 ;
          matrix_a_coeff_(0,2) = 1.0 ;
        }else
        {
	  matrix_r_(0,0) = 0.1 ;
          matrix_a_(0,1) = 1.0 ;
          matrix_a_coeff_(0,2) = 0.0 ;
          cf_ = control_conf_->lat_controller_conf().cf();
          cr_ = control_conf_->lat_controller_conf().cr();
          feedforward_coeff = 1.0;
        }

//*************根据车位类型配置参数*************************//
      if(planning_published_trajectory->decision().main_decision().parking().parking_type()== planning::Parking_Type::HORIZONTAL)  //水平车位参数
      {
	/********水平车位参数配置*********************/
        ADEBUG << "水平车位泊车" ;
//TODO:按照剩余距离调试参数，待测试，先删除     
       //   UpdateState(debug);
       //   GetPathRemain(debug);
       //   if(debug->path_remain()<0.3)
       //  {
       //           feedforward_coeff  = 0 ;
       //   }else
       //   {
       //  feedforward_coeff  = 0.9 ;
       //   }

	if(vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
	{
          preview_window_ = 110 ;  //-------------------水平车位预瞄
          for (int i = 0; i < q_param_size ; i++)
          {
		matrix_q_(0,0) = 0.1 ;
		matrix_q_(1,1) = 0 ;
		matrix_q_(2,2) = 1.2 ; 
		matrix_q_(3,3) = 0 ;
          }
	}else
	{
	  preview_window_ = 110 ;
	  matrix_q_(0,0) = 0 ;
	  matrix_q_(1,1) = 0.0 ;
	  matrix_q_(2,2) = 1.0 ;
	  matrix_q_(3,3) = 0.0 ;
	}
	
        /*******end***********************************/
      }else
      {
        AINFO << "垂直车位以及斜列车位" ;
        if(planning_published_trajectory->decision().main_decision().parking().status()==2)
	{
	  //-----一段式泊车参数选择----------//
	  preview_window_ = 133 ;
	  feedforward_coeff = 0.75 ;
	  matrix_q_(0,0) =0.08 ;
	  matrix_q_(1,1) = 0.0 ;
	  matrix_q_(2,2) = 2.0 ;
	  matrix_q_(3,3) = 0.0 ;
	  AINFO << "VERTICAL && ONE PARKING TRAJECTORY";
	}else
	{
          UpdateState(debug);
          GetPathRemain(debug);
          ADEBUG << "path remain : " << debug->path_remain();
          
	  //-----多段式泊车R档-------------//
	  if(vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
       	  {
	    feedforward_coeff = 1.0;
	    preview_window_ = 350;
            if(debug->path_remain() > 1.5)
	    {
		matrix_q_(0,0) = 1.0 ;
                matrix_q_(1,1) = 0 ;
                matrix_q_(2,2) = 0.5 ;
                matrix_q_(3,3) = 0 ; 
                ADEBUG << "parking coeff 1";
            }else
            {
                feedforward_coeff = 0 ;
		matrix_q_(0,0) = 0 ;
                matrix_q_(1,1) = 0 ;
                matrix_q_(2,2) = 2.8 ;
                matrix_q_(3,3) = 0 ;
                ADEBUG << "parking coeff 2" ;	  
             }
	//    for (int i = 0; i < q_param_size ; i++)
        //    {
        //     	matrix_q_(i,i) = control_conf_->lat_controller_conf().reverse_matrix_q(i) ;
        //    }
      	  }else
      	  {
            //垂直或斜列车位多段式泊车-D档下Q矩阵参数
             matrix_q_(0,0) = 0.0 ;
             matrix_q_(1,1) = 0.0 ;
             matrix_q_(2,2) = 3.5 ;
             matrix_q_(3,3) = 0.0 ;
             preview_window_ = 120 ;
             feedforward_coeff = 1.3 ;
      	  }
	}
     }
     //更新A
        matrix_a_(1, 2) = (cf_ + cr_) / mass_;
        matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
        matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
        matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
        matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
        matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
        //更新B
        matrix_b_(1, 0) = cf_ / mass_;
        matrix_b_(3, 0) = lf_ * cf_ / iz_;
        matrix_bd_ = matrix_b_ * ts_;
        //更新航向，倒车时要+pi
        UpdateDrivingOrientation();
        //更新X
        UpdateState(debug);
        //更新A
        UpdateMatrix(debug);
        //A离散
        UpdateMatrixCompound();
        //LQR求解
        double  lqr_start_time = Clock::NowInSeconds();
    /*    if(vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
	{
            if(planning_published_trajectory->decision().main_decision().parking().status() ==2)
            {
            	matrix_k_(0,0)= -0.583981;
	        matrix_k_(0,1) = 0.059624;
                matrix_k_(0,2) = 2.37107 ;//2.35107
                matrix_k_(0,3) = -0.419932;
		feedforward_coeff = debug->curvature()>0?0.95:1.0 ;
            }else{
		 matrix_k_(0,0)= -0.682362;
                matrix_k_(0,1) = 0.0596598;
                matrix_k_(0,2) = 2.58896 ;
                matrix_k_(0,3) = -0.419068;

	    }
          // common::math::SolveLQRProblem(matrix_adc_,matrix_bdc_,matrix_q_,matrix_r_,
          //                            lqr_eps_,lqr_max_iteration_,&matrix_k_);
	}else
	{
	    common::math::SolveLQRProblem(matrix_adc_,matrix_bdc_,matrix_q_,matrix_r_,
                                      lqr_eps_,lqr_max_iteration_,&matrix_k_);
	}         */

        common::math::SolveLQRProblem(matrix_adc_,matrix_bdc_,matrix_q_,matrix_r_,
                                      lqr_eps_,lqr_max_iteration_,&matrix_k_);
        double lqr_end_time = Clock::NowInSeconds();
        ADEBUG << "lqr compute time " << (lqr_end_time - lqr_start_time) *1000 ;
        //计算方向盘转角
        //前轮弧度 -> 方向盘弧度 -> 方向盘角度
        steering_angle_feedback = -(matrix_k_ * matrix_state_)(0,0)
                                * steer_transmission_ratio_ * rad_to_deg ;
        ADEBUG << "steering_angle_feedback " << steering_angle_feedback ;
        steering_angle_feedforward = ComputeFeedForward(debug);

	    ADEBUG << "feedforward_coeff " << feedforward_coeff ;
        steering_angle = steering_angle_feedback + feedforward_coeff * steering_angle_feedforward ;

        ComputeSteeringAngle(steering_angle) ;
        //cmd->set_steering_angle(steering_angle) ;
    }else if(debug->is_summon_mode())
    {
      ADEBUG << "SUMMON MODE" ;
      //将轨迹点转化到质心
  	preview_window_ = 100 ;
//        trajectory_analyzer_.TrajectoryTransformToCOM(lr_);
        //Q矩阵
          //读取参数
      if(vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE)
       {
          cf_ = -control_conf_->lat_controller_conf().cf();
          cr_ = -control_conf_->lat_controller_conf().cr();
          for (int i = 0; i < q_param_size ; i++)
          {
             matrix_q_(i,i) = control_conf_->lat_controller_conf().reverse_matrix_q(i) ;
          }
          matrix_a_(0,1) = 0.0 ;
          matrix_a_coeff_(0,2) = 1.0 ;
          matrix_r_(0,0) = 0.1 ;
        }else
	{
          matrix_a_(0,1) = 1.0 ;
          matrix_a_coeff_(0,2) = 0.0 ;
          cf_ = control_conf_->lat_controller_conf().cf();
          cr_ = control_conf_->lat_controller_conf().cr();
          matrix_q_(0,0) = 0.4 ;
          matrix_q_(1,1) = 0.0 ;
	  matrix_q_(2,2) = 1.0 ;
	  matrix_q_(3,3) = 0.0 ;
          ADEBUG << "Q(0,0) " << matrix_q_(0, 0) ;
        }
        //更新A
        matrix_a_(1, 2) = (cf_ + cr_) / mass_;
        matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
        matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
        matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
        matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
        matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
        //更新B
        matrix_b_(1, 0) = cf_ / mass_;
        matrix_b_(3, 0) = lf_ * cf_ / iz_;
        matrix_bd_ = matrix_b_ * ts_;
        //更新航向，倒车时要+pi
        UpdateDrivingOrientation();
        //更新X
        UpdateState(debug);
        //更新A
        UpdateMatrix(debug);
        //A离散
        UpdateMatrixCompound();
        //LQR求解
        double  lqr_start_time = Clock::NowInSeconds();
        common::math::SolveLQRProblem(matrix_adc_,matrix_bdc_,matrix_q_,matrix_r_,
                                      lqr_eps_,lqr_max_iteration_,&matrix_k_);
        double lqr_end_time = Clock::NowInSeconds();
        ADEBUG << "lqr compute time " << (lqr_end_time - lqr_start_time) *1000 ;
        //计算方向盘转角
        //前轮弧度 -> 方向盘弧度 -> 方向盘角度
      //  ADEBUG << "matrix_k_ " << matrix_k_(0,0) << "," << matrix_k_(0,1) << "," << matrix_k_(0,2) << "," << matrix_k_(0,3);
      //  ADEBUG << "matrix_state_ " << matrix_state_(0,0) << " " << matrix_state_(1,0) << " " << matrix_state_(2,0) << " " << matrix_state_(3,0);
        steering_angle_feedback = -(matrix_k_ * matrix_state_)(0,0)
                                * steer_transmission_ratio_ * rad_to_deg ;
        ADEBUG << "steering_angle_feedback " << steering_angle_feedback ;
        steering_angle_feedforward = ComputeFeedForward(debug);
        steering_angle = steering_angle_feedback + steering_angle_feedforward;
        ComputeSteeringAngle(steering_angle) ;
        //cmd->set_steering_angle(steering_angle) ;
    }else
    {
        ADEBUG << "Driving Mode" ;
        //重置泊车第一端轨迹状态
        //gear_change_num = 0 ;
        debug->set_is_parking_mode(false) ;
        debug->set_is_summon_mode(false);
        //轨迹不转化到质心
        cf_ = control_conf_->lat_controller_conf().cf();
        cr_ = control_conf_->lat_controller_conf().cr();
        //Q矩阵
        for (int i = 0; i < q_param_size ; i++)
        {
          matrix_q_(i,i) = control_conf_->lat_controller_conf().matrix_q(i) ;
          ADEBUG << "Q(" << i << ") " << matrix_q_(i, i) ;
        }
        matrix_a_(1, 2) = (cf_ + cr_) / mass_;
        matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
        matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
        matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
        matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
        matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
        matrix_b_(1, 0) = cf_ / mass_;
        matrix_b_(3, 0) = lf_ * cf_ / iz_;
        matrix_bd_ = matrix_b_ * ts_;
        //更新航向，倒车时要+pi
        UpdateDrivingOrientation();
        //更新X
        UpdateState(debug);
        //更新A
        UpdateMatrix(debug);
        //A离散
        UpdateMatrixCompound();
        //LQR求解
        common::math::SolveLQRProblem(matrix_adc_,matrix_bdc_,matrix_q_,matrix_r_,
                                      lqr_eps_,lqr_max_iteration_,&matrix_k_);
        //计算方向盘转角
        //前轮弧度 -> 方向盘弧度 -> 方向盘角度
        steering_angle_feedback = -(matrix_k_ * matrix_state_)(0,0)
                                * steer_transmission_ratio_ * rad_to_deg ;
        ADEBUG << "steering_angle_feedback " << steering_angle_feedback ;
        steering_angle_feedforward = ComputeFeedForward(debug);
        steering_angle = steering_angle_feedback + steering_angle_feedforward ;
        ComputeSteeringAngle(steering_angle) ;
        //cmd->set_steering_angle(steering_angle) ;

    }
     for (int i = 0; i < q_param_size ; i++)
        {
             ADEBUG << "Q(" << i << ") " << matrix_q_(i, i) ;
        }
    AINFO << "matrix_k_ " << matrix_k_(0,0) << "," << matrix_k_(0,1) << "," << matrix_k_(0,2) << "," << matrix_k_(0,3);
    ADEBUG << "matrix_state_ " << matrix_state_(0,0) << " " << matrix_state_(1,0) << " " << matrix_state_(2,0) << " " << matrix_state_(3,0);

    double steering_angle_cmd_diff = steering_angle - last_steering_angle_cmd ;
	if(debug->is_parking_mode())
	{
		steering_angle_cmd_diff = common::math::Clamp(steering_angle_cmd_diff,-4.0,4.0);
	}else
	{
		steering_angle_cmd_diff = common::math::Clamp(steering_angle_cmd_diff,-3.0,3.0);
	}
    steering_angle = last_steering_angle_cmd + steering_angle_cmd_diff ;
    cmd->set_steering_angle(steering_angle);
    ADEBUG << "gear " << vehicle_state->gear() ;
    ADEBUG << "CAR speed" << vehicle_state->linear_velocity() ;
    ADEBUG << "steering angle command " << steering_angle ;
    cmd->set_steering_rate(FLAGS_steer_angle_rate);
    const double steer_angle_lateral_contribution =
      -matrix_k_(0, 0) * matrix_state_(0, 0) * 180 / M_PI * steer_transmission_ratio_;

    const double steer_angle_lateral_rate_contribution =
      -matrix_k_(0, 1) * matrix_state_(1, 0) * 180 / M_PI * steer_transmission_ratio_;

    const double steer_angle_heading_contribution =
      -matrix_k_(0, 2) * matrix_state_(2, 0) * 180 / M_PI * steer_transmission_ratio_;

    const double steer_angle_heading_rate_contribution =
      -matrix_k_(0, 3) * matrix_state_(3, 0) * 180 / M_PI * steer_transmission_ratio_;
    ADEBUG << "Q(0-3) contribution" << steer_angle_lateral_contribution << ","
            << steer_angle_lateral_rate_contribution << ","
            << steer_angle_heading_contribution << ","
            << steer_angle_heading_rate_contribution ;

    debug->set_steering_angle_lateral_contribution(steer_angle_lateral_contribution);
    debug->set_steering_angle_lateral_rate_contribution(steer_angle_lateral_rate_contribution);
    debug->set_steering_angle_heading_contribution(steer_angle_heading_contribution);
    debug->set_steering_angle_heading_rate_contribution(steer_angle_heading_rate_contribution);
    debug->set_steering_angle_feedforward(steering_angle_feedforward);
    debug->set_steering_angle_feedback(steering_angle_feedback);
    debug->set_steering_position(chassis->steering_percentage());
    debug->set_ref_speed(vehicle_state->linear_velocity());
    debug->set_steering_target_position_error(steering_angle - chassis->steering_percentage()) ;//增加车轮目标位置和实际位置误差输出信息
    ADEBUG << "steering_position:" << debug->steering_position();
    ADEBUG << "steering_angle :" << steering_angle ;
    debug->set_steering_angle_cmd(steering_angle);
    last_steering_angle_cmd = steering_angle ;
  //debug->set_gear_location(canbus::Chassis::vehicle_state->gear());
  //输出csv日志
    if (FLAGS_enable_csv_debug && steer_log_file_ != nullptr)
    {
      ProcessLogs(debug, chassis);
    }
    return Status::OK();

}



Status LatController::Reset() {
  //steering_pid_controller_.Reset();
  //lateral_pid_controller_.Reset();
  return Status::OK();
}
//更新状态矩阵
void LatController::UpdateState(SimpleLateralDebug *debug) {
  auto vehicle_state = VehicleStateProvider::instance();
//  if ( debug->is_summon_mode())
//  {
//     const auto &com = vehicle_state->ComputeCOMPosition(lr_);
//     vehicle_x = com.x() ;
//     vehicle_y = com.y() ;
//     ComputeLateralErrors(vehicle_x,vehicle_y,
//                         driving_orientation,
//                         vehicle_state->linear_velocity(),
//                         vehicle_state->angular_velocity(),
//                         vehicle_state->linear_acceleration(),
//                         trajectory_analyzer_, debug);
//  }else
//  {
    ComputeLateralErrors(vehicle_state->x(),vehicle_state->y(),
                         driving_orientation,
                         vehicle_state->linear_velocity(),
                         vehicle_state->angular_velocity(),
                         vehicle_state->linear_acceleration(),
                         trajectory_analyzer_, debug);
//  }
  // Reverse heading error if vehicle is going in reverse
    if (vehicle_state->gear() ==
        canbus::Chassis::GEAR_REVERSE) {
     debug->set_heading_error(-debug->heading_error());
      AINFO << "vehicle is going in reverse!" ;
    }
// State matrix update;
  // First four elements are fixed;
    matrix_state_(0, 0) = debug->lateral_error();
    matrix_state_(1, 0) = debug->lateral_error_rate();
    matrix_state_(2, 0) = debug->heading_error();
    matrix_state_(3, 0) = debug->heading_error_rate();
    ADEBUG << "lateral_error " <<debug->lateral_error();
    ADEBUG << "lateral_error_rate " <<debug->lateral_error_rate();
    ADEBUG << "heading_error " <<debug->heading_error();
    ADEBUG << "heading_error_rate " <<debug->heading_error_rate();
    AINFO << "Update state X succeed!";
}

void LatController::UpdateMatrix(SimpleLateralDebug *debug){
  // 倒车模式下，用运动模型替代动力学模型
   double v = 0.0 ;
   if ( VehicleStateProvider::instance()->gear() == canbus::Chassis::GEAR_REVERSE) {
    v = -0.5 ;
    matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
  } else
 {
	if(debug->is_parking_mode() || debug->is_summon_mode())
	{
		v = 0.5 ;
	}else
	{
	        v = std::max(VehicleStateProvider::instance()->linear_velocity(), minimum_speed_protection_);
               // v = 1.0 ;
	}
    matrix_a_(0, 2) = 0.0;
 //   v= minimum_speed_protection_ ;
  }
 ADEBUG << "UpdateMatrix v= " << v ;
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);
  AINFO << "Update Matrix A succeed!" ;
}

void LatController::UpdateMatrixCompound() {
  // Initialize preview matrix
  matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
  matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;

  AINFO << "UpdateMatrixCompound succeed!" ;
}

double LatController::ComputeFeedForward(SimpleLateralDebug *debug) {
  ADEBUG << "kappa_printf " << debug->curvature() ;
//  const double kv =
//      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
  double ref_curvature = debug->curvature() ;
 // ref_curvature = curvature_mean_filter_.Update(ref_curvature);
// ref_curvature = common::math::Clamp(ref_curvature,-0.2,0.2);
  double kappa_diff = ref_curvature - last_kappa ;
  kappa_diff = common::math::Clamp(kappa_diff,-0.01,0.01);
  ref_curvature = last_kappa + kappa_diff ;
  last_kappa = ref_curvature ;
  ADEBUG <<"kappa limited" << ref_curvature ;
  // then change it from rad to %
  const double v = VehicleStateProvider::instance()->linear_velocity();
  double steer_angle_feedforwardterm = 0.0;
  if (debug->is_parking_mode()||debug->is_summon_mode()) {
    steer_angle_feedforwardterm =wheelbase_ * ref_curvature * rad_to_deg * steer_transmission_ratio_ ;
  } else {
 //   double kappa_diff = ref_curvature - last_kappa ;
 //   kappa_diff = common::math::Clamp(kappa_diff,-0.005,0.005);
 //   ref_curvature = last_kappa + kappa_diff ;
 //   last_kappa = ref_curvature ;
//    ADEBUG <<"kappa limited" << ref_curvature ;
    steer_angle_feedforwardterm =wheelbase_ * ref_curvature * rad_to_deg * steer_transmission_ratio_ ;
    //(wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
    //     matrix_k_(0, 2) *
    //         (lr_ * ref_curvature -
    //          lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
    //    rad_to_deg * steer_transmission_ratio_ ;

  }
  ADEBUG << "steering_angle_feedforward " << steer_angle_feedforwardterm ;
  return steer_angle_feedforwardterm;
}

void LatController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v,const double linear_a, const TrajectoryAnalyzer &trajectory_analyzer,
    SimpleLateralDebug *debug) {
   double heading_error = 0.0 ;
   double lateral_error = 0.0 ;
   double lateral_error_rate = 0.0 ;
    //泊车时选取预瞄点为目标点
    auto match_nearest_point = trajectory_analyzer.QueryNearestPointByPosition(x,y);
    double match_theta = match_nearest_point.path_point().theta() ;
    auto target_point = trajectory_analyzer.QueryPreviewPointByPosition(x,y,preview_window_);
    double target_point_theta = target_point.path_point().theta() ;
    if (VehicleStateProvider::instance()->gear() == canbus::Chassis::GEAR_REVERSE)
    {
      target_point_theta = common::math::NormalizeAngle(target_point.path_point().theta() + M_PI );
      match_theta =common::math::NormalizeAngle(match_nearest_point.path_point().theta() + M_PI );
    }
    ADEBUG << "x_error y_error heading_error " << x - match_nearest_point.path_point().x() << " "
                                               << y - match_nearest_point.path_point().y() << " "
                                               << theta - match_theta ;
    /***********debug信息输出***************/
    //**********************************//
    debug->set_theta_error(theta - match_theta);
    debug->set_x_error(x - match_nearest_point.path_point().x());
    debug->set_y_error(y - match_nearest_point.path_point().y());
    debug->set_match_x(match_nearest_point.path_point().x());
    debug->set_match_y(match_nearest_point.path_point().y());
    debug->set_match_theta(match_theta);
    debug->set_vehicle_x(x);
    debug->set_vehicle_y(y);
    debug->set_heading(theta);
    debug->set_match_curvature(match_nearest_point.path_point().kappa());
    debug->set_current_station(match_nearest_point.path_point().s());
    /*********************************/

    ADEBUG << "target point " << target_point.ShortDebugString() ;
    ADEBUG << "x,y " << x << " , " << y ;
    double dx = x - target_point.path_point().x();
    double dy = y - target_point.path_point().y();
   // dx = debug->x_error();
   // dy = debug->y_error() ;
    ADEBUG << "dx dy " << dx << " , " << dy ;
    //以目标点航向建立SL坐标系
    double cos_theta = std::cos(target_point_theta);
    double sin_theta = std::sin(target_point_theta);
    ADEBUG << "cos_theta " << cos_theta ;
    ADEBUG << "sin_theta " << sin_theta ;
    //横向误差，SL坐标系
    lateral_error = dy * cos_theta - dx * sin_theta;
    debug->set_lateral_error(lateral_error) ;
    //航向误差
    heading_error = common::math::NormalizeAngle(theta - target_point_theta);
    debug->set_ref_heading(target_point_theta);
    debug->set_heading_error(heading_error);
    ADEBUG << "theta " << theta << " target theta " << target_point_theta ;
    //曲率
    debug->set_curvature(target_point.path_point().kappa());
//    if(debug->is_parking_mode()&& VehicleStateProvider::instance()->gear() == canbus::Chassis::GEAR_REVERSE&&
//	debug->curvature()>0)
//    {
//	debug->set_curvature(0.5*debug->curvature());
//    }
    //横向误差变化率
    lateral_error_rate = linear_v * std::sin(heading_error);
    debug->set_lateral_error_rate(lateral_error_rate);
    //航向误差率
    debug->set_ref_heading_rate(target_point.path_point().kappa() * linear_v);
    if (VehicleStateProvider::instance()->gear() == canbus::Chassis::GEAR_REVERSE)
    {
      debug->set_heading_rate(-angular_v) ;
    }else
    {
      debug->set_heading_rate(angular_v);
    }
    debug->set_heading_error_rate(debug->heading_rate() - debug->ref_heading_rate());

    return ;

}

//load steer_torque calibration
void LatController::LoadSteerCalibrationTable(const LatControllerConf &lat_controller_conf){
  const auto &steer_torque_table = lat_controller_conf.steer_calibration_table() ;
  AINFO << "Steer_torque calibration table is loaded,the size of calibration is"
      << steer_torque_table.steer_calibration_size() ;
  Interpolation1D::DataType xy ;
  for (const auto &calibration : steer_torque_table.steer_calibration()){
    xy.push_back(std::make_pair(calibration.angle(),
                                  calibration.torque())) ;
  }
  steer_torque_interpolation_.reset(new Interpolation1D) ;
  CHECK(steer_torque_interpolation_->Init(xy))<<"Fail to init steer_torque_interpolation"  ;
   AINFO << "Load Lateral control steering-torque calibration succeed!";
}

void LatController::UpdateDrivingOrientation() {
  auto vehicle_state = VehicleStateProvider::instance();
  driving_orientation = vehicle_state->heading();
  matrix_bd_ = matrix_b_ * ts_;
  // Reverse the driving direction if the vehicle is in reverse mode
  if (FLAGS_reverse_heading_control) {
    if (vehicle_state->gear() == canbus::Chassis::GEAR_REVERSE) {
      driving_orientation =
          common::math::NormalizeAngle(driving_orientation + M_PI);
      // Update Matrix_b for reverse mode
      matrix_bd_ = -matrix_b_ * ts_;
      ADEBUG << "Matrix_b changed due to gear direction";
    }
  }
}

void LatController::ComputeSteeringAngle(double & steering_angle_)
{
   //限制在方向盘最大转角内
   ADEBUG << "origin steering_angle" << steering_angle_ ;
   //数字滤波
//   steering_angle_ = digital_filter_.Filter(steering_angle);
//   ADEBUG << "after digital filter steering_angle" << steering_angle_ ;
   //和底盘反馈当前方向盘角度对比,前后差值不能超过90度
   double steering_angle_diff = steering_angle_ - chassis_->steering_percentage() ;
   steering_angle_diff = common::math::Clamp(steering_angle_diff,-60.0,60.0) ;
   steering_angle_ = chassis_->steering_percentage() + steering_angle_diff ;
   ADEBUG << "after chassis limited steering angle " << steering_angle_ ;
   //限制角度防止底盘执行超调
  // steering_angle = steering_mean_filter_.Update(steering_angle);//均值滤波
   steering_angle = common::math::Clamp(steering_angle_,-482.0,482.0) ;
   return ;
}

void LatController::ComputeLateralErrorsUsePretime(const double x,const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a ,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleLateralDebug *debug)
{
    auto vehicle_state = VehicleStateProvider::instance();
    double heading_error = 0.0 ;
    double lateral_error = 0.0 ;
    double lateral_error_rate = 0.0 ;
    //计算pretime时间后，车辆的未来位置
    auto pretime = preview_window_*ts_ ;//----------------预瞄时间
    auto preview_station = vehicle_state->EstimateFuturePosition(pretime);//车辆未来位置
    auto preview_station_heading = theta + angular_v * pretime ;//---------车辆未来航向
    //根据预瞄位置选择最近的轨迹点
    //------------------预瞄点
    auto target_point = trajectory_analyzer.QueryMatchedPathPoint(preview_station.x(),preview_station.y());
    //根据当前位置选择的最近的轨迹点
    //------------------最近点
    auto match_nearest_point = trajectory_analyzer.QueryMatchedPathPoint(x,y);
    double match_theta = match_nearest_point.theta() ;

    double target_point_theta = target_point.theta() ;//-------预瞄航向
    if (VehicleStateProvider::instance()->gear() == canbus::Chassis::GEAR_REVERSE)
    {
      target_point_theta = common::math::NormalizeAngle(target_point_theta + M_PI );
      match_theta =common::math::NormalizeAngle(match_theta + M_PI );
    }
    ADEBUG << "x_error y_error heading_error " << x - match_nearest_point.x() << " "
                                               << y - match_nearest_point.y() << " "
                                               << theta - match_theta ;

    debug->set_theta_error(theta - match_theta);

    debug->set_x_error(x - match_nearest_point.x());
    debug->set_y_error(y - match_nearest_point.y());

    ADEBUG << "target point " << target_point.ShortDebugString() ;
    ADEBUG << "x,y " << x << " , " << y ;
    double dx = preview_station.x() - target_point.x();
    double dy = preview_station.y() - target_point.y();
    ADEBUG << "dx dy " << dx << " , " << dy ;
    //以目标点航向建立SL坐标系
    double cos_theta = std::cos(target_point_theta);
    double sin_theta = std::sin(target_point_theta);
    ADEBUG << "cos_theta " << cos_theta ;
    ADEBUG << "sin_theta " << sin_theta ;
    //横向误差，SL坐标系
    lateral_error = dy * cos_theta - dx * sin_theta;
    debug->set_lateral_error(lateral_error) ;
    //航向误差
    heading_error = common::math::NormalizeAngle(preview_station_heading - target_point_theta);
    debug->set_ref_heading(target_point_theta);
    debug->set_heading_error(heading_error);
    ADEBUG << "theta " << preview_station_heading << " target theta " << target_point_theta ;
    //曲率
    debug->set_curvature(target_point.kappa());
    //横向误差变化率
    lateral_error_rate = linear_v * std::sin(heading_error);
    debug->set_lateral_error_rate(lateral_error_rate);
    //航向误差率
    debug->set_ref_heading_rate(target_point.kappa() * linear_v);
    if (VehicleStateProvider::instance()->gear() == canbus::Chassis::GEAR_REVERSE)
    {
      debug->set_heading_rate(-angular_v) ;
    }else
    {
      debug->set_heading_rate(angular_v);
    }
    debug->set_heading_error_rate(debug->heading_rate() - debug->ref_heading_rate());

    return ;
}

 void LatController::GetPathRemain(SimpleLateralDebug *debug) {
    int stop_index = trajectory_message_->trajectory_point_size() - 1; //---------------轨迹最后一个点下标;
    debug->set_path_remain(std::fabs( trajectory_message_->trajectory_point(stop_index).path_point().s() - debug->current_station()));
    ADEBUG << "stop index " << stop_index ;
    ADEBUG << "trajectory point size" << trajectory_message_->trajectory_point_size() ;
    ADEBUG << "last trajectory point station " << trajectory_message_->trajectory_point(stop_index).path_point().s() ;
    ADEBUG << "current station " << debug->current_station() ;
    ADEBUG << "path remain " << debug->path_remain() ;
    return ;
  }


}  // namespace control
}  // namespace jmc_auto
