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

/**
 * @file
 * @brief Defines the LatController class.
 */

#ifndef MODULES_CONTROL_CONTROLLER_LAT_CONTROLLER_H_
#define MODULES_CONTROL_CONTROLLER_LAT_CONTROLLER_H_

#include <fstream>
#include <memory>
#include <string>

#include "Eigen/Core"

#include "modules/common/configs/proto/vehicle_config.pb.h"

#include "modules/common/filters/digital_filter.h"
#include "modules/common/filters/digital_filter_coefficients.h"
#include "modules/common/filters/mean_filter.h"
#include "modules/control/common/interpolation_1d.h"
#include "modules/control/common/interpolation_2d.h"
#include "modules/control/common/trajectory_analyzer.h"
#include "modules/control/controller/controller.h"
#include "modules/control/common/pid_controller.h"
#include <map>
/**
 * @namespace jmc_auto::control
 * @brief jmc_auto::control
 */
namespace jmc_auto {
namespace control {

/**
 * @class LatController
 *
 * @brief LQR-Based lateral controller, to compute steering target.
 * For more details, please refer to "Vehicle dynamics and control."
 * Rajamani, Rajesh. Springer Science & Business Media, 2011.
 */
class LatController : public Controller {
 public:
  /**
   * @brief constructor
   */
  LatController();

  /**
   * @brief destructor
   */
  virtual ~LatController();

  /**
   * @brief initialize Lateral Controller
   * @param control_conf control configurations
   * @return Status initialization status
   */
  common::Status Init(const ControlConf *control_conf) override;

  /**
   * @brief compute steering target based on current vehicle status
   *        and target trajectory
   * @param localization vehicle location
   * @param chassis vehicle status e.g., speed, acceleration
   * @param trajectory trajectory generated by planning
   * @param cmd control command
   * @return Status computation status
   */
  common::Status ComputeControlCommand(
      const localization::LocalizationEstimate *localization,
      const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
      ControlCommand *cmd) override;

  /**
   * @brief reset Lateral Controller
   * @return Status reset status
   */
  common::Status Reset() override;

  /**
   * @brief stop Lateral controller
   */
  void Stop() override;

  /**
   * @brief Lateral controller name
   * @return string controller name in string
   */
  std::string Name() const override;

 protected:
 // logic for reverse driving mode
  void UpdateDrivingOrientation();
  
  void UpdateState(SimpleLateralDebug *debug);

  void UpdateMatrix(SimpleLateralDebug *debug);

  void UpdateMatrixCompound();

  double ComputeFeedForward(SimpleLateralDebug *debug);
  //根据预瞄点个数选择预瞄点
  void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a ,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleLateralDebug *debug);
  /************************************************
  根据预瞄时间选择预瞄点
  1.预估pretime时间后车辆位置，记为预瞄位置
  2.根据预瞄位置匹配最近的轨迹点，记为预瞄点
  3.预瞄位置和预瞄点之间的横向和航向误差为状态矩阵
  *************************************************/
  void ComputeLateralErrorsUsePretime(const double x,const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a ,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleLateralDebug *debug);
  bool LoadControlConf(const ControlConf *control_conf);
  void InitializeFilters(const ControlConf *control_conf);
  void LoadLatGainScheduler(const LatControllerConf &lat_controller_conf);
  void LogInitParameters();
  void ProcessLogs(const SimpleLateralDebug *debug,
                   const canbus::Chassis *chassis);
  void LoadSteerCalibrationTable(const LatControllerConf &lat_controller_conf) ;
  void GetPathRemain(SimpleLateralDebug *debug);
  void CloseLogFile();
  void ComputeSteeringAngle(double & steering_angle_);
  // vehicle parameter
  const ControlConf *control_conf_ = nullptr;
  common::VehicleParam vehicle_param_;
  std::unique_ptr<Interpolation1D> steer_torque_interpolation_;

  // a proxy to analyze the planning trajectory
  TrajectoryAnalyzer trajectory_analyzer_;
  const planning::ADCTrajectory *trajectory_message_ = nullptr ;
  const canbus::Chassis *chassis_ = nullptr;
  // the following parameters are vehicle physics related.
  const double rad_to_deg = 180 /M_PI ;
  // control time interval
  double ts_ = 0.0;
  // corner stiffness; front
  double cf_ = 0.0;
  // corner stiffness; rear
  double cr_ = 0.0;
  // distance between front and rear wheel center
  double wheelbase_ = 0.0;
  // mass of the vehicle
  double mass_ = 0.0;
  // distance from front wheel center to COM
  double lf_ = 0.0;
  // distance from rear wheel center to COM
  double lr_ = 0.0;
  // rotational inertia
  double iz_ = 0.0;
  // the ratio between the turn of the steering wheel and the turn of the wheels
  double steer_transmission_ratio_ = 0.0;
  // the maximum turn of steer
  double steering_max_degree_ = 0.0;
  //vehicle position x now
  double vehicle_x = 0.0 ;
  //vehicle position y now ;
  double vehicle_y = 0.0 ;
  // limit steering to maximum theoretical lateral acceleration
  double max_lat_acc_ = 0.0;

  // number of control cycles look ahead (preview controller)
  int preview_window_ = 0;
  // number of states without previews, includes
  // lateral error, lateral error rate, heading error, heading error rate
  const int basic_state_size_ = 4;
  // vehicle state matrix
  Eigen::MatrixXd matrix_a_;
  // vehicle state matrix (discrete-time)
  Eigen::MatrixXd matrix_ad_;
  // vehicle state matrix compound; related to preview
  Eigen::MatrixXd matrix_adc_;
  // control matrix
  Eigen::MatrixXd matrix_b_;
  // control matrix (discrete-time)
  Eigen::MatrixXd matrix_bd_;
  // control matrix compound
  Eigen::MatrixXd matrix_bdc_;
  // gain matrix
  Eigen::MatrixXd matrix_k_;
  // control authority weighting matrix
  Eigen::MatrixXd matrix_r_;
  // state weighting matrix
  Eigen::MatrixXd matrix_q_;
  // updated state weighting matrix
  Eigen::MatrixXd matrix_q_updated_;
  // vehicle state matrix coefficients
  Eigen::MatrixXd matrix_a_coeff_;
  // 4 by 1 matrix; state matrix
  Eigen::MatrixXd matrix_state_;

  // parameters for lqr solver; number of iterations
  int lqr_max_iteration_ = 0;
  // parameters for lqr solver; threshold for computation
  double lqr_eps_ = 0.0;

  common::DigitalFilter digital_filter_;

  std::unique_ptr<Interpolation1D> lat_err_interpolation_;

  std::unique_ptr<Interpolation1D> heading_err_interpolation_;

  // MeanFilter heading_rate_filter_;
  common::MeanFilter curvature_mean_filter_;
  common::MeanFilter steering_mean_filter_;

  // for logging purpose
  //std::ofstream steer_log_file_;
  FILE *steer_log_file_ = nullptr ;

  const std::string name_;

  double minimum_speed_protection_ = 0.1;
  PIDController steering_pid_controller_ ; //转向PID定义
  PIDController heading_pid_controller_;
  PIDController lateral_pid_controller_; 
  double previous_steering_angle = 0.0 ;
  //double previous_steering_angle_cmd = 0.0;
 // double previous_kappa = 0.0 ;
  //double current_trajectory_timestamp_ = -1.0;

  //5.5
 // double lookahead_station_ = 0.0 ;
 // double lookback_station_ = 0.0 ;
  double driving_orientation = 0.0 ;
  double steer_angle =0.0;
  double steering_angle = 0.0 ;
  double steer_angle_feedforward = 0.0 ;
  double last_steering_angle_cmd = 0.0 ;
  double last_kappa = 0.0 ;
//  int gear_change_num = 0 ;
//  canbus::Chassis::GearPosition preGear;

  //double init_vehicle_heading_ = 0.0;
};

}  // namespace control
}  // namespace jmc_auto

#endif  // MODULES_CONTROL_CONTROLLER_LATERAL_CONTROLLER_H_
