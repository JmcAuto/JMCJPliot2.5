#ifndef MODULES_REMOTECONTROL_SPEED_CONTROL_H_
#define MODULES_REMOTECONTROL_SPEED_CONTROL_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/filters/digital_filter.h"
#include "modules/common/filters/digital_filter_coefficients.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/proto/lon_controller_conf.pb.h"
#include "modules/control/common/interpolation_2d.h"
#include "modules/control/common/pid_controller.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/remotecontrol/proto/remote_control.pb.h"
#include "modules/control/proto/control_conf.pb.h"


/**
 * @namespace jmc_auto::remotecontrol
 * @brief jmc_auto::remotecontrol
 */
namespace jmc_auto {
namespace remotecontrol {

/**
 * @class LonController
 *
 * @brief Longitudinal controller, to compute brake / throttle values.
 */
class SpeedControl {
 public:
  /**
   * @brief constructor
   */
  SpeedControl();

  /**
   * @brief destructor
   */
  virtual ~SpeedControl();

  /**
   * @brief initialize Longitudinal Controller
   * @param control_conf control configurations
   * @return Status initialization status
   */
  common::Status Init() ;

  /**
   * @brief compute brake / throttle values based on current vehicle status
   *        and target trajectory
   * @param target vehicle speed
   * @param chassis vehicle status e.g., speed, acceleration
   * @param trajectory trajectory generated by planning
   * @param cmd control command
   * @return Status computation status
   */
  common::Status ComputeControlCommand(
      const float Targrt_vehicle_speed,
      const canbus::Chassis *chassis,
      jmc_auto::remote::RemoteControl *cmd) ;

  /**
   * @brief reset longitudinal controller
   * @return Status reset status
   */
  common::Status Reset() ;

  /**
   * @brief stop longitudinal controller
   */
  void Stop();

  /**
   * @brief longitudinal controller name
   * @return string controller name in string
   */
  std::string Name() const;



 private:
 
 void SetDigitalFilterPitchAngle(const control::LonControllerConf &lon_controller_conf);
 void SetDigitalFilter(double ts, double cutoff_freq,
                        common::DigitalFilter *digital_filter);
  void LoadControlCalibrationTable(
      const control::LonControllerConf &lon_controller_conf);

  void CloseLogFile();

//   const canbus::Chassis *chassis_ = nullptr;

  std::unique_ptr<control::Interpolation2D> control_interpolation_;
  std::string name_;
  bool controller_initialized_ = false;

  control::PIDController speed_pid_controller_;

  FILE *speed_log_file_ = nullptr;

  common::DigitalFilter digital_filter_pitch_angle_;

  control::ControlConf control_conf_;
  control::LonControllerConf lon_controller_conf;

  // vehicle parameter
  common::VehicleParam vehicle_param_;
};
}  // namespace control
}  // namespace jmc_auto
#endif  // MODULES_REMOTECONTROL_SPEED_CONTROL_H_
