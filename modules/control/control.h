
#ifndef MODULES_CONTROL_CONTROL_H_
#define MODULES_CONTROL_CONTROL_H_

#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/control_conf.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"

#include "modules/common/jmc_auto_app.h"
#include "modules/common/util/util.h"
#include "modules/control/controller/controller_agent.h"
#include "modules/drivers/proto/tte_conti_radar.pb.h"

/**
 * @namespace jmc_auto::control
 * @brief jmc_auto::control
 */
namespace jmc_auto {
namespace control {

/**
 * @class Control
 *
 * @brief control module main class, it processes localization, chasiss, and
 * pad data to compute throttle, brake and steer values.
 */
class Control : public jmc_auto::common::JmcAutoApp {
  friend class ControlTestBase;

 public:
  Control()
      : monitor_logger_(jmc_auto::common::monitor::MonitorMessageItem::CONTROL) {}

  /**
   * @brief module name
   */
  std::string Name() const override;

  /**
   * @brief module initialization function
   * @return initialization status
   */
  jmc_auto::common::Status Init() override;

  /**
   * @brief module start function
   * @return start status
   */
  jmc_auto::common::Status Start() override;

  /**
   * @brief module stop function
   */
  void Stop() override;

  /**
   * @brief destructor
   */
  virtual ~Control() = default;

 private:
  // Upon receiving pad message
  void OnPad(const jmc_auto::control::PadMessage &pad);
  // Upon receiving monitor message
  void OnMonitor(
      const jmc_auto::common::monitor::MonitorMessage &monitor_message);
  // Watch dog timer
  void OnTimer(const ros::TimerEvent &);
  common::Status ProduceControlCommand(ControlCommand *control_command);
  common::Status CheckInput();
  common::Status CheckTimestamp();
  common::Status CheckPad();
  void CheckRadarDistance( double &Radar_speed ) ;
  void PrintfRadarDistance() ;
  void SendCmd(ControlCommand *control_command);
 private:
  double init_time_ = 0.0;
  localization::LocalizationEstimate localization_;
  canbus::Chassis chassis_;
  planning::ADCTrajectory trajectory_;
  drivers::TteContiRadar tte_radar_;
  PadMessage pad_msg_;
  ControllerAgent controller_agent_;
  ControlConf control_conf_;
  jmc_auto::common::monitor::MonitorLogger monitor_logger_;
  bool estop_ = false;
  std::string estop_reason_;
  bool pad_received_ = false;
  bool pause_status =false ;
  double last_chassis_steering_angle = 0;
  double last_steering_angle_command = 0;
  int pause_num ;
  int speed_num = 0 ;
  bool received_reset = false;
  ros::Timer timer_;
};

}  // namespace control
}  // namespace jmc_auto

#endif  // MODULES_CONTROL_CONTROL_H_
