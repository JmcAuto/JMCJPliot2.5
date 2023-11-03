#ifndef MODULES_REMOTECONTROL_REMOTECONTROL_H_
#define MODULES_REMOTECONTROL_REMOTECONTROL_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <sstream>
#include <thread>
#include <mutex>
#include "ros/include/ros/ros.h"

#include "modules/common/jmc_auto_app.h"
#include "modules/common/macro.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
// #include "modules/control/proto/control_cmd.pb.h"
// #include "modules/guardian/proto/guardian.pb.h"
#include "modules/remotecontrol/udp_stream/udp_data.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/canbus/proto/vehicle_chassis.pb.h"
#include "modules/remotecontrol/proto/remote_control.pb.h"
#include "modules/remotecontrol/proto/remote_control_conf.pb.h"
#include "modules/remotecontrol/common/speed_control.h"
/**
 * @namespace jmc_auto::canbus
 * @brief jmc_auto::canbus
 */
namespace jmc_auto
{
namespace remotecontrol
{

/**
 * @class Canbus
 *
 * @brief canbus module main class.
 * It processes the control data to send protocol messages to can card.
 */
class RemoteControl : public jmc_auto::common::JmcAutoApp
{
public:
  RemoteControl()
      : monitorger_(jmc_auto::common::monitor::MonitorMessageItem::REMOTECONTROL) {}
  ~RemoteControl(){Stop();}
  /**
   * @brief obtain module name
   * @return module name
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

private:
  // void OnChassis();
  //void PublishChassisDetail();
  // void OnTimer(const ros::TimerEvent &event);
  // void PublishControlCommand(const jmc_auto::control::ControlCommand &control_command);
  // void PublishControlCommand(const JMC_AUTO::TCP::TCP_DATA &control_command);
  //   void OnGuardianCommand(
  //       const jmc_auto::guardian::GuardianCommand &guardian_command);
  // jmc_auto::common::Status OnError(const std::string &error_msg);

  jmc_auto::canbus::Vehicle_Chassis tcp_chassis(const jmc_auto::canbus::Chassis &chassis_);
  void OnChassis(const jmc_auto::canbus::Chassis &chassis);
  void emergency_stop(jmc_auto::remote::RemoteControl &RemoteControlCommand);
  void UDPThreadFunc();
  jmc_auto::control::ControlCommand RemoteCmdToControlCmd(const jmc_auto::remote::RemoteControl &RemoteControlCommand);

  int count = 0;

  char send_buffer[512] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
  char recive_buffer[2048];
  int recive_byte;
  int send_data_flag = 0;
  int send_data_count = 0;
  bool remote_mode = false;
  bool stop_mode = false;
  bool is_vehcile_stop = false;
  bool is_stop_mode = false;
  bool is_steer_zero=false;
  double currenttimestamp;
  double timeouttimestamp;
  double wait_time = 2;
  float vehicle_speed=0;
  bool is_running_=false;
  // jmc_auto::udp::UdpStream udp_vehcile("192.168.1.109", 8000, 100);
  jmc_auto::remote::RemoteControlConf Remote_Control_Conf_;
  SpeedControl speed_control;
  jmc_auto::canbus::Chassis chassis_;
  std::mutex chassis_mutex;
  std::unique_ptr<std::thread> thread_;
  int64_t last_timestamp_ = 0;
  ros::Timer timer_;
  jmc_auto::common::monitor::MonitorLogger monitorger_;
};

} // namespace remotecontrol
} // namespace jmc_auto

#endif // MODULES_REMOTECONTROL_REMOTECONTROL_H_
