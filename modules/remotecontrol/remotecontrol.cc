#include <sstream>
#include "ros/ros.h"
#include <cstring>
#include "modules/remotecontrol/common/remotecontrol_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/time/time.h"
#include "modules/common/log.h"
#include "modules/remotecontrol/remotecontrol.h"

// INFO
// #include "modules/common/monitor/monitor_buffer.h"
// #include "modules/common/monitor/monitor.h"

namespace jmc_auto
{
    namespace remotecontrol
    {
        // using jmc_auto::canbus::Chassis;
        using jmc_auto::canbus::Chassis;
        using jmc_auto::canbus::Vehicle_Chassis;
        using jmc_auto::common::Status;
        using jmc_auto::common::adapter::AdapterManager;
        using jmc_auto::control::ControlCommand;
        // using jmc_auto::remote::GearPosition;

        std::string RemoteControl::Name() const { return FLAGS_remotecontrol_module_name; }
        // jmc_auto::udp::UdpStream udp_vehcile("192.168.1.109", 8000, 100);
        Status RemoteControl::Init()
        {
            AdapterManager::Init(FLAGS_remotecontrol_adapter_config_filename);
            AINFO << "The adapter manager is successfully initialized.";
            common::util::GetProtoFromFile(FLAGS_remotecontrol_conf_file, &Remote_Control_Conf_);
            AINFO << Remote_Control_Conf_.DebugString();
            AdapterManager::AddChassisCallback(&RemoteControl::OnChassis, this);
            speed_control.Init();

            return Status::OK();
        }

        Status RemoteControl::Start()
        {
            // udp_vehcile.open();
            // udp_vehcile.connect();
            // const double duration = 1.0 / FLAGS_remotecontrol_freq;
            // timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
            //                                      &RemoteControl::OnTimer, this);
            is_running_ = true;
            thread_.reset(new std::thread([this] { UDPThreadFunc(); }));
            jmc_auto::common::monitor::MonitorLogBuffer buffer(&monitorger_);
            buffer.INFO("RemoteControl is started.");
            return Status::OK();
        }

        Vehicle_Chassis RemoteControl::tcp_chassis(const jmc_auto::canbus::Chassis &chassis_)
        {
            Vehicle_Chassis remote_chassis;
            if (chassis_.has_speed_mps())
            {
                remote_chassis.set_speed_mps(chassis_.speed_mps());
            }
            if (chassis_.has_brake_percentage())
            {
                remote_chassis.set_brake_percentage(chassis_.brake_percentage());
            }
            if (chassis_.has_throttle_percentage())
            {
                remote_chassis.set_throttle_percentage(chassis_.throttle_percentage());
            }

            if (chassis_.has_steering_percentage())
            {
                remote_chassis.set_steering_percentage(chassis_.steering_percentage());
            }
            if (chassis_.has_driving_mode())
            {
                remote_chassis.set_driving_mode(chassis_.driving_mode());
            }
            if (chassis_.has_error_code())
            {
                remote_chassis.set_error_code(chassis_.error_code());
            }
            if (chassis_.has_gear_location())
            {
                remote_chassis.set_gear_location(chassis_.gear_location());
            }
            return remote_chassis;
        }

        void RemoteControl::OnChassis(const Chassis &chassis)
        {
            int recive_byte;
            ROS_INFO("Reci remote_control_topic.\n");
            if (!chassis.has_speed_mps())
            {
                AERROR << "chassis has not speed_mps";
            }

            {
                std::unique_lock<std::mutex> lck(chassis_mutex);
                chassis_ = chassis;
            }
            

            Vehicle_Chassis remote_chassis = tcp_chassis(chassis);
            recive_byte = remote_chassis.ByteSize();
            remote_chassis.SerializeToArray(send_buffer, recive_byte);
            send_data_flag = 1;
            send_data_count = recive_byte;
            if (remote_chassis.speed_mps() == 0)
            {
                is_vehcile_stop = true;
            }
            else
            {
                is_vehcile_stop = false;
            }
            if (chassis.driving_mode() == Chassis::DrivingMode::Chassis_DrivingMode_AUTO_SPEED_ONLY)
            {
                is_stop_mode = true;
            }
            else
            {
                is_stop_mode = false;
            }
            if (remote_chassis.steering_percentage() == 0)
            {
                is_steer_zero = true;
            }
            else
            {
                is_steer_zero = false;
            }

            AINFO << "get chassis:" + remote_chassis.DebugString();
        }
        void RemoteControl::emergency_stop(jmc_auto::remote::RemoteControl &RemoteControlCommand)
        {
            if (is_vehcile_stop)
            {
                RemoteControlCommand.set_pedal_throttle_percent(0);
                RemoteControlCommand.set_pedal_brake_percent(0);
                RemoteControlCommand.set_gear_data(Chassis::GEAR_PARKING);
                RemoteControlCommand.set_mode_apply(Chassis::DrivingMode::Chassis_DrivingMode_AUTO_SPEED_ONLY);
            }
            else
            {
                RemoteControlCommand.set_pedal_throttle_percent(0);
                RemoteControlCommand.set_pedal_brake_percent(70);
                RemoteControlCommand.set_mode_apply(Chassis::DrivingMode::Chassis_DrivingMode_AUTO_SPEED_ONLY);
            }
            AINFO << "get emergency_stop:";
        }
    

        void RemoteControl::UDPThreadFunc()
        {

            jmc_auto::remote::RemoteControl RemoteControlCommand;
            ControlCommand control_command;
            std::chrono::duration<double, std::micro> default_period{2 * 1000};
            // jmc_auto::udp::UdpStream udp_vehcile("202.109.144.72", 8000, 100);
            //  jmc_auto::udp::UdpStream udp_vehcile("192.168.0.101", 8000, 100);
            // const char* address=Remote_Control_Conf_.address().data();
            jmc_auto::udp::UdpStream udp_vehcile(Remote_Control_Conf_.address().data(), Remote_Control_Conf_.port(), Remote_Control_Conf_.timeout_usec());
            udp_vehcile.open();
            udp_vehcile.connect();
            while (1)
            {
                currenttimestamp = jmc_auto::common::time::Clock::NowInSeconds();

                AINFO << "currenttimestamp:" << currenttimestamp;
                RemoteControlCommand.Clear();
                recive_byte = udp_vehcile.read((uint8_t *)recive_buffer, 1024);
                udp_vehcile.write((const uint8_t *)send_buffer, send_data_count);
                ROS_INFO("recive_byte is %d.\n", recive_byte);
                AINFO << "recive_byte:" << recive_byte;

                if (recive_byte > 0)
                {
                    timeouttimestamp = currenttimestamp + wait_time;
                    AINFO << "timeouttimestamp:" << timeouttimestamp;
                    ++count;
                    RemoteControlCommand.ParseFromArray(recive_buffer, recive_byte);
                    AINFO << "get RemoteControlCommand:" << RemoteControlCommand.DebugString();
                    AINFO << "currenttimestamp:-ros" << ros::Time::now().toSec() - RemoteControlCommand.stamp();
                    //
                    // if (RemoteControlCommand.has_pedal_throttle_percent())
                    // {
                    //     speed_control.ComputeControlCommand(RemoteControlCommand.pedal_throttle_percent(), &chassis_, &control_command);
                    //     //control_command.set_throttle(RemoteControlCommand.pedal_throttle_percent());
                    // }
                    // if (RemoteControlCommand.has_pedal_brake_percent())
                    // {
                    //     if (RemoteControlCommand.pedal_brake_percent() != 0)
                    //     {
                    //         control_command.set_throttle(0);
                    //     }

                    //     control_command.set_brake(RemoteControlCommand.pedal_brake_percent());
                    // }
                    //
                    speed_control.ComputeControlCommand(RemoteControlCommand.pedal_throttle_percent(), &chassis_, &RemoteControlCommand);
                    if (RemoteControlCommand.emergency_stop())
                    {
                        emergency_stop(RemoteControlCommand);
                    }
                    AINFO << "get control_command" + control_command.DebugString();
                    AdapterManager::PublishRemoteControl(RemoteControlCommand);
                }
                else
                {
                    if (currenttimestamp > timeouttimestamp)
                    {
                        emergency_stop(RemoteControlCommand);
                        ROS_INFO("timeouttimestamp.\n");
                        // chatter_pub.publish(control_command);

                        // control_command.set_steering_target(0);
                        AINFO << "get timeout control_command" + control_command.DebugString();
                        AdapterManager::PublishRemoteControl(RemoteControlCommand);
                        AINFO << "timeout over 2";
                    }
                    AINFO << "timeout";
                }

                if (send_data_flag == 1)
                {
                    send_data_flag = 0;
                    udp_vehcile.write((const uint8_t *)send_buffer, send_data_count);
                    send_data_count = 0;
                }
                std::this_thread::sleep_for(default_period);
            }
        }

        void RemoteControl::Stop()
        {
            if (is_running_)
            {
                AINFO << "Stopping UDPThreadFunc ...";
                is_running_ = false;
                if (thread_ != nullptr && thread_->joinable())
                {
                    thread_->join();
                }
                thread_.reset();
            }
            else
            {
                AINFO << "Can client receiver is not running.";
            }
            AINFO << "Can client receiver stopped [ok].";
        }
    } // namespace remotecontrol
} // namespace jmc_auto
