/*****************************************************************************
 * Copyright 2017 The JmcAuto Authors. All Rights Reserved.
 *****************************************************************************/
#include "modules/control/control.h"
//#define _GNU_SOURCE
#include <iomanip>
#include <string>
#include "ros/include/std_msgs/String.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/control/common/control_gflags.h"

#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/decision.pb.h"

namespace jmc_auto {
namespace control {

using jmc_auto::canbus::Chassis;
using jmc_auto::common::ErrorCode;
using jmc_auto::common::Status;
using jmc_auto::common::VehicleStateProvider;
using jmc_auto::common::adapter::AdapterManager;
using jmc_auto::common::monitor::MonitorMessageItem;
using jmc_auto::common::time::Clock;
using jmc_auto::localization::LocalizationEstimate;
using jmc_auto::planning::ADCTrajectory;

std::string Control::Name() const { return FLAGS_control_node_name; }

Status Control::Init() 
{
//  cpu_set_t mask;
//  CPU_ZERO(&mask);
//  CPU_SET(6, &mask);
//  sched_setaffinity(0, sizeof(cpu_set_t), &mask);
  AINFO << "Control init, starting ...";
  init_time_ = Clock::NowInSeconds();//当前时间，单位秒
  CHECK(common::util::GetProtoFromFile(FLAGS_control_conf_file, &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;  //读控制配置文件lincoln.pb.txt
  ADEBUG << "Conf file: " << FLAGS_control_conf_file << " is loaded.";
  AdapterManager::Init(FLAGS_control_adapter_config_filename);//读message消息类别，adapter.conf
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_); //
  // set controller
  if (!controller_agent_.Init(&control_conf_).ok()) 
  {
    std::string error_msg = "Control init controller failed! Stopping...";
    buffer.ERROR(error_msg);
    AINFO << error_msg;
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }//注册控制器，目前只支持LON/LAN/MPC
  // lock it in case for after sub, init_vehicle not ready, but msg trigger
  CHECK(AdapterManager::GetLocalization())<< "Localization is not initialized.";
  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";
  CHECK(AdapterManager::GetPlanning()) << "Planning is not initialized.";
  CHECK(AdapterManager::GetPad()) << "Pad is not initialized.";
  CHECK(AdapterManager::GetMonitor()) << "Monitor is not initialized.";
  CHECK(AdapterManager::GetControlCommand())<< "ControlCommand publisher is not initialized.";
  CHECK(AdapterManager::GetTteContiRadar()) << "TteContiRadar is not initialozed." ;
  AdapterManager::AddPadCallback(&Control::OnPad, this);
  AdapterManager::AddMonitorCallback(&Control::OnMonitor, this);
  return Status::OK();
}

Status Control::Start() 
{
  // set initial vehicle state by cmd通过cmd初始化车辆状态
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so至少需要休眠80ms
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // should init_vehicle first, let car enter work status, then use status msg trigger control触发控制
  AINFO << "Control default driving action is "
        << DrivingAction_Name(control_conf_.action());
  pad_msg_.set_action(control_conf_.action());
  timer_ = AdapterManager::CreateTimer(
      ros::Duration(control_conf_.control_period()), &Control::OnTimer, this);
  AINFO << "Control init done!";
  common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
  buffer.INFO("control started");
  return Status::OK();
}

void Control::OnPad(const PadMessage &pad) 
{
  pad_msg_ = pad;
  AINFO << "Received Pad Msg:" << pad.DebugString();
  // AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";
  // do something according to pad message
  if (pad_msg_.has_action()&&pad_msg_.action() == DrivingAction::RESET) 
  {
    AINFO << "Control received RESET action!";
    estop_ = true;
    estop_reason_.clear();
    pause_num = 0 ;
    pause_status = false ;
    received_reset= true ;
  }else
  {
    received_reset = false ;
  }
  
  if (pad_msg_.action() == DrivingAction::START)
  {
    AINFO << "Control received START action!";
   // estop_ = true;
   // estop_reason_.clear();
   // pause_num = 0 ;
   // pause_status = false ;
    received_reset= false;
  }
 
  if (pad_msg_.has_motion() && pad_msg_.motion()==DrivingMotion::PAUSE)
  {
    ADEBUG << "PAUSE FROM DREAMVIEW" ;
    pause_status = true ;
    pause_num = 0;
  }else if (pad_msg_.has_motion() && pad_msg_.motion()==DrivingMotion::CONTINUE)
  {
    ADEBUG << "CONTINUE FROM DREAMVIEW" ;
    pause_status = false ;
    pause_num = 0 ;
  }
  pad_received_ = true;
}

void Control::OnMonitor(const common::monitor::MonitorMessage &monitor_message)
{
  for (const auto &item : monitor_message.item()) 
  {
    if (item.log_level() == MonitorMessageItem::FATAL) 
    {
      estop_ = true;
      return;
    }
  }
}

void Control::OnTimer(const ros::TimerEvent &) 
{  
  double start_timestamp = Clock::NowInSeconds();
  ControlCommand control_command;
  double end_timestamp =0.0 ;
  double time_diff_ms = 0.0 ;
  Status status ;
  /**************step1 ：检查输入*****************************************/
  status = CheckInput();
  if (!status.ok()) {
    AERROR_EVERY(10) << "Control input data failed: "
                      << status.error_message();//ERROR消息发布频率100hz
    control_command.mutable_engage_advice()->set_advice(
        jmc_auto::common::EngageAdvice::DISALLOW_ENGAGE);
    control_command.mutable_engage_advice()->set_reason(
        status.error_message()); 
    estop_ = true;
    estop_reason_ = status.error_message();
  }else{
     status = CheckTimestamp();
     if (!status.ok()){
        AERROR << "Input messages timeout";
        estop_ = true;
        if (chassis_.driving_mode() !=
            jmc_auto::canbus::Chassis::COMPLETE_AUTO_DRIVE){
               control_command.mutable_engage_advice()->set_advice(
               jmc_auto::common::EngageAdvice::DISALLOW_ENGAGE);
            control_command.mutable_engage_advice()->set_reason(
               status.error_message());
            AINFO << "chassis_.driving_mode() != jmc_auto::canbus::Chassis::COMPLETE_AUTO_DRIVE";
        }
      }else{
         estop_ = false ;
         control_command.mutable_engage_advice()->set_advice(
            jmc_auto::common::EngageAdvice::READY_TO_ENGAGE);
      }
  }
  //---------------------检查输入完毕----------------------------------//
  //-----step2 : 判断是否为暂停模式
  if(!pause_status)
  {
    //-------------非暂停模式，计算控制指令
    pause_status = false ;
    status = ProduceControlCommand(&control_command);
    AERROR_IF(!status.ok()) << "Failed to produce control command:"
                          << status.error_message();
  } else{
     ADEBUG << "To Pause" ;
     //-----------暂停模式，停车，不计算控制指令
     control_command.set_speed(chassis_.speed_mps()-0.2);
     control_command.set_pam_esp_stop_distance(100) ;

     if (control_command.speed() < 0)
     {
       control_command.set_speed(0) ;
       control_command.set_pam_esp_stop_distance(0) ;
     }
     control_command.set_gear_location(chassis_.gear_location());
     ADEBUG << "pause_num " << pause_num ;
     if(pause_num == 0)
     {
        control_command.set_steering_angle(chassis_.steering_percentage());
     }else{
        control_command.set_steering_angle(last_steering_angle_command);  
     }
     last_steering_angle_command = control_command.steering_angle();
     ++pause_num ;
  }
    end_timestamp = Clock::NowInSeconds();
   
   //TODO: --------step3：是否收到pad消息以及到终点方向盘自动回正功能
    if (pad_received_) {
      if (received_reset)
      {
        if (chassis_.steering_percentage() >10)
        {
          control_command.set_steering_angle(chassis_.steering_percentage() - 10) ;
          pad_received_ = true ;
          received_reset = true ;
        }else if(chassis_.steering_percentage() < -10)
        {
           control_command.set_steering_angle(chassis_.steering_percentage() + 10) ;
           pad_received_ = true ;
           received_reset = true ;
        }else
        {
          pad_msg_.set_action(DrivingAction::RESET);
          control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
          pad_received_ = false ;
          received_reset = false ;
        }
      }else
      {
        control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
        pad_received_ = false ;
        received_reset = false ;
      }
    }//pad
    time_diff_ms = (end_timestamp - start_timestamp) * 1000;
    control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
    control_command.mutable_latency_stats()->set_total_time_exceeded(
    time_diff_ms < control_conf_.control_period());
    AINFO << "control cycle time is: " << time_diff_ms << " ms.";
    status.Save(control_command.mutable_header()->mutable_status());
    if (estop_){
      control_command.mutable_header()->mutable_status()->set_msg(estop_reason_);
      estop_ = false ;
      AINFO << "Reset estop to false" ;
    }
    //先打方向，再启动车辆
    ADEBUG << "chassis_.driving_mode " << chassis_.driving_mode() ;
    if(chassis_.driving_mode() == canbus::Chassis::COMPLETE_AUTO_DRIVE && chassis_.speed_mps() == 0 && (!(chassis_.gear_location() == canbus::Chassis::GEAR_PARKING)))
    {
	ADEBUG << "Running speed_num++" ;
	speed_num++ ;
    }else
    {
	speed_num = 0 ;
    }
    ADEBUG << "speed_num" << speed_num ;
    double steering_angle_command_diff = std::fabs(control_command.steering_angle() -chassis_.steering_percentage()) ;
    if (speed_num > 20 && steering_angle_command_diff > 20)
    {
      ADEBUG << "First turning";
      control_command.set_speed(0);
      control_command.set_pam_esp_stop_distance(0);
    }
    
    if (control_command.speed() == 0 || control_command.pam_esp_stop_distance() == 0)
    {
      control_command.set_pam_esp_stop_distance(0);
      control_command.set_speed(0);
    }

    AINFO << "The send_speed is: " << control_command.speed();
    AINFO << "The send_distance is: " << control_command.pam_esp_stop_distance();
    AINFO << "The send_steering angle is: " << control_command.steering_angle();
    //last_chassis_steering_angle = control_command.steering_angle() ;
    SendCmd(&control_command);
}

Status Control::ProduceControlCommand(ControlCommand *control_command) {
   Status status ;
   double Radar_speed = 0.0;
   CheckRadarDistance(Radar_speed);
  // check estop,if planning set estop, then no control process triggered
  estop_ = FLAGS_enable_persistent_estop ?
    estop_ || trajectory_.estop().is_estop() : trajectory_.estop().is_estop();

  if (!estop_) {
    //estop_ = false
     pause_num = 0 ;
     //如果当前是MANUAL,
    if (chassis_.driving_mode() == Chassis::COMPLETE_MANUAL) {
      controller_agent_.Reset();//
      AINFO_EVERY(10) << "Reset Controllers in Manual Mode";//手动模式下重置控制器
    }
    auto debug = control_command->mutable_debug()->mutable_input_debug();
    debug->mutable_localization_header()->CopyFrom(localization_.header());
    debug->mutable_canbus_header()->CopyFrom(chassis_.header());
    debug->mutable_trajectory_header()->CopyFrom(trajectory_.header());
    Status status_compute = controller_agent_.ComputeControlCommand(
        &localization_, &chassis_, &trajectory_, control_command);
    if (!status_compute.ok()) {
      AERROR << "Control main function failed"
             << " with localization: " << localization_.ShortDebugString()
             << " with chassis: " << chassis_.ShortDebugString()
             << " with trajectory: " << trajectory_.ShortDebugString()
             << " with cmd: " << control_command->ShortDebugString()
             << " status:" << status_compute.error_message();
     // estop_ = true;
      estop_reason_ = status_compute.error_message();
      status = status_compute;
    }
  }else if(estop_ ||(control_command->speed()==0 && control_command->pam_esp_stop_distance()==0)){
     //急停模式
    if (trajectory_.estop().is_estop()) {
    estop_reason_ = "estop from planning";
    AINFO << estop_reason_ ;
    }
   // control_command->set_speed(chassis_.speed_mps()-0.2);
    control_command->set_speed(0);
    control_command->set_pam_esp_stop_distance(0);

   control_command->set_gear_location(chassis_.gear_location());
    ADEBUG << "ESTOP pause num " << pause_num ;
   if(pause_num == 0){
    control_command->set_steering_angle(chassis_.steering_percentage());
    }else{
      control_command->set_steering_angle(last_steering_angle_command);
    }
    last_steering_angle_command = control_command->steering_angle() ;
    ++pause_num ;
    control_command->set_gear_location(chassis_.gear_location());
    }
  // check 转向信号等
    if (trajectory_.decision().has_vehicle_signal()){
       control_command->mutable_signal()->CopyFrom(
         trajectory_.decision().vehicle_signal());
  }
  return status;
}

Status Control::CheckInput() {
  AdapterManager::Observe();
  auto localization_adapter = AdapterManager::GetLocalization();
  if (localization_adapter->Empty())
  {
    AERROR_EVERY(100) << "No Localization msg yet. ";
    //AINFO << localization_adapter.ShortDebugString();
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No localization msg");
  }
  localization_ = localization_adapter->GetLatestObserved();//返回观察队列最新数据，调用之前需调用Empty()确定是否有数据
  AINFO << "Received localization:" << localization_.ShortDebugString();//定位数据

  auto chassis_adapter = AdapterManager::GetChassis();
  if (chassis_adapter->Empty())
  {
    AERROR_EVERY(100) << "No Chassis msg yet. ";
    //AINFO << chassis_adapter.ShortDebugString();
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No chassis msg");
  }
  chassis_ = chassis_adapter->GetLatestObserved();//底盘数据
  AINFO << "Received chassis:" << chassis_.ShortDebugString();

  auto trajectory_adapter = AdapterManager::GetPlanning();
  if (trajectory_adapter->Empty())
  {
    AERROR_EVERY(100) << "No planning msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No planning msg");
  }
  trajectory_ = trajectory_adapter->GetLatestObserved();//planning轨迹点
  AINFO << "Received trajectory" ;
  if (trajectory_.estop().is_estop()||trajectory_.trajectory_point_size() == 0) {
    AERROR_EVERY(100) << "planning has no trajectory point. ";
    AERROR << "trajectory_: " << trajectory_.ShortDebugString();
    if(trajectory_.estop().is_estop()){
    ADEBUG << "Planning estop!" ;
   }
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "planning has no trajectory point.");

  } else{
  for (auto &trajectory_point : *trajectory_.mutable_trajectory_point())
  {
    if (std::fabs(trajectory_point.v()) < control_conf_.minimum_speed_resolution())
    {
      ADEBUG << "Trajectory_point v : " << trajectory_point.v() ;
      ADEBUG << "Control_conf minimum speed resolution" << control_conf_.minimum_speed_resolution() ;
      trajectory_point.set_v(0.0);
      trajectory_point.set_a(0.0);
      ADEBUG << "There are trajectroy points with velocity zero!";
    }
   }
}
  VehicleStateProvider::instance()->Update(localization_, chassis_);

  auto tte_conti_radar_adapter = AdapterManager::GetTteContiRadar();
  if (!tte_conti_radar_adapter->Empty()) {
    tte_radar_ = tte_conti_radar_adapter->GetLatestObserved();
    ADEBUG << "Received Radar message." ;
  }else{
   ADEBUG << "NO radar message.";
}  

  AINFO << "Input no problem!" ;
  return Status::OK();

}
Status Control::CheckTimestamp()
{
  if (!FLAGS_enable_input_timestamp_check || FLAGS_is_control_test_mode)
  {
    AINFO << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  double current_timestamp = Clock::NowInSeconds();
  double localization_diff =
      current_timestamp - localization_.header().timestamp_sec();
  if (localization_diff >
      (FLAGS_max_localization_miss_num * control_conf_.localization_period()))
  {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Localization msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }
  else
  {
    AINFO << "Localization msg timestamp is normal!" ;
  }

  double chassis_diff = current_timestamp - chassis_.header().timestamp_sec();
  if (chassis_diff >(FLAGS_max_chassis_miss_num * control_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Chassis msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }
  else
  {
    AINFO << "Chassis msg timestamp is normal!" ;
  }

  double trajectory_diff = current_timestamp - trajectory_.header().timestamp_sec();
  if (trajectory_diff > (FLAGS_max_planning_miss_num * control_conf_.trajectory_period()))
  {
    AERROR << "Trajectory msg lost for " << std::setprecision(6)
           << trajectory_diff << "s";
    common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR("Trajectory msg lost");
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory msg timeout");
  }
   else
  {
    AINFO << "Trajectory msg timestamp is normal!" ;
  }

  double tte_radar_diff = current_timestamp - tte_radar_.header().timestamp_sec();
  AINFO << "tte radar diff is: " << tte_radar_diff;

  return Status::OK();
}

void Control::SendCmd(ControlCommand *control_command) {
  // set header
  AdapterManager::FillControlCommandHeader(Name(), control_command);

 AdapterManager::PublishControlCommand(*control_command);
 AINFO << "Control command pubilsh succeed! Control command msg:" 
       << control_command->ShortDebugString();
}

void Control::Stop() {}

void Control::CheckRadarDistance(double & Radar_speed){
    if (!(tte_radar_.has_debug_apareardistanceinfo_457()||tte_radar_.has_debug_apafrontdistanceinfo_458())){
      ADEBUG << "No tte_radar data!!!";
      return ;
    }
    if (trajectory_.decision().main_decision().has_parking()){
      ADEBUG << "PARKING MODE" ;
          if(chassis_.gear_location()==canbus::Chassis::GEAR_REVERSE){
            if (tte_radar_.debug_apareardistanceinfo_457().aparrm_distance()<= 150 
            ||tte_radar_.debug_apareardistanceinfo_457().aparlm_distance() <= 150 
            ||tte_radar_.debug_apareardistanceinfo_457().aparr_distance() <= 150 
            ||tte_radar_.debug_apareardistanceinfo_457().aparl_distance() <= 150){
              //Radar_speed = 0.25;
              ADEBUG << "slow down from tte radar,parking reverse" ;   
              PrintfRadarDistance();
            }
                if(tte_radar_.debug_apareardistanceinfo_457().aparrm_distance()<= FLAGS_safe_distance_level_2||
                      tte_radar_.debug_apareardistanceinfo_457().aparlm_distance() <= FLAGS_safe_distance_level_2 ||
                      tte_radar_.debug_apareardistanceinfo_457().aparr_distance() <= FLAGS_safe_distance_level_1 ||
                      tte_radar_.debug_apareardistanceinfo_457().aparl_distance() <= FLAGS_safe_distance_level_1 ||
                      tte_radar_.debug_apareardistanceinfo_457().aparls_distance()*100 <= FLAGS_safe_distance_level_0 ||
                      tte_radar_.debug_apareardistanceinfo_457().aparrs_distance()*100 <= FLAGS_safe_distance_level_0||
                      tte_radar_.debug_apafrontdistanceinfo_458().apafr_distance() <= FLAGS_safe_distance_level_0 ||
                      tte_radar_.debug_apafrontdistanceinfo_458().apafl_distance()<=FLAGS_safe_distance_level_0 ||
                      tte_radar_.debug_apafrontdistanceinfo_458().apafrs_distance()*100 <=FLAGS_safe_distance_level_0 ||
                      tte_radar_.debug_apafrontdistanceinfo_458().apafls_distance()*100<=FLAGS_safe_distance_level_0
                      ){
                        estop_ = true ;    
                        ADEBUG << "Estop from tte radar,parking reverse" ;   
                      PrintfRadarDistance() ;
                }
          }else{
            if(tte_radar_.debug_apafrontdistanceinfo_458().apaflm_distance() <= FLAGS_safe_distance_level_5||
                  tte_radar_.debug_apafrontdistanceinfo_458().apafrm_distance() <= FLAGS_safe_distance_level_5 ||
                  tte_radar_.debug_apafrontdistanceinfo_458().apafr_distance() <= FLAGS_safe_distance_level_1 ||
                  tte_radar_.debug_apafrontdistanceinfo_458().apafl_distance() <= FLAGS_safe_distance_level_1 ||
                  tte_radar_.debug_apafrontdistanceinfo_458().apafls_distance()*100 <= FLAGS_safe_distance_level_0 ||
                  tte_radar_.debug_apafrontdistanceinfo_458().apafrs_distance()*100 <= FLAGS_safe_distance_level_0||
                  tte_radar_.debug_apareardistanceinfo_457().aparr_distance() <= FLAGS_safe_distance_level_1 ||
                  tte_radar_.debug_apareardistanceinfo_457().aparl_distance() <= FLAGS_safe_distance_level_1 ||
                  tte_radar_.debug_apareardistanceinfo_457().aparls_distance()*100 <= FLAGS_safe_distance_level_0 ||
                  tte_radar_.debug_apareardistanceinfo_457().aparrs_distance()*100 <= FLAGS_safe_distance_level_0){
              estop_ = true ;    
              ADEBUG << "Estop from tte radar,parking ,drive." ;   
            //s is m ,no s is cm
              PrintfRadarDistance() ;
            }
          }
    } else{
        ADEBUG << "DRIVE MODE" ;
        if(tte_radar_.debug_apafrontdistanceinfo_458().apaflm_distance() <= FLAGS_safe_distance_level_5||
            tte_radar_.debug_apafrontdistanceinfo_458().apafrm_distance() <= FLAGS_safe_distance_level_5 ||
            tte_radar_.debug_apafrontdistanceinfo_458().apafr_distance() <= FLAGS_safe_distance_level_2 ||
            tte_radar_.debug_apafrontdistanceinfo_458().apafl_distance() <= FLAGS_safe_distance_level_2 ||
            tte_radar_.debug_apafrontdistanceinfo_458().apafls_distance()*100 <= FLAGS_safe_distance_level_0 ||
            tte_radar_.debug_apafrontdistanceinfo_458().apafrs_distance()*100 <= FLAGS_safe_distance_level_0 ||
            tte_radar_.debug_apareardistanceinfo_457().aparls_distance()*100 <= FLAGS_safe_distance_level_0 ||
            tte_radar_.debug_apareardistanceinfo_457().aparrs_distance()*100 <= FLAGS_safe_distance_level_0){
              estop_ = true ;    
              ADEBUG << "Estop from tte radar,drive." ;   
            //s is m ,no s is cm
              PrintfRadarDistance() ;
            }
      }
     return;
}

void Control::PrintfRadarDistance(){
        ADEBUG <<"frs " <<tte_radar_.debug_apafrontdistanceinfo_458().apafrs_distance() * 100 ;
        ADEBUG <<"fls " <<tte_radar_.debug_apafrontdistanceinfo_458().apafls_distance() * 100 ;
        ADEBUG <<"frm " <<tte_radar_.debug_apafrontdistanceinfo_458().apafrm_distance() ;
        ADEBUG <<"flm " <<tte_radar_.debug_apafrontdistanceinfo_458().apaflm_distance() ;
        ADEBUG <<"fr " <<tte_radar_.debug_apafrontdistanceinfo_458().apafr_distance() ;
        ADEBUG <<"fl " <<tte_radar_.debug_apafrontdistanceinfo_458().apafl_distance() ;
        ADEBUG <<"rrm " <<tte_radar_.debug_apareardistanceinfo_457().aparrm_distance() ;
        ADEBUG <<"rlm " <<tte_radar_.debug_apareardistanceinfo_457().aparlm_distance() ;
        ADEBUG <<"rr " <<tte_radar_.debug_apareardistanceinfo_457().aparr_distance() ;
        ADEBUG <<"rl " <<tte_radar_.debug_apareardistanceinfo_457().aparl_distance() ;
        ADEBUG <<"rls " <<tte_radar_.debug_apareardistanceinfo_457().aparls_distance()*100 ;
        ADEBUG <<"rrs " <<tte_radar_.debug_apareardistanceinfo_457().aparrs_distance()*100;
        return ;
}

}  // namespace control
}  // namespace jmc_auto
