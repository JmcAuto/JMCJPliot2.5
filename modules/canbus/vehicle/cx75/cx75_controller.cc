/* Copyright 2017 The jmc_auto Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/cx75/cx75_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "modules/canbus/vehicle/cx75/cx75_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace jmc_auto
{
  namespace canbus
  {
    namespace cx75
    {

      using ::jmc_auto::common::ErrorCode;
      using ::jmc_auto::control::ControlCommand;
      using ::jmc_auto::drivers::canbus::ProtocolData;

      namespace
      {

        const int32_t kMaxFailAttempt = 25;
        const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
        const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
        const int32_t CHECK_RESPONSE_APA_UNIT_FLAG = 4;
        const int32_t CHECK_RESPONSE_ESPPAM_UNIT_FLAG = 8;
      } // namespace

      ErrorCode Cx75Controller::Init(
          const VehicleParameter &params,
          CanSender<::jmc_auto::canbus::ChassisDetail> *const can_sender,
          MessageManager<::jmc_auto::canbus::ChassisDetail> *const message_manager)
      {
        if (is_initialized_)
        {
          AINFO << "Cx75Controller has already been initiated.";
          return ErrorCode::CANBUS_ERROR;
        }

        params_.CopyFrom(params);
        if (!params_.has_driving_mode())
        {
          AERROR << "Vehicle conf pb not set driving_mode.";
          return ErrorCode::CANBUS_ERROR;
        }

        if (can_sender == nullptr)
        {
          return ErrorCode::CANBUS_ERROR;
        }
        can_sender_ = can_sender;

        if (message_manager == nullptr)
        {
          AERROR << "protocol manager is null.";
          return ErrorCode::CANBUS_ERROR;
        }
        message_manager_ = message_manager;

        // sender part
        pam_0x270_270_ = dynamic_cast<Pam0x270270 *>(message_manager_->GetMutableProtocolDataById(Pam0x270270::ID));
        if (pam_0x270_270_ == nullptr)
        {
          AERROR << "Pam0x270270 does not exist in the Cx75MessageManager!";
          return ErrorCode::CANBUS_ERROR;
        }

        //pam_0x271_271_ = dynamic_cast<Pam0x271271 *>(message_manager_->GetMutableProtocolDataById(Pam0x271271::ID));
        //if (pam_0x271_271_ == nullptr)
        //{
        //  AERROR << "Pam0x271271 does not exist in the Cx75MessageManager!";
        //  return ErrorCode::CANBUS_ERROR;
        //}

        pam_0x272_272_ = dynamic_cast<Pam0x272272 *>(message_manager_->GetMutableProtocolDataById(Pam0x272272::ID));
        if (pam_0x272_272_ == nullptr)
        {
          AERROR << "Pam0x272272 does not exist in the Cx75MessageManager!";
          return ErrorCode::CANBUS_ERROR;
        }

        mrr_0x238_238_ = dynamic_cast<Mrr0x238238 *>(message_manager_->GetMutableProtocolDataById(Mrr0x238238::ID));
        if (mrr_0x238_238_ == nullptr)
        {
          AERROR << "Mrr0x238238 does not exist in the Cx75MessageManager!";
          return ErrorCode::CANBUS_ERROR;
        }

        mrr_0x239_239_ = dynamic_cast<Mrr0x239239 *>(message_manager_->GetMutableProtocolDataById(Mrr0x239239::ID));
        if (mrr_0x239_239_ == nullptr)
        {
          AERROR << "Mrr0x239239 does not exist in the Cx75MessageManager!";
          return ErrorCode::CANBUS_ERROR;
        }

        mrr_0x246_246_ = dynamic_cast<Mrr0x246246 *>(message_manager_->GetMutableProtocolDataById(Mrr0x246246::ID));
        if (mrr_0x246_246_ == nullptr)
        {
          AERROR << "Mrr0x246246 does not exist in the Cx75MessageManager!";
          return ErrorCode::CANBUS_ERROR;
        }

        mrr_frobj_0x279_279_ = dynamic_cast<Mrrfrobj0x279279 *>(message_manager_->GetMutableProtocolDataById(Mrrfrobj0x279279::ID));
        if (mrr_frobj_0x279_279_ == nullptr)
        {
          AERROR << "Mrrfrobj0x279279 does not exist in the Cx75MessageManager!";
          return ErrorCode::CANBUS_ERROR;
        }

        mrr_frobj_0x480_480_ = dynamic_cast<Mrrfrobj0x480480 *>(message_manager_->GetMutableProtocolDataById(Mrrfrobj0x480480::ID));
        if (mrr_frobj_0x480_480_ == nullptr)
        {
          AERROR << "Mrrfrobj0x480480 does not exist in the Cx75MessageManager!";
          return ErrorCode::CANBUS_ERROR;
        }

        ipm_0x245_245_ = dynamic_cast<Ipm0x245245 *>(message_manager_->GetMutableProtocolDataById(Ipm0x245245::ID));
        if (ipm_0x245_245_ == nullptr)
        {
          AERROR << "Ipm0x245245 does not exist in the Cx75MessageManager!";
          return ErrorCode::CANBUS_ERROR;
        }

        ipm_leftline_0x278_278_ = dynamic_cast<Ipmleftline0x278278 *>(message_manager_->GetMutableProtocolDataById(Ipmleftline0x278278::ID));
        if (ipm_leftline_0x278_278_ == nullptr)
        {
          AERROR << "Ipmleftline0x278278 does not exist in the Cx75MessageManager!";
          return ErrorCode::CANBUS_ERROR;
        }

        ipm_rightline_0x490_490_ = dynamic_cast<Ipmrightline0x490490 *>(message_manager_->GetMutableProtocolDataById(Ipmrightline0x490490::ID));
        if (ipm_rightline_0x490_490_ == nullptr)
        {
          AERROR << "Ipmrightline0x490490 does not exist in the Cx75MessageManager!";
          return ErrorCode::CANBUS_ERROR;
        }
	
        can_sender_->AddMessage(Pam0x270270::ID, pam_0x270_270_, false,6);
        //can_sender_->AddMessage(Pam0x271271::ID, pam_0x271_271_, false);
        //can_sender_->AddMessage(Pam0x272272::ID, pam_0x272_272_, false);
        // can_sender_->AddMessage(Mrr0x238238::ID, mrr_0x238_238_, false,6);
        // can_sender_->AddMessage(Mrr0x239239::ID, mrr_0x239_239_, false,6);
        //can_sender_->AddMessage(Mrr0x246246::ID, mrr_0x246_246_, false);
        //can_sender_->AddMessage(Mrrfrobj0x279279::ID, mrr_frobj_0x279_279_, false);
        //can_sender_->AddMessage(Mrrfrobj0x480480::ID, mrr_frobj_0x480_480_, false);
        // can_sender_->AddMessage(Ipm0x245245::ID, ipm_0x245_245_, false,6);
        //can_sender_->AddMessage(Ipmleftline0x278278::ID, ipm_leftline_0x278_278_, false);
        //can_sender_->AddMessage(Ipmrightline0x490490::ID, ipm_rightline_0x490_490_, false);

        // need sleep to ensure all messages received
        AINFO << "Cx75Controller is initialized.";

        is_initialized_ = true;
        return ErrorCode::OK;
      }

      Cx75Controller::~Cx75Controller() 
      {
         
      }

      bool Cx75Controller::Start()
      {
        if (!is_initialized_)
        {
          AERROR << "Cx75Controller has NOT been initiated.";
          return false;
        }
        const auto &update_func = [this] { SecurityDogThreadFunc(); };
        thread_.reset(new std::thread(update_func));

        return true;
      }

      void Cx75Controller::Stop()
      {
        if (!is_initialized_)
        {
          AERROR << "Cx75Controller stops or starts improperly!";
          return;
        }
        // DisableAutoMode();
        // DisableAPAMode();
        if (thread_ != nullptr && thread_->joinable())
        {
          thread_->join();
          thread_.reset();
          AINFO << "Cx75Controller stopped.";
        }
      }

      Chassis Cx75Controller::chassis()
      {
        chassis_.Clear();

        ChassisDetail chassis_detail;
        message_manager_->GetSensorData(&chassis_detail);

        // 21, 22, previously 1, 2
        if (driving_mode() == Chassis::EMERGENCY_MODE)
        {
          set_chassis_error_code(Chassis::NO_ERROR);
        }

        chassis_.set_driving_mode(driving_mode());
        chassis_.set_error_code(chassis_error_code());

        // 3
        chassis_.set_engine_started(true);
        /* ADD YOUR OWN CAR CHASSIS OPERATION*/			
		AINFO<<"EPS_LKAControlStatus:"<<chassis_detail.cx75().eps_advanced_0x176_176().eps_lkacontrolstatus();
		AINFO<<"ESP_VLC_Active:"<<chassis_detail.cx75().esp_vlc_0x223_223().esp_vlc_active();
		AINFO<<"ESP_VLC_Available:"<<chassis_detail.cx75().esp_vlc_0x223_223().esp_vlc_available();
		AINFO<<"Eps_epspamstsType:"<<chassis_detail.cx75().eps_advanced_0x176_176().eps_epspamsts();
    AINFO<<"ESP_PAM_LC_FailureSts:"<<chassis_detail.cx75().esp_vlc_0x223_223().esp_pam_lc_failurests();
        //ABS_SPEED
        if (chassis_detail.cx75().has_abs_sts_0x221_221() && chassis_detail.cx75().abs_sts_0x221_221().has_abs_vehspdlgt() &&
            chassis_detail.cx75().abs_sts_0x221_221().abs_vehspdlgtstatus() == Abs_sts_0x221_221::ABS_VEHSPDLGTSTATUS_VALID)
        {
          chassis_.set_speed_mps(
              static_cast<float>(chassis_detail.cx75().abs_sts_0x221_221().abs_vehspdlgt() / 3.6));
          chassis_.set_abs_vehspdlgt_valid(true);
        }
        else
        {
          chassis_.set_speed_mps(0);
          chassis_.set_abs_vehspdlgt_valid(false);
        }
        //ESP_Algt
        if (chassis_detail.cx75().has_esp_axay_0x242_242() && chassis_detail.cx75().esp_axay_0x242_242().has_esp_algt() &&
            chassis_detail.cx75().esp_axay_0x242_242().esp_algtstatus() == Esp_axay_0x242_242::ESP_ALGTSTATUS_OK)
        {
          chassis_.set_lon_acc(
              static_cast<float>(chassis_detail.cx75().esp_axay_0x242_242().esp_algt()));
          chassis_.set_esp_algtstatus_valid(true);
        }
        else
        {
          chassis_.set_lon_acc(0);
          chassis_.set_esp_algtstatus_valid(false);
        }
        //tcu_gearinfo P档有问题
        if (chassis_detail.cx75().has_gw_tcu_gearinfo_0x123_123() &&
            chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().has_tcu_displaygear())
        {
          Chassis::GearPosition gear_pos = Chassis::GEAR_INVALID;

          if (chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_displaygear() ==
              Gw_tcu_gearinfo_0x123_123::TCU_DISPLAYGEAR_CURRENT_GEAR_N)
          {
            gear_pos = Chassis::GEAR_NEUTRAL;
          }
          if (chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_displaygear() ==
              Gw_tcu_gearinfo_0x123_123::TCU_DISPLAYGEAR_CURRENT_GEAR_R)
          {
            gear_pos = Chassis::GEAR_REVERSE;
          }
          /* if (chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_currentgearposition() == Gw_tcu_gearinfo_0x123_123::TCU_CURRENTGEARPOSITION_1STGEAR ||
              chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_currentgearposition() == Gw_tcu_gearinfo_0x123_123::TCU_CURRENTGEARPOSITION_2NDGEAR ||
              chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_currentgearposition() == Gw_tcu_gearinfo_0x123_123::TCU_CURRENTGEARPOSITION_3RDGEAR ||
              chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_currentgearposition() == Gw_tcu_gearinfo_0x123_123::TCU_CURRENTGEARPOSITION_4THGEAR ||
              chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_currentgearposition() == Gw_tcu_gearinfo_0x123_123::TCU_CURRENTGEARPOSITION_5THGEAR ||
              chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_currentgearposition() == Gw_tcu_gearinfo_0x123_123::TCU_CURRENTGEARPOSITION_6THGEAR ||
              chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_currentgearposition() == Gw_tcu_gearinfo_0x123_123::TCU_CURRENTGEARPOSITION_7THGEAR ||
              chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_currentgearposition() == Gw_tcu_gearinfo_0x123_123::TCU_CURRENTGEARPOSITION_8THGEAR)
          {
            gear_pos = Chassis::GEAR_DRIVE;
          }*/
          if (chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_displaygear() == Gw_tcu_gearinfo_0x123_123::TCU_DISPLAYGEAR_CURRENT_GEAR_D)
          {
            gear_pos = Chassis::GEAR_DRIVE;
          }
          if (chassis_detail.cx75().gw_tcu_gearinfo_0x123_123().tcu_displaygear() == Gw_tcu_gearinfo_0x123_123::TCU_DISPLAYGEAR_CURRENT_GEAR_P)
          {
            gear_pos = Chassis::GEAR_PARKING;
          }

          chassis_.set_gear_location(gear_pos);
        }
        // else
        // {
        //   chassis_.set_gear_location(Chassis::GEAR_NONE);
        // }
        //SteerWheelAngle和SAS_Raw_SteerWheelAngle的区别？
        if (chassis_detail.cx75().has_sas_sensor_0x175_175() &&
            chassis_detail.cx75().sas_sensor_0x175_175().has_sas_steerwheelangle() &&
            chassis_detail.cx75().sas_sensor_0x175_175().has_sas_sasstssnsr() == Sas_sensor_0x175_175::SAS_SASSTSSNSR_SENSOR_VALUE_VALID)
        {
          chassis_.set_steering_percentage(static_cast<double>(
              chassis_detail.cx75().sas_sensor_0x175_175().sas_steerwheelangle()));
          chassis_.set_sas_sasstssnsr_valid(true);
        }
        // else
        // {
        //   chassis_.set_steering_percentage(0);
        //   chassis_.set_sas_sasstssnsr_valid(false);
        // }

        //steerwheelrotspd
        if (chassis_detail.cx75().has_sas_sensor_0x175_175() &&
            chassis_detail.cx75().sas_sensor_0x175_175().has_sas_steerwheelrotspd() &&
            chassis_detail.cx75().sas_sensor_0x175_175().sas_steerwheelrotspdstatus() == Sas_sensor_0x175_175::SAS_STEERWHEELROTSPDSTATUS_VALID)
        {
          chassis_.set_steer_wheel_rot_spd(static_cast<float>(
              chassis_detail.cx75().sas_sensor_0x175_175().sas_steerwheelrotspd()));
          chassis_.set_steerwheelrotspd_valid(true);
        }
        // else
        // {
        //   chassis_.set_steer_wheel_rot_spd(0);
        //   chassis_.set_steerwheelrotspd_valid(false);
        // }
        //steering_torque_nm
        if (chassis_detail.cx75().has_eps_advanced_0x176_176() &&
            chassis_detail.cx75().eps_advanced_0x176_176().has_eps_lkaresponsetorque() &&
            chassis_detail.cx75().eps_advanced_0x176_176().eps_lkaresponsetorquevalid() == Eps_advanced_0x176_176::EPS_LKARESPONSETORQUEVALID_VAILD)
        {
          chassis_.set_steering_torque_nm(static_cast<double>(
              chassis_detail.cx75().eps_advanced_0x176_176().eps_lkaresponsetorque()));
          chassis_.set_eps_lkaresponsetorque_valid(true);
        }
        // else
        // {
        //   chassis_.set_steering_torque_nm(0);
        //   chassis_.set_eps_lkaresponsetorque_valid(false);
        // }
        //eps_torsionbartorque
        if (chassis_detail.cx75().has_eps_advanced_0x176_176() &&
            chassis_detail.cx75().eps_advanced_0x176_176().has_eps_torsionbartorque() &&
            chassis_detail.cx75().eps_advanced_0x176_176().eps_tosionbartorquevalid() == Eps_advanced_0x176_176::EPS_TOSIONBARTORQUEVALID_VAILD)
        {
          chassis_.set_eps_torsionbartorque(static_cast<double>(
              chassis_detail.cx75().eps_advanced_0x176_176().eps_torsionbartorque()));
          chassis_.set_eps_tosionbartorquevalid(true);
        }
        // else
        // {
        //   chassis_.set_eps_torsionbartorque(0);
        //   chassis_.set_eps_tosionbartorquevalid(false);
        // }
        //右前轮累积上升沿和下降沿之和
        if (chassis_detail.cx75().has_abs_sts_0x221_221() &&
            chassis_detail.cx75().abs_sts_0x221_221().has_abs_whlmilgfrntri() &&
            chassis_detail.cx75().abs_sts_0x221_221().abs_whlmilgfrntristatus() == Abs_sts_0x221_221::ABS_WHLMILGFRNTRISTATUS_VALID)
        {
          chassis_.set_abs_whlmilgfrntri(static_cast<int>(
              chassis_detail.cx75().abs_sts_0x221_221().abs_whlmilgfrntri()));
          //chassis_.set_eps_tosionbartorquevalid(true);
        }
        // else
        // {
        //   chassis_.set_abs_whlmilgfrntri(0);
        //   //chassis_.set_eps_tosionbartorquevalid(false);
        // }
        //左前轮累积上升沿和下降沿之和
        if (chassis_detail.cx75().has_abs_sts_0x221_221() &&
            chassis_detail.cx75().abs_sts_0x221_221().has_abs_whlmilgfrntle() &&
            chassis_detail.cx75().abs_sts_0x221_221().abs_whlmilgfrntlestatus() == Abs_sts_0x221_221::ABS_WHLMILGFRNTLESTATUS_VALID)
        {
          chassis_.set_abs_whlmilgfrntle(static_cast<int>(
              chassis_detail.cx75().abs_sts_0x221_221().abs_whlmilgfrntle()));
          //chassis_.set_eps_tosionbartorquevalid(true);
        }
        // else
        // {
        //   chassis_.set_abs_whlmilgfrntle(0);
        //   //chassis_.set_eps_tosionbartorquevalid(false);
        // }
        //右后轮累积上升沿和下降沿之和
        if (chassis_detail.cx75().has_esp_direction_0x235_235() &&
            chassis_detail.cx75().esp_direction_0x235_235().has_esp_whlmilgrearre() &&
            chassis_detail.cx75().esp_direction_0x235_235().esp_whlmilgrearristatus() == Esp_direction_0x235_235::ESP_WHLMILGREARRISTATUS_VALID)
        {
          chassis_.set_abs_whlmilgrearre(static_cast<int>(
              chassis_detail.cx75().esp_direction_0x235_235().esp_whlmilgrearre()));
          //chassis_.set_eps_tosionbartorquevalid(true);
        }
        // else
        // {
        //   chassis_.set_abs_whlmilgrearre(0);
        //   //chassis_.set_eps_tosionbartorquevalid(false);
        // }
        //左后轮累积上升沿和下降沿之和
        if (chassis_detail.cx75().has_esp_direction_0x235_235() &&
            chassis_detail.cx75().esp_direction_0x235_235().has_esp_whlmilgrearle() &&
            chassis_detail.cx75().esp_direction_0x235_235().esp_whlmilgrearlestatus() == Esp_direction_0x235_235::ESP_WHLMILGREARLESTATUS_VALID)
        {
          chassis_.set_abs_whlmilgfrntle(static_cast<int>(
              chassis_detail.cx75().esp_direction_0x235_235().esp_whlmilgrearle()));
          //chassis_.set_eps_tosionbartorquevalid(true);
        }
        // else
        // {
        //   chassis_.set_abs_whlmilgfrntle(0);
        //   //chassis_.set_eps_tosionbartorquevalid(false);
        // }
        //轮速度脉冲以20ms为单位
        if (chassis_detail.cx75().has_esp_whlpulse_0x236_236())
        {
          if (chassis_detail.cx75().esp_whlpulse_0x236_236().has_esp_wheelpulse_fl() &&
              chassis_detail.cx75().esp_whlpulse_0x236_236().esp_wheelpulse_fl_valid() == Esp_whlpulse_0x236_236::ESP_WHEELPULSE_FL_VALID_VALID)
          {
            chassis_.set_esp_wheelpulse_fl(static_cast<int>(
                chassis_detail.cx75().esp_whlpulse_0x236_236().esp_wheelpulse_fl()));
            //chassis_.set_eps_tosionbartorquevalid(true);
          }
          if (chassis_detail.cx75().esp_whlpulse_0x236_236().has_esp_wheelpulse_fr() &&
              chassis_detail.cx75().esp_whlpulse_0x236_236().esp_wheelpulse_fr_valid() == Esp_whlpulse_0x236_236::ESP_WHEELPULSE_FR_VALID_VALID)
          {
            chassis_.set_esp_wheelpulse_fr(static_cast<int>(
                chassis_detail.cx75().esp_whlpulse_0x236_236().esp_wheelpulse_fr()));
            //chassis_.set_eps_tosionbartorquevalid(true);
          }
          if (chassis_detail.cx75().esp_whlpulse_0x236_236().has_esp_wheelpulse_rl() &&
              chassis_detail.cx75().esp_whlpulse_0x236_236().esp_wheelpulse_rl_valid() == Esp_whlpulse_0x236_236::ESP_WHEELPULSE_RL_VALID_VALID)
          {
            chassis_.set_esp_wheelpulse_rl(static_cast<int>(
                chassis_detail.cx75().esp_whlpulse_0x236_236().esp_wheelpulse_rl()));
            //chassis_.set_eps_tosionbartorquevalid(true);
          }
          if (chassis_detail.cx75().esp_whlpulse_0x236_236().has_esp_wheelpulse_rr() &&
              chassis_detail.cx75().esp_whlpulse_0x236_236().esp_wheelpulse_rr_valid() == Esp_whlpulse_0x236_236::ESP_WHEELPULSE_RR_VALID_VALID)
          {
            chassis_.set_esp_wheelpulse_rr(static_cast<int>(
                chassis_detail.cx75().esp_whlpulse_0x236_236().esp_wheelpulse_rr()));
            //chassis_.set_eps_tosionbartorquevalid(true);
          }
        }

        if (chassis_detail.cx75().has_gw_ems_whltq_0x107_107())
        {
          if (chassis_detail.cx75().gw_ems_whltq_0x107_107().has_ems_accpedalratio() &&
              chassis_detail.cx75().gw_ems_whltq_0x107_107().ems_accpedalratioerror() == Gw_ems_whltq_0x107_107::EMS_ACCPEDALRATIOERROR_NOERROR)
          {
            chassis_.set_ems_accpedalratio(static_cast<double>(
                chassis_detail.cx75().gw_ems_whltq_0x107_107().ems_accpedalratio()));
            //chassis_.set_eps_tosionbartorquevalid(true);
          }
          if (chassis_detail.cx75().gw_ems_whltq_0x107_107().has_ems_brkpedalstasus())
          {
            if (chassis_detail.cx75().gw_ems_whltq_0x107_107().ems_brkpedalstasus() == Gw_ems_whltq_0x107_107::EMS_BRKPEDALSTASUS_NOTPRESSED)
            {
              chassis_.set_brkpedalstasus(Chassis::NOT_PRESSED);
            }
            else if (chassis_detail.cx75().gw_ems_whltq_0x107_107().ems_brkpedalstasus() == Gw_ems_whltq_0x107_107::EMS_BRKPEDALSTASUS_PRESSED)
            {
              chassis_.set_brkpedalstasus(Chassis::PRESSED);
            }
            else if (chassis_detail.cx75().gw_ems_whltq_0x107_107().ems_brkpedalstasus() == Gw_ems_whltq_0x107_107::EMS_BRKPEDALSTASUS_ERROR)
            {
              chassis_.set_brkpedalstasus(Chassis::ERROR);
            }
          }
        }
        if (chassis_detail.cx75().has_abs_whlspd_0x211_211())
        {
          if (chassis_detail.cx75().abs_whlspd_0x211_211().has_abs_whlspdfrntle() &&
              chassis_detail.cx75().abs_whlspd_0x211_211().abs_whlspdfrntlestatus() == Abs_whlspd_0x211_211::ABS_WHLSPDFRNTLESTATUS_VALID)
          {
            chassis_.set_abs_whlspdfrntle(static_cast<double>(
                chassis_detail.cx75().abs_whlspd_0x211_211().abs_whlspdfrntle()));
            //chassis_.set_eps_tosionbartorquevalid(true);
          }
          if (chassis_detail.cx75().abs_whlspd_0x211_211().has_abs_whlspdfrntri() &&
              chassis_detail.cx75().abs_whlspd_0x211_211().abs_whlspdfrntristatus() == Abs_whlspd_0x211_211::ABS_WHLSPDFRNTRISTATUS_VALID)
          {
            chassis_.set_abs_whlspdfrntri(static_cast<double>(
                chassis_detail.cx75().abs_whlspd_0x211_211().abs_whlspdfrntri()));
            //chassis_.set_eps_tosionbartorquevalid(true);
          }
          if (chassis_detail.cx75().abs_whlspd_0x211_211().has_abs_whlspdrele() &&
              chassis_detail.cx75().abs_whlspd_0x211_211().abs_whlspdrelestatus() == Abs_whlspd_0x211_211::ABS_WHLSPDRELESTATUS_VALID)
          {
            chassis_.set_abs_whlspdrele(static_cast<double>(
                chassis_detail.cx75().abs_whlspd_0x211_211().abs_whlspdrele()));
            //chassis_.set_eps_tosionbartorquevalid(true);
          }
          if (chassis_detail.cx75().abs_whlspd_0x211_211().has_abs_whlspdreri() &&
              chassis_detail.cx75().abs_whlspd_0x211_211().abs_whlspdreristatus() == Abs_whlspd_0x211_211::ABS_WHLSPDRERISTATUS_VALID)
          {
            chassis_.set_abs_whlspdreri(static_cast<double>(
                chassis_detail.cx75().abs_whlspd_0x211_211().abs_whlspdreri()));
            //chassis_.set_eps_tosionbartorquevalid(true);
          }
        }
        if (chassis_detail.cx75().has_ins_acc_500())
        {
          if (chassis_detail.cx75().ins_acc_500().has_acc_x() && chassis_detail.cx75().ins_acc_500().has_acc_y() && chassis_detail.cx75().ins_acc_500().has_acc_z())
          {
            chassis_.set_acc_x(static_cast<double>(
                chassis_detail.cx75().ins_acc_500().acc_x()));
            chassis_.set_acc_y(static_cast<double>(
                chassis_detail.cx75().ins_acc_500().acc_y()));
            chassis_.set_acc_z(static_cast<double>(
                chassis_detail.cx75().ins_acc_500().acc_z()));
          }
        }
        if (chassis_detail.cx75().has_ins_gyro_501())
        {
          if (chassis_detail.cx75().ins_gyro_501().has_gyro_x() && chassis_detail.cx75().ins_gyro_501().has_gyro_y() && chassis_detail.cx75().ins_gyro_501().has_gyro_z())
          {
            chassis_.set_gyro_x(static_cast<double>(
                chassis_detail.cx75().ins_gyro_501().gyro_x()));
            chassis_.set_gyro_y(static_cast<double>(
                chassis_detail.cx75().ins_gyro_501().gyro_y()));
            chassis_.set_gyro_z(static_cast<double>(
                chassis_detail.cx75().ins_gyro_501().gyro_z()));
          }
        }
        if (chassis_detail.cx75().has_ins_headingpitchroll_502())
        {
          if (chassis_detail.cx75().ins_headingpitchroll_502().has_ins_pitchangle() && chassis_detail.cx75().ins_headingpitchroll_502().has_ins_rollangle() && chassis_detail.cx75().ins_headingpitchroll_502().has_ins_headingangle())
          {
            chassis_.set_ins_pitchangle(static_cast<double>(
                chassis_detail.cx75().ins_headingpitchroll_502().ins_pitchangle()));
            chassis_.set_ins_rollangle(static_cast<double>(
                chassis_detail.cx75().ins_headingpitchroll_502().ins_rollangle()));
            chassis_.set_ins_headingangle(static_cast<double>(
                chassis_detail.cx75().ins_headingpitchroll_502().ins_headingangle()));
          }
        }
        if (chassis_detail.cx75().has_ins_heightandtime_503())
        {
          if (chassis_detail.cx75().ins_heightandtime_503().has_ins_locatheight() && chassis_detail.cx75().ins_heightandtime_503().has_ins_time())
          {
            chassis_.set_ins_locatheight(static_cast<double>(
                chassis_detail.cx75().ins_heightandtime_503().ins_locatheight()));
            chassis_.set_ins_time(static_cast<double>(
                chassis_detail.cx75().ins_heightandtime_503().ins_time()));
          }
        }
        if (chassis_detail.cx75().has_ins_latitudelongitude_504())
        {
          if (chassis_detail.cx75().ins_latitudelongitude_504().has_ins__latitude() && chassis_detail.cx75().ins_latitudelongitude_504().has_ins_longitude())
          {
            chassis_.set_ins_latitude(static_cast<double>(
                chassis_detail.cx75().ins_latitudelongitude_504().ins__latitude()));
            chassis_.set_ins_longitude(static_cast<double>(
                chassis_detail.cx75().ins_latitudelongitude_504().ins_longitude()));
          }
        }
        if (chassis_detail.cx75().has_ins_speed_505())
        {
          if (chassis_detail.cx75().ins_speed_505().has_ins_northspd() && chassis_detail.cx75().ins_speed_505().has_ins_eastspd() && chassis_detail.cx75().ins_speed_505().has_ins_togroundspd())
          {
            chassis_.set_ins_northspd(static_cast<double>(
                chassis_detail.cx75().ins_speed_505().ins_northspd()));
            chassis_.set_ins_eastspd(static_cast<double>(
                chassis_detail.cx75().ins_speed_505().ins_eastspd()));
            chassis_.set_ins_togroundspd(static_cast<double>(
                chassis_detail.cx75().ins_speed_505().ins_togroundspd()));
          }
        }
        if (chassis_detail.cx75().has_ins_datainfo_506())
        {
          if (chassis_detail.cx75().ins_datainfo_506().has_ins_gpsflag_pos() && chassis_detail.cx75().ins_datainfo_506().has_ins_numsv() && chassis_detail.cx75().ins_datainfo_506().has_ins_gpsflag_heading() &&
              chassis_detail.cx75().ins_datainfo_506().has_ins_gps_age() && chassis_detail.cx75().ins_datainfo_506().has_ins_car_status() && chassis_detail.cx75().ins_datainfo_506().has_ins_status())
          {
            chassis_.set_ins_gpsflag_pos(static_cast<int>(
                chassis_detail.cx75().ins_datainfo_506().ins_gpsflag_pos()));
            chassis_.set_ins_numsv(static_cast<double>(
                chassis_detail.cx75().ins_datainfo_506().ins_numsv()));
            chassis_.set_ins_gpsflag_heading(static_cast<int>(
                chassis_detail.cx75().ins_datainfo_506().ins_gpsflag_heading()));
            chassis_.set_ins_gps_age(static_cast<double>(
                chassis_detail.cx75().ins_datainfo_506().ins_gps_age()));
            chassis_.set_ins_car_status(
                chassis_detail.cx75().ins_datainfo_506().ins_car_status());
            chassis_.set_ins_status(
                chassis_detail.cx75().ins_datainfo_506().ins_status());
          }
        }
        if (chassis_detail.cx75().has_ins_std_507())
        {
          if (chassis_detail.cx75().ins_std_507().has_ins_std_lat() && chassis_detail.cx75().ins_std_507().has_ins_std_lon() &&
              chassis_detail.cx75().ins_std_507().has_ins_std_locatheight() && chassis_detail.cx75().ins_std_507().has_inins_std_heading())
          {
            chassis_.set_ins_std_lat(static_cast<double>(
                chassis_detail.cx75().ins_std_507().ins_std_lat()));
            chassis_.set_ins_std_lon(static_cast<double>(
                chassis_detail.cx75().ins_std_507().ins_std_lon()));
            chassis_.set_ins_std_locat_height(static_cast<double>(
                chassis_detail.cx75().ins_std_507().ins_std_locatheight()));
            chassis_.set_ins_std_heading(static_cast<double>(
                chassis_detail.cx75().ins_std_507().inins_std_heading()));
          }
        }
      if (chassis_detail.cx75().has_esp_vlc_0x223_223())
      {
        chassis_.set_esp_vlc_active(static_cast<int>(
                chassis_detail.cx75().esp_vlc_0x223_223().esp_vlc_active()));
        chassis_.set_esp_vlc_available(static_cast<int>(
                chassis_detail.cx75().esp_vlc_0x223_223().esp_vlc_available())); 
        chassis_.set_esp_apa_gearboxenable(static_cast<int>(
                chassis_detail.cx75().esp_vlc_0x223_223().esp_apa_gearboxenable()));
        chassis_.set_esp_vlc_apactive(static_cast<int>(
                chassis_detail.cx75().esp_vlc_0x223_223().esp_vlc_apactive()));  
        chassis_.set_esp_vlcapa_available(static_cast<int>(
                chassis_detail.cx75().esp_vlc_0x223_223().esp_vlcapa_available()));   
      }
	   if (chassis_detail.cx75().has_eps_advanced_0x176_176())
      {
        chassis_.set_eps_epspamsts(static_cast<int>(
                chassis_detail.cx75().eps_advanced_0x176_176().eps_epspamsts()));
      }
      if (chassis_detail.cx75().has_eps_advanced_0x176_176())
      {
        chassis_.set_eps_lkacontrolstatus(static_cast<int>(
                chassis_detail.cx75().eps_advanced_0x176_176().eps_lkacontrolstatus())); 
        chassis_.set_eps_epspaminh(static_cast<int>(
                chassis_detail.cx75().eps_advanced_0x176_176().eps_epspaminh()));
      }
      if (chassis_detail.cx75().has_esp_status_0x243_243())
      {
        chassis_.set_esp_epbstatus(static_cast<int>(
                chassis_detail.cx75().esp_status_0x243_243().esp_epbstatus()));
      }



    
        return chassis_;
      }

      void Cx75Controller::Emergency()
      {
        // set_driving_mode(Chassis::EMERGENCY_MODE);
        set_driving_mode(Chassis::COMPLETE_MANUAL);
        ResetProtocol();
      }

      ErrorCode Cx75Controller::EnableAutoMode()
      {
        AINFO << "START EnableAutoMode";
        if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE)
        {
          AINFO << "already in COMPLETE_AUTO_DRIVE mode";
          return ErrorCode::OK;
        }

        // if (chassis_.esp_vlc_available()==0)
        // {
        //   AERROR<<"esp_vlc is not available,check esp ErrorCode";
        //   return ErrorCode::CANBUS_ERROR;
        // }
        // if (chassis_.eps_lkacontrolstatus()==3||chassis_.eps_lkacontrolstatus()==2)
        // {
        //   AERROR<<"eps_lka is not available,check eps ErrorCode";
        //   return ErrorCode::CANBUS_ERROR;
        // }

        if (chassis_.esp_vlcapa_available()==0)
        {
          AERROR<<"esp_vlcapa is not available,check esp ErrorCode";
          return ErrorCode::CANBUS_ERROR;
        }
		    if (chassis_.eps_epspamsts()==0||chassis_.eps_epspamsts()==3||chassis_.eps_epspamsts()==7)
        {
         AERROR<<"eps_epspamsts is not available,check esp ErrorCode";
         return ErrorCode::CANBUS_ERROR;
        }
        // mrr_0x239_239_->set_acc_state(Mrr_0x239_239::ACC_STATE_ACTIVE_CONTROL_MODE);
        // ipm_0x245_245_->set_ipm_laneassit_torquereqstatus(Ipm_0x245_245::IPM_LANEASSIT_TORQUEREQSTATUS_TORQUE_REQUEST);
        pam_0x270_270_->set_pam_brakemodests(Pam_0x270_270::PAM_BRAKEMODESTS_ACTIVE);
        pam_0x270_270_->set_pam_brakefunctionmode(Pam_0x270_270::PAM_BRAKEFUNCTIONMODE_AUTOMATICPARK);//需不需要
        //pam_0x270_270_->set_pam_cmdepssts(Pam_0x270_270::PAM_CMDEPSSTS_EPS_CONTROL);//需不需要
		    pam_0x270_270_->set_pam_cmdepssts(Pam_0x270_270::PAM_CMDEPSSTS_CONTROL_EPS_REQUEST);
        pam_0x270_270_->set_stopstartinhibit_apa(Pam_0x270_270::STOPSTARTINHIBIT_APA_TRUE);


       // can_sender_->send_EventMessage(debug_extended_mode_736_->ID);

        // mrr_0x239_239_->set_acc_driveoff(Mrr_0x239_239::ACC_DRIVEOFF_DEMAND);
        can_sender_->Update();
        // const int32_t flag =
        //     CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
        const int32_t flag =
            CHECK_RESPONSE_APA_UNIT_FLAG | CHECK_RESPONSE_ESPPAM_UNIT_FLAG;
        /*if (!CheckResponse(flag, true))
        {
          AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
          Emergency();
          set_chassis_error_code(Chassis::CHASSIS_ERROR);
          return ErrorCode::CANBUS_ERROR;
        }
        else
        {
          set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
          AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
          return ErrorCode::OK;
        }*/
        set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
        AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
        return ErrorCode::OK;
      }
      ErrorCode Cx75Controller::DisableAutoMode()
      {
        set_driving_mode(Chassis::COMPLETE_MANUAL);
        ResetProtocol();
        // can_sender_->Update();
        
        set_chassis_error_code(Chassis::NO_ERROR);
        AINFO << "Switch to COMPLETE_MANUAL OK!";
        return ErrorCode::OK;
      }

      ErrorCode Cx75Controller::EnableRemoteMode()
      {
        if (driving_mode() == Chassis::REMOTE_MODE)
        {
          AINFO << "already in REMOTE_MODE mode";
          return ErrorCode::OK;
        }
      }

      ErrorCode Cx75Controller::EnableSteeringOnlyMode()
      {
        if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
            driving_mode() == Chassis::AUTO_STEER_ONLY)
        {
          set_driving_mode(Chassis::AUTO_STEER_ONLY);
          AINFO << "Already in AUTO_STEER_ONLY mode";
          return ErrorCode::OK;
        }
        return ErrorCode::OK;
        /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_disable();
  throttle_62_->set_disable();
  steering_64_->set_enable();

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
    return ErrorCode::OK;
  }
  */
      }

      ErrorCode Cx75Controller::EnableSpeedOnlyMode()
      {
        if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
            driving_mode() == Chassis::AUTO_SPEED_ONLY)
        {
          set_driving_mode(Chassis::AUTO_SPEED_ONLY);
          AINFO << "Already in AUTO_SPEED_ONLY mode";
          return ErrorCode::OK;
        }
        return ErrorCode::OK;
        /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_enable();
  throttle_62_->set_enable();
  steering_64_->set_disable();

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
    return ErrorCode::OK;
  }
  */
      }

      ErrorCode Cx75Controller::EnableAPAMode()
      {
        if (driving_mode() == Chassis::APA_MODE)
        {
          set_driving_mode(Chassis::APA_MODE);
          AINFO << "Already in AUTO_SPEED_ONLY mode";
          return ErrorCode::OK;
        }

        pam_0x270_270_->set_pam_brakemodests(Pam_0x270_270::PAM_BRAKEMODESTS_ACTIVE);
        pam_0x270_270_->set_pam_brakefunctionmode(Pam_0x270_270::PAM_BRAKEFUNCTIONMODE_AUTOMATICPARK);

        //can_sender_->Update();
        const int32_t flag =
            CHECK_RESPONSE_APA_UNIT_FLAG ;
        if (CheckResponse(flag, true) == false)
        {
          AERROR << "Failed to switch to APAMode mode.";
          Emergency();
          set_chassis_error_code(Chassis::CHASSIS_ERROR);
          return ErrorCode::CANBUS_ERROR;
        }
        else
        {
          set_driving_mode(Chassis::APA_MODE);
          AINFO << "Switch to APA_MODE mode ok.";
          return ErrorCode::OK;
        }
      }

      ErrorCode Cx75Controller::DisableAPAMode()
      {
        set_driving_mode(Chassis::COMPLETE_MANUAL);
        pam_0x270_270_->set_pam_brakemodests(Pam_0x270_270::PAM_BRAKEMODESTS_MANEUVERFINISHED);
        pam_0x270_270_->set_pam_brakefunctionmode(Pam_0x270_270::PAM_BRAKEFUNCTIONMODE_AUTOMATICPARK);
        pam_0x270_270_->set_pam_esp_target_gear_request(Pam_0x270_270::PAM_ESP_TARGET_GEAR_REQUEST_PARK);
        //can_sender_->Update();
        
        set_chassis_error_code(Chassis::NO_ERROR);
        AINFO << "Switch to COMPLETE_MANUAL OK!";
        return ErrorCode::OK;
      }

      void Cx75Controller::Acceleration(double acc)
      {
        if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
              driving_mode() == Chassis::AUTO_SPEED_ONLY))
        {
          AINFO << "this drive mode no need to set Acceleration.";
          return;
        }
        mrr_0x238_238_->set_acc_tgtax(acc);
        mrr_0x238_238_->set_acc_tgtaxlowercomftband(0.2);
        mrr_0x238_238_->set_acc_tgtaxlowerlim(-4);
        mrr_0x238_238_->set_acc_tgtaxuppercomftband(0.1);
        mrr_0x238_238_->set_acc_tgtaxupperlim(0.3);
        AINFO<<"acc:"<<acc<<"   chassis_.esp_epbstatus:"<<chassis_.esp_epbstatus();
        if (acc>0&&(chassis_.esp_epbstatus()==1||chassis_.esp_epbstatus()==3))
        {
          mrr_0x239_239_->set_acc_driveoff(Mrr_0x239_239::ACC_DRIVEOFF_DEMAND);
        }
        else 
        {
          mrr_0x239_239_->set_acc_driveoff(Mrr_0x239_239::ACC_DRIVEOFF_NO_DEMAND);
        }

      }

      // NEUTRAL, REVERSE, DRIVE
      void Cx75Controller::Gear(Chassis::GearPosition gear_position)
      {
        if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
              driving_mode() == Chassis::AUTO_SPEED_ONLY))
        {
          AINFO << "this drive mode no need to set gear.";
          return;
        }
	    AERROR<<"gear_position"<<gear_position;
        switch (gear_position)
        {
        case Chassis::GEAR_NEUTRAL:
        {
          pam_0x270_270_->set_pam_esp_target_gear_request(Pam_0x270_270::PAM_ESP_TARGET_GEAR_REQUEST_NEUTRAL);
          break;
        }
        case Chassis::GEAR_REVERSE:
        {
          pam_0x270_270_->set_pam_esp_target_gear_request(Pam_0x270_270::PAM_ESP_TARGET_GEAR_REQUEST_REVERSE);
          break;
        }
        case Chassis::GEAR_DRIVE:
        {
          pam_0x270_270_->set_pam_esp_target_gear_request(Pam_0x270_270::PAM_ESP_TARGET_GEAR_REQUEST_DRIVE);
          break;
        }
        case Chassis::GEAR_PARKING:
        {
          pam_0x270_270_->set_pam_esp_target_gear_request(Pam_0x270_270::PAM_ESP_TARGET_GEAR_REQUEST_PARK);
          break;
        }
        default:
        {
          pam_0x270_270_->set_pam_esp_target_gear_request(Pam_0x270_270::PAM_ESP_TARGET_GEAR_REQUEST_PARK);
          break;
        }
        }
      }

      // brake with new acceleration
      // acceleration:0.00~99.99, unit:
      // acceleration:0.0 ~ 7.0, unit:m/s^2
      // acceleration_spd:60 ~ 100, suggest: 90
      // -> pedal
      void Cx75Controller::Brake(double pedal)
      {
      }

      // drive with old acceleration
      // gas:0.00~99.99 unit:
      void Cx75Controller::Throttle(double pedal)
      {
      }

      // cx75 default, -470 ~ 470, left:+, right:-
      // need to be compatible with control module, so reverse
      // steering with old angle speed
      // angle:-99.99~0.00~99.99, unit:, left:-, right:+
      void Cx75Controller::SteerTorque(double torque)
      {
        if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
              driving_mode() == Chassis::AUTO_STEER_ONLY))
        {
          AINFO << "The current driving mode does not need to set steer.";
          return;
        }
        AINFO << "SteerTorque��" << torque;
        ipm_0x245_245_->set_ipm_laneassit_torquereq(torque);
        ipm_0x245_245_->set_ipm_laneassit_torquevalidity(Ipm_0x245_245::IPM_LANEASSIT_TORQUEVALIDITY_VALID);
      }

      void Cx75Controller::Steer(double angle)
      {
        /*if (!driving_mode() == Chassis::APA_MODE)
        {
          AINFO << "The current driving mode does not need to set steer angle.";
          return;
        }
        pam_0x270_270_->set_pam_trgtepsstrgwhlang(angle);*/
        if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
              driving_mode() == Chassis::AUTO_STEER_ONLY))
        {
          AINFO << "The current driving mode does not need to set steer.";
          return;
        }
         AERROR << "Steerangle" << angle;
        // ipm_0x245_245_->set_ipm_laneassit_torquereq(angle);
        // ipm_0x245_245_->set_ipm_laneassit_torquevalidity(Ipm_0x245_245::IPM_LANEASSIT_TORQUEVALIDITY_VALID);
		// if (chassis_.eps_epspamsts()==1)
    //     {
		//   AERROR<<"1";
    //       pam_0x270_270_->set_pam_cmdepssts(Pam_0x270_270::PAM_CMDEPSSTS_CONTROL_EPS_REQUEST);
    //     }
		// else if (chassis_.eps_epspamsts()==4||chassis_.eps_epspamsts()==2)
    //     {
		// 	AERROR<<"2and4";
    //       pam_0x270_270_->set_pam_cmdepssts(Pam_0x270_270::PAM_CMDEPSSTS_EPS_CONTROL);
    //     }
		// else 
    //     {
		// 	AERROR<<"eps erro";
    //       pam_0x270_270_->set_pam_cmdepssts(Pam_0x270_270::PAM_CMDEPSSTS_NO_REQUEST);
    //     }
        pam_0x270_270_->set_pam_trgtepsstrgwhlang(angle);
      }

      void Cx75Controller::PamStopDistance(int distance)
      {
        if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
              driving_mode() == Chassis::AUTO_SPEED_ONLY))
        {
          AINFO << "this drive mode no need to set PamStopDistance.";
          return;
        }
        pam_0x270_270_->set_pam_esp_stop_distance(distance);
		AERROR<<"distance "<<distance;
      }

      void Cx75Controller::SpeedTarget(float speed)
      {
        if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
              driving_mode() == Chassis::AUTO_SPEED_ONLY))
        {
          AINFO << "this drive mode no need to set SpeedTarget.";
          return;
        }
		AERROR<<"speed "<<speed;
        pam_0x270_270_->set_pam_esp_speed_target(speed);
      }
      // steering with new angle speed
      // angle:-99.99~0.00~99.99, unit:, left:-, right:+
      // angle_spd:0.00~99.99, unit:deg/s
      void Cx75Controller::Steer(double angle, double angle_spd)
      {
        if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
              driving_mode() == Chassis::AUTO_STEER_ONLY))
        {
          AINFO << "The current driving mode does not need to set steer.";
          return;
        }
        /* ADD YOUR OWN CAR CHASSIS OPERATION
  const double real_angle = params_.max_steer_angle() * angle / 100.0;
  const double real_angle_spd = ProtocolData::BoundedValue(
      params_.min_steer_angle_spd(), params_.max_steer_angle_spd(),
      params_.max_steer_angle_spd() * angle_spd / 100.0);
  steering_64_->set_steering_angle(real_angle)
      ->set_steering_angle_speed(real_angle_spd);
  */
      }

      void Cx75Controller::SetEpbBreak(const ControlCommand &command)
      {
        if (command.parking_brake())
        {
          // None
        }
        else
        {
          // None
        }
      }

      void Cx75Controller::SetBeam(const ControlCommand &command)
      {
        if (command.signal().high_beam())
        {
          // None
        }
        else if (command.signal().low_beam())
        {
          // None
        }
        else
        {
          // None
        }
      }

      void Cx75Controller::SetHorn(const ControlCommand &command)
      {
        if (command.signal().horn())
        {
          // None
        }
        else
        {
          // None
        }
      }

      void Cx75Controller::SetTurningSignal(const ControlCommand &command)
      {
        // Set Turn Signal
        /* ADD YOUR OWN CAR CHASSIS OPERATION
  auto signal = command.signal().turn_signal();
  if (signal == Signal::TURN_LEFT) {
    turnsignal_68_->set_turn_left();
  } else if (signal == Signal::TURN_RIGHT) {
    turnsignal_68_->set_turn_right();
  } else {
    turnsignal_68_->set_turn_none();
  }
  */
      }

      void Cx75Controller::ResetProtocol()
      {
        message_manager_->ResetSendMessages();
		can_sender_->Update();
      }

      bool Cx75Controller::CheckChassisError()
      {
        /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
        return false;
      }

      void Cx75Controller::SecurityDogThreadFunc()
      {
        int32_t vertical_ctrl_fail = 0;
        int32_t horizontal_ctrl_fail = 0;

        if (can_sender_ == nullptr)
        {
          AERROR << "Fail to run SecurityDogThreadFunc() because can_sender_ is "
                    "nullptr.";
          return;
        }
        while (!can_sender_->IsRunning())
        {
          std::this_thread::yield();
        }

        std::chrono::duration<double, std::micro> default_period{50000};
        int64_t start = 0;
        int64_t end = 0;
        while (can_sender_->IsRunning())
        {
          

          start = ::jmc_auto::common::time::AsInt64<::jmc_auto::common::time::micros>(
              ::jmc_auto::common::time::Clock::Now());
          const Chassis::DrivingMode mode = driving_mode();
          //循环验证转向状态，切换转向状态，防止EPS一直报错
          if (mode == Chassis::COMPLETE_AUTO_DRIVE ||
               mode == Chassis::AUTO_STEER_ONLY) 
               {
                 AERROR<<"EPS错误码"<<chassis_.eps_epspaminh();
                 AERROR<<"EPS状态"<<chassis_.eps_epspamsts();
                 if (chassis_.eps_epspamsts() == 1)
                 {
                   AERROR << "eps ready";
                   pam_0x270_270_->set_pam_cmdepssts(Pam_0x270_270::PAM_CMDEPSSTS_CONTROL_EPS_REQUEST);
                 }
                 else if (chassis_.eps_epspamsts() == 4)
                 {
                   AERROR << "eps ok";
                   pam_0x270_270_->set_pam_cmdepssts(Pam_0x270_270::PAM_CMDEPSSTS_EPS_CONTROL);
                 }
                 else if (chassis_.eps_epspamsts() == 2)
                 {
                   AERROR << "eps very good";
                 }
                 else if (chassis_.eps_epspamsts() == 3)                 
                 {
                   AERROR << "eps erro";
                  //pam_0x270_270_->set_pam_cmdepssts(Pam_0x270_270::PAM_CMDEPSSTS_NO_REQUEST);
				           AERROR <<"chassis_.steering_percentage:"<<chassis_.steering_percentage();
                 }



     
                

               }


         
          bool emergency_mode = false;

          // 1. horizontal control check
          if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
               mode == Chassis::AUTO_STEER_ONLY) &&
              // CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false)
              CheckResponse(CHECK_RESPONSE_ESPPAM_UNIT_FLAG, false) == false)
          {
            ++horizontal_ctrl_fail;
			AINFO<<"horizontal_ctrl_fail:"<<horizontal_ctrl_fail;
            if (horizontal_ctrl_fail >= kMaxFailAttempt)
            {
              emergency_mode = true;
			  AERROR<<"steer ctrl_fail";
              set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
            }
          }
          else
          {
            horizontal_ctrl_fail = 0;
          }

          // 2. vertical control check
          if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
               mode == Chassis::AUTO_SPEED_ONLY) &&
              // CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false) == false)
              CheckResponse(CHECK_RESPONSE_APA_UNIT_FLAG, false) == false)
          {
            ++vertical_ctrl_fail;
			AINFO<<"vertical_ctrl_fail:"<<vertical_ctrl_fail;
            if (vertical_ctrl_fail >= kMaxFailAttempt)
            {
              emergency_mode = true;
			  AERROR<<"acc ctrl_fail";
              set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
            }
          }
          else
          {
            vertical_ctrl_fail = 0;
          }
          if (CheckChassisError())
          {
            set_chassis_error_code(Chassis::CHASSIS_ERROR);
            emergency_mode = true;
          }

          if (emergency_mode && mode != Chassis::COMPLETE_MANUAL)
          {
            // set_driving_mode(Chassis::EMERGENCY_MODE);
            AERROR<<"COMPLETE_MANUAL,ResetSendMessages";
            set_driving_mode(Chassis::COMPLETE_MANUAL);
            ResetProtocol();
            //message_manager_->ResetSendMessages();
            
            //message_manager_->ResetSendMessages();
          }
          end = ::jmc_auto::common::time::AsInt64<::jmc_auto::common::time::micros>(
              ::jmc_auto::common::time::Clock::Now());
          std::chrono::duration<double, std::micro> elapsed{end - start};
          if (elapsed < default_period)
          {
            std::this_thread::sleep_for(default_period - elapsed);
          }
          else
          {
            AERROR
                << "Too much time consumption in Cx75Controller looping process:"
                << elapsed.count();
          }
        }
      }

      bool Cx75Controller::CheckResponse(const int32_t flags, bool need_wait)
      {
        int32_t retry_num = 20;
        ChassisDetail chassis_detail;
        bool is_eps_online = false;
        bool is_vcu_online = false;
        bool is_esp_online = false;
        bool is_apa_online=false;
        bool is_epspam_online=false;

        do
        {
          if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK)
          {
            AERROR_EVERY(100) << "get chassis detail failed.";
            return false;
          }
          bool check_ok = true;
          if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG)
          {
            is_eps_online = chassis_detail.has_check_response() &&
                            chassis_detail.check_response().has_is_eps_online() &&
                            chassis_detail.check_response().is_eps_online();
            check_ok = check_ok && is_eps_online;
          }

          if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG)
          {
            // is_vcu_online = chassis_detail.has_check_response() &&
            //                 chassis_detail.check_response().has_is_vcu_online() &&
            //                 chassis_detail.check_response().is_vcu_online();
            is_esp_online = chassis_detail.has_check_response() &&
                            chassis_detail.check_response().has_is_esp_online() &&
                            chassis_detail.check_response().is_esp_online();
            // check_ok = check_ok && is_vcu_online && is_esp_online;
            check_ok = check_ok && is_esp_online;
          }
          if (flags & CHECK_RESPONSE_APA_UNIT_FLAG){
            is_apa_online = chassis_detail.has_check_response() &&
                            chassis_detail.check_response().has_is_apa_online() &&
                            chassis_detail.check_response().is_apa_online();
            check_ok = check_ok && is_apa_online;
          }
          
          if (flags & CHECK_RESPONSE_ESPPAM_UNIT_FLAG){
            is_epspam_online = chassis_detail.has_check_response() &&
                            chassis_detail.check_response().has_is_epspam_online() &&
                            chassis_detail.check_response().is_epspam_online();
            check_ok = check_ok && is_epspam_online;
          }

          AINFO << "check_ok." << check_ok << "  is_vcu_online." << is_vcu_online << "  is_esp_online." << is_esp_online<<" is_apa_online."<<is_apa_online<<"is_epspam_online."<<is_epspam_online;
          if (check_ok)
          {
		    //CANBUS正常时，恢复正常
            AINFO<<"check_ok NO_ERROR";
            set_chassis_error_code(Chassis::NO_ERROR);
            return true;
          }
          else
          {
            AINFO << "Need to check response again.";
          }
          if (need_wait)
          {
            --retry_num;
            std::this_thread::sleep_for(
                std::chrono::duration<double, std::milli>(40));
          }
        } while (need_wait && retry_num);

        AINFO << "check_response fail: is_eps_online:" << is_eps_online
              << ", is_vcu_online:" << is_vcu_online
              << ", is_esp_online:" << is_esp_online;

        return false;
        /* int32_t retry_num = 20;
        ChassisDetail chassis_detail;
        bool is_eps_online = false;
        bool is_pam_esp_online = false;
        bool is_esp_online = false;
        bool is_pam_eps_online = false;
        do
        {
          if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK)
          {
            AERROR_EVERY(100) << "get chassis detail failed.";
            return false;
          }
          bool check_ok = true;
          if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG)
          {
            is_eps_online = chassis_detail.cx75().has_eps_advanced_0x176_176() &&
                            chassis_detail.cx75().eps_advanced_0x176_176().has_eps_lkacontrolstatus() &&
                            chassis_detail.cx75().eps_advanced_0x176_176().eps_lkacontrolstatus()==Eps_advanced_0x176_176::EPS_LKACONTROLSTATUS_REQUEST_HONORED;
            check_ok = check_ok && is_eps_online;
          }

          if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG)
          {
            is_esp_online = chassis_detail.cx75().has_esp_vlc_0x223_223() &&
                            chassis_detail.cx75().esp_vlc_0x223_223().has_esp_vlc_active() &&
                            chassis_detail.cx75().esp_vlc_0x223_223().esp_vlc_active();
            check_ok = check_ok && is_esp_online;
          }

          if (flags & CHECK_RESPONSE_PAM_ESP_UNIT_FLAG)
          {
            is_pam_esp_online = chassis_detail.cx75().has_esp_vlc_0x223_223() &&
                               chassis_detail.cx75().esp_vlc_0x223_223().has_esp_pam_lc_status() &&
                               chassis_detail.cx75().esp_vlc_0x223_223().esp_pam_lc_status()==Esp_vlc_0x223_223::ESP_PAM_LC_STATUS_ACTIVE_AUTOMATICPARK;
            check_ok = check_ok && is_pam_esp_online;
          }
          if (flags & CHECK_RESPONSE_PAM_EPS_UNIT_FLAG)
          {
            is_pam_eps_online = chassis_detail.cx75().has_eps_advanced_0x176_176() &&
                            chassis_detail.cx75().eps_advanced_0x176_176().has_eps_epspamsts() &&
                            chassis_detail.cx75().eps_advanced_0x176_176().eps_epspamsts()==Eps_advanced_0x176_176::EPS_EPSPAMSTS_ACTIVE;
            check_ok = check_ok && is_pam_eps_online;
          }

          if (check_ok)
          {
            return true;
          }
          else
          {
            AINFO << "Need to check response again.";
          }
          if (need_wait)
          {
            --retry_num;
            std::this_thread::sleep_for(
                std::chrono::duration<double, std::milli>(20));
          }
        } while (need_wait && retry_num);

        AINFO << "check_response fail: is_eps_online:" << is_eps_online
              << ", is_esp_online:" << is_esp_online
              << ", is_pam_esp_online:" << is_pam_esp_online
              << ", is_pam_eps_online:" << is_pam_eps_online;
        return false;*/
      }

      void Cx75Controller::set_chassis_error_mask(const int32_t mask)
      {
        std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
        chassis_error_mask_ = mask;
      }

      int32_t Cx75Controller::chassis_error_mask()
      {
        std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
        return chassis_error_mask_;
      }

      Chassis::ErrorCode Cx75Controller::chassis_error_code()
      {
        std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
        return chassis_error_code_;
      }

      void Cx75Controller::set_chassis_error_code(
          const Chassis::ErrorCode &error_code)
      {
        std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
        chassis_error_code_ = error_code;
      }

    } // namespace cx75
  }   // namespace canbus
} // namespace jmc_auto
