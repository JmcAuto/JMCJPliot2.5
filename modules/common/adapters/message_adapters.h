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

#ifndef MODULES_ADAPTERS_MESSAGE_ADAPTERS_H_
#define MODULES_ADAPTERS_MESSAGE_ADAPTERS_H_

#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "velodyne_msgs/VelodyneScanUnified.h"

#include "modules/calibration/republish_msg/proto/relative_odometry.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/common/monitor_log/proto/monitor_log.pb.h"
#include "modules/common/proto/drive_event.pb.h"
#include "modules/control/proto/control_cmd.pb.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/data/proto/static_info.pb.h"
//#include "modules/dreamview/proto/voice_detection.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/drivers/gnss/proto/gnss_status.pb.h"
#include "modules/drivers/gnss/proto/heading.pb.h"
#include "modules/drivers/gnss/proto/imu.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/proto/mobileye.pb.h"
// #include "modules/drivers/proto/racobit_radar.pb.h"
// #include "modules/drivers/proto/ultrasonic_radar.pb.h"
#include "modules/guardian/proto/guardian.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/proto/imu.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/sins_pva.pb.h"
//#include "modules/map/relative_map/proto/navigation.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"
#include "modules/remotecontrol/proto/remote_control.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"

//#include "modules/localization_dy/proto/localization.pb.h"
#include "modules/common/adapters/adapter.h"

#include "modules/drivers/proto/tte_conti_radar.pb.h"
#include "modules/perception/proto/slotcheck_Info.pb.h"
#include <sensor_msgs/Imu.h>

/**
 * @file message_adapters.h
 * @namespace jmc_auto::common::adapter
 * @brief This is an agglomeration of all the message adapters supported that
 * specializes the adapter template.
 */
namespace jmc_auto {
namespace common {
namespace adapter {

using ChassisAdapter = Adapter<::jmc_auto::canbus::Chassis>;
using ChassisDetailAdapter = Adapter<::jmc_auto::canbus::ChassisDetail>;
using ControlCommandAdapter = Adapter<control::ControlCommand>;
using GpsAdapter = Adapter<jmc_auto::localization::Gps>;
using ImuAdapter = Adapter<localization::CorrectedImu>;
using RawImuAdapter = Adapter<jmc_auto::drivers::gnss::Imu>;
using LocalizationAdapter = Adapter<jmc_auto::localization::LocalizationEstimate>;
using MonitorAdapter = Adapter<jmc_auto::common::monitor::MonitorMessage>;
using PadAdapter = Adapter<control::PadMessage>;
using PerceptionObstaclesAdapter = Adapter<perception::PerceptionObstacles>;
using PerceptionRoundviewObstaclesAdapter = Adapter<perception::PerceptionObstacles>;
using PlanningAdapter = Adapter<planning::ADCTrajectory>;
using PointCloudAdapter = Adapter<::sensor_msgs::PointCloud2>;
using VLP16PointCloudAdapter = Adapter<::sensor_msgs::PointCloud2>;
using ImageFrontAdapter = Adapter<::sensor_msgs::Image>;
using ImageShortAdapter = Adapter<::sensor_msgs::Image>;
using ImageLongAdapter = Adapter<::sensor_msgs::Image>;
using ImageRoundviewAdapter = Adapter<::sensor_msgs::Image>;
using PredictionAdapter = Adapter<prediction::PredictionObstacles>;
using DriveEventAdapter = Adapter<DriveEvent>;
using TrafficLightDetectionAdapter = Adapter<perception::TrafficLightDetection>;
using PerceptionLaneMaskAdapter = Adapter<::sensor_msgs::Image>;
using RoutingRequestAdapter = Adapter<routing::RoutingRequest>;
using RoutingResponseAdapter = Adapter<routing::RoutingResponse>;
using RelativeOdometryAdapter =
    Adapter<calibration::republish_msg::RelativeOdometry>;
using InsStatAdapter = Adapter<drivers::gnss::InsStat>;
using InsStatusAdapter = Adapter<drivers::gnss_status::InsStatus>;
using GnssStatusAdapter = Adapter<drivers::gnss_status::GnssStatus>;
using SystemStatusAdapter = Adapter<jmc_auto::monitor::SystemStatus>;
using StaticInfoAdapter = Adapter<jmc_auto::data::StaticInfo>;
using MobileyeAdapter = Adapter<drivers::Mobileye>;
using DelphiESRAdapter = Adapter<drivers::DelphiESR>;
using ContiRadarAdapter = Adapter<drivers::ContiRadar>;
// using RacobitRadarAdapter = Adapter<drivers::RacobitRadar>;
// using UltrasonicAdapter = Adapter<drivers::Ultrasonic>;
using CompressedImageAdapter = Adapter<sensor_msgs::CompressedImage>;
using GnssRtkObsAdapter = Adapter<jmc_auto::drivers::gnss::EpochObservation>;
using GnssRtkEphAdapter = Adapter<jmc_auto::drivers::gnss::GnssEphemeris>;
using GnssBestPoseAdapter = Adapter<jmc_auto::drivers::gnss::GnssBestPose>;
using LocalizationMsfGnssAdapter =
    Adapter<jmc_auto::localization::LocalizationEstimate>;
using LocalizationMsfLidarAdapter =
    Adapter<jmc_auto::localization::LocalizationEstimate>;
using LocalizationMsfSinsPvaAdapter =
    Adapter<jmc_auto::localization::IntegSinsPva>;
using LocalizationMsfStatusAdapter =
    Adapter<jmc_auto::localization::LocalizationStatus>;
using RelativeMapAdapter = Adapter<jmc_auto::relative_map::MapMsg>;
//using NavigationAdapter = Adapter<jmc_auto::relative_map::NavigationInfo>;
//using VoiceDetectionRequestAdapter =
//    Adapter<jmc_auto::dreamview::VoiceDetectionRequest>;
//using VoiceDetectionResponseAdapter =
//    Adapter<jmc_auto::dreamview::VoiceDetectionResponse>;
// for pandora
using PandoraPointCloudAdapter = Adapter<::sensor_msgs::PointCloud2>;
using PandoraCameraFrontColorAdapter = Adapter<::sensor_msgs::Image>;
using PandoraCameraRightGrayAdapter = Adapter<::sensor_msgs::Image>;
using PandoraCameraLeftGrayAdapter = Adapter<::sensor_msgs::Image>;
using PandoraCameraFrontGrayAdapter = Adapter<::sensor_msgs::Image>;
using PandoraCameraBackGrayAdapter = Adapter<::sensor_msgs::Image>;
using GuardianAdapter = Adapter<jmc_auto::guardian::GuardianCommand>;
using GnssRawDataAdapter = Adapter<std_msgs::String>;
using StreamStatusAdapter = Adapter<drivers::gnss_status::StreamStatus>;
using GnssHeadingAdapter = Adapter<drivers::gnss::Heading>;
using RtcmDataAdapter = Adapter<std_msgs::String>;

// for velodyne
using VelodyneRaw0Adapter = Adapter<velodyne_msgs::VelodyneScanUnified>;
using VelodyneRaw1Adapter = Adapter<velodyne_msgs::VelodyneScanUnified>;
using PointCloudRaw0Adapter = Adapter<::sensor_msgs::PointCloud2>;
using PointCloudRaw1Adapter = Adapter<::sensor_msgs::PointCloud2>;
using PointCloudFusionAdapter = Adapter<::sensor_msgs::PointCloud2>;
//for remotecontrol
using RemoteControlAdapter = Adapter<::jmc_auto::remote::RemoteControl>;

//using LocalizationdyAdapter = Adapter<jmc_auto::localization_dy::LocalizationEstimate>;

using TteContiRadarAdapter = Adapter<drivers::TteContiRadar>;
using ValidSlotAdapter = Adapter<::jmc_auto::perception::MulvalidSlot>;

using RosImuAdapter = Adapter<sensor_msgs::Imu>;

using InsAdapter = Adapter<drivers::gnss::Ins>;

using PlanningPadAdapter = Adapter<jmc_auto::planning::PadMessage>;
}  // namespace adapter
}  // namespace common
}  // namespace jmc_auto

#endif  // MODULES_ADAPTERS_MESSAGE_ADAPTERS_H_
