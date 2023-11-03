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

#include "modules/common/adapters/adapter_gflags.h"

DEFINE_bool(enable_adapter_dump, false,
            "Whether enable dumping the messages to "
            "/tmp/adapters/<topic_name>/<seq_num>.txt for debugging purposes.");
DEFINE_string(gps_topic, "/jmc_auto/sensor/gnss/odometry", "GPS topic name");
DEFINE_string(imu_topic, "/jmc_auto/sensor/gnss/corrected_imu", "IMU topic name");
DEFINE_string(hmi_status_topic, "/jmc_auto/hmi/status", "HMI status topic name.");
DEFINE_string(raw_imu_topic, "/jmc_auto/sensor/gnss/imu", "Raw IMU topic name");
DEFINE_string(chassis_topic, "/jmc_auto/canbus/chassis", "chassis topic name");
DEFINE_string(chassis_detail_topic, "/jmc_auto/canbus/chassis_detail",
              "chassis detail topic name");
DEFINE_string(localization_topic, "/jmc_auto/localization/pose",
              "localization topic name");
DEFINE_string(planning_trajectory_topic, "/jmc_auto/planning",
              "planning trajectory topic name");
DEFINE_string(monitor_topic, "/jmc_auto/monitor", "ROS topic for monitor");
DEFINE_string(pad_topic, "/jmc_auto/control/pad",
              "control pad message topic name");
DEFINE_string(control_command_topic, "/jmc_auto/control",
              "control command topic name");
DEFINE_string(pointcloud_topic,
              "/jmc_auto/sensor/velodyne64/compensator/PointCloud2",
              "pointcloud topic name");
DEFINE_string(pointcloud_raw_topic, "/jmc_auto/sensor/velodyne64/PointCloud2",
              "pointcloud raw topic name");
DEFINE_string(velodyne_raw_topic,
              "/jmc_auto/sensor/velodyne64/VelodyneScanUnified",
              "velodyne64 raw data topic name");
DEFINE_string(pointcloud_fusion_topic,
              "/jmc_auto/sensor/velodyne64/fusion/PointCloud2",
              "pointcloud fusion topic name");
DEFINE_string(vlp16_pointcloud_topic,
              "/jmc_auto/sensor/velodyne16/compensator/PointCloud2",
              "16 beam Lidar pointcloud topic name");
DEFINE_string(prediction_topic, "/jmc_auto/prediction", "prediction topic name");
DEFINE_string(perception_obstacle_topic, "/jmc_auto/perception/obstacles",
              "perception obstacle topic name");
DEFINE_string(perception_roundview_obstacle_topic,"/jmc_auto/perception/roundview_obstacles",
              "perception roundview obstacle topic name");
DEFINE_string(drive_event_topic, "/jmc_auto/drive_event",
              "drive event topic name");
DEFINE_string(traffic_light_detection_topic, "/jmc_auto/perception/traffic_light",
              "traffic light detection topic name");
DEFINE_string(perception_lane_mask_segmentation_topic,
              "/jmc_auto/perception/lane_mask",
              "lane mask segmentation topic name");
DEFINE_string(routing_request_topic, "/jmc_auto/routing_request",
              "routing request topic name");
DEFINE_string(routing_response_topic, "/jmc_auto/routing_response",
              "routing response topic name");
DEFINE_string(relative_odometry_topic, "/jmc_auto/calibration/relative_odometry",
              "relative odometry topic name");
DEFINE_string(ins_stat_topic, "/jmc_auto/sensor/gnss/ins_stat",
              "ins stat topic name");
DEFINE_string(ins_status_topic, "/jmc_auto/sensor/gnss/ins_status",
              "ins status topic name");
DEFINE_string(gnss_status_topic, "/jmc_auto/sensor/gnss/gnss_status",
              "gnss status topic name");
DEFINE_string(system_status_topic, "/jmc_auto/monitor/system_status",
              "System status topic name");
DEFINE_string(static_info_topic, "/jmc_auto/monitor/static_info",
              "Static info topic name");
DEFINE_string(mobileye_topic, "/jmc_auto/sensor/mobileye", "mobileye topic name");
DEFINE_string(delphi_esr_topic, "/jmc_auto/sensor/delphi_esr",
              "delphi esr radar topic name");
DEFINE_string(conti_radar_topic, "/jmc_auto/sensor/conti_radar",
              "continental radar topic name");
DEFINE_string(conti_radar_front_topic, "/jmc_auto/sensor/radar_front",
              "continental radar topic name");
DEFINE_string(conti_radar_left_topic, "/jmc_auto/sensor/radar_left",
              "continental radar topic name");
DEFINE_string(conti_radar_right_topic, "/jmc_auto/sensor/radar_right",
              "continental radar topic name");
DEFINE_string(racobit_radar_topic, "/jmc_auto/sensor/racobit_radar",
              "racobit radar topic name");
DEFINE_string(ultrasonic_radar_topic, "/jmc_auto/sensor/ultrasonic_radar",
              "ultrasonic esr radar topic name");
// TODO(Authors): Change the topic name
DEFINE_string(compressed_image_topic, "camera/image_raw",
              "CompressedImage topic name");
DEFINE_string(image_front_topic, "/jmc_auto/sensor/camera/obstacle/front_6mm",
              "front camera image topic name for obstacles from camera");
DEFINE_string(image_short_topic, "/jmc_auto/sensor/camera/traffic/image_short",
              "short camera image topic name");
DEFINE_string(image_long_topic, "/jmc_auto/sensor/camera/traffic/image_long",
              "long camera image topic name");
DEFINE_string(image_roundview_topic, "/jmc_auto/sensor/camera/roundview",
              "roundview camera image topic name");
DEFINE_string(gnss_rtk_obs_topic, "/jmc_auto/sensor/gnss/rtk_obs",
              "Gnss rtk observation topic name");
DEFINE_string(gnss_rtk_eph_topic, "/jmc_auto/sensor/gnss/rtk_eph",
              "Gnss rtk ephemeris topic name");
DEFINE_string(gnss_best_pose_topic, "/jmc_auto/sensor/gnss/best_pose",
              "Gnss rtk best gnss pose");
DEFINE_string(localization_gnss_topic, "/jmc_auto/localization/msf_gnss",
              "Gnss localization measurement topic name");
DEFINE_string(localization_lidar_topic, "/jmc_auto/localization/msf_lidar",
              "Lidar localization measurement topic name");
DEFINE_string(localization_sins_pva_topic, "/jmc_auto/localization/msf_sins_pva",
              "Localization sins pva topic name");
DEFINE_string(localization_msf_status, "/jmc_auto/localization/msf_status",
              "msf localization status");
DEFINE_string(relative_map_topic, "/jmc_auto/relative_map", "relative map");
//DEFINE_string(navigation_topic, "/jmc_auto/navigation", "navigation");
//DEFINE_string(voice_detection_request_topic,
//              "/jmc_auto/hmi/voice_detection_request",
//              "Voice detetection request topic name.");
//DEFINE_string(voice_detection_response_topic,
//              "/jmc_auto/hmi/voice_detection_response",
//              "Voice detetection response topic name.");
// For pandora.
DEFINE_string(pandora_pointcloud_topic,
              "/jmc_auto/sensor/pandora/hesai40/PointCloud2",
              "pandora pointcloud topic name");
DEFINE_string(pandora_camera_front_color_topic,
              "/jmc_auto/sensor/pandora/camera/front_color",
              "pandora front color camera topic name");
DEFINE_string(pandora_camera_right_gray_topic,
              "/jmc_auto/sensor/pandora/camera/right_gray",
              "pandora right gray camera topic name");
DEFINE_string(pandora_camera_left_gray_topic,
              "/jmc_auto/sensor/pandora/camera/left_gray",
              "pandora left gray camera topic name");
DEFINE_string(pandora_camera_front_gray_topic,
              "/jmc_auto/sensor/pandora/camera/front_gray",
              "pandora front gray camera topic name");
DEFINE_string(pandora_camera_back_gray_topic,
              "/jmc_auto/sensor/pandora/camera/back_gray",
              "pandora back gray camera topic name");
DEFINE_string(guardian_topic, "/jmc_auto/guardian", "Guardian topic.");
DEFINE_string(gnss_raw_data_topic, "/jmc_auto/sensor/gnss/raw_data",
              "gnss raw data topic name");
DEFINE_string(stream_status_topic, "/jmc_auto/sensor/gnss/stream_status",
              "gnss stream status topic name");
DEFINE_string(heading_topic, "/jmc_auto/sensor/gnss/heading",
              "gnss heading topic name");
DEFINE_string(rtcm_data_topic, "/jmc_auto/sensor/gnss/rtcm_data",
              "gnss rtcm data topic name");
DEFINE_string(remote_control_topic, "/jmc_auto/remote/remote_control", "REMOTE CONTROL topic name");

//DEFINE_string(localization_dy_topic, "/jmc_auto/localization_dy/pose", "localization_dy topic name");
DEFINE_string(ros_imu_topic, "/jmc_auto/imu", "imu topic name");

//for valid slot
DEFINE_string(valid_slot_topic, "/jmc_auto/slotcheck", "valid slot topic name");

DEFINE_string(tte_conti_radar_topic, "/jmc_auto/sensor/tte_conti_radar", "tte_conti_radar topic name");

DEFINE_string(ins_topic, "/jmc_auto/sensor/gnss/ins", "ins topic name");

DEFINE_string(planning_pad_topic, "/jmc_auto/planning/pad", "planning pad message name");
// DEFINE_string(ros_imu_topic, "/jmc_auto/imu", "imu topic name");
