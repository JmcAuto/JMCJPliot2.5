/******************************************************************************
 * Copyright 2017 The jmc_auto Authors. All Rights Reserved.
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

#include "modules/localization/vins/vins_localization.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/time/time.h"
#include "modules/common/time/time_util.h"
#include "modules/localization/common/localization_gflags.h"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"//图形用户界面

#include "modules/localization/vins/apriltag/tag36h11.h"
#include "modules/perception/traffic_light/base/image.h"

namespace jmc_auto
{
  namespace localization
  {

    using ::Eigen::Vector3d;
    using jmc_auto::common::Status;
    using jmc_auto::common::adapter::AdapterManager;
    using jmc_auto::common::adapter::ImuAdapter;
    using jmc_auto::common::monitor::MonitorMessageItem;
    using jmc_auto::common::time::Clock;
    using namespace std;
    using namespace cv;
    using jmc_auto::perception::traffic_light::Image;

    // constexpr double DEG_TO_RAD_LOCAL = M_PI / 180.0;
    // const char *WGS84_TEXT = "+proj=latlong +ellps=WGS84";

    VINSLocalization::VINSLocalization()
        : monitor_logger_(MonitorMessageItem::LOCALIZATION) {}

    VINSLocalization::~VINSLocalization() {}

    Status VINSLocalization::Start()
    {
      AdapterManager::Init(FLAGS_vins_adapter_config_file);

      // start ROS timer, one-shot = false, auto-start = true
      const double duration = 1.0 / FLAGS_localization_publish_freq;
      timer_ = AdapterManager::CreateTimer(ros::Duration(duration),
                                           &VINSLocalization::OnTimer, this);

      common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
      if (!AdapterManager::GetChassis())
      {
        buffer.ERROR() << "chassis input not initialized. Check file "
                       << FLAGS_vins_adapter_config_file;
        buffer.PrintLog();
        return Status(common::LOCALIZATION_ERROR, "no chassis_ins adapter");
      }
      if (!AdapterManager::GetImageShort())
      {
        buffer.ERROR() << "ImageFront input not initialized. Check file "
                       << FLAGS_vins_adapter_config_file;
        buffer.PrintLog();
        return Status(common::LOCALIZATION_ERROR, "no ImageFront adapter");
      }
//Apriltag
      td = apriltag_detector_create();
      tf = tag36h11_create();
      apriltag_detector_add_family(td, tf);
      td->quad_decimate = FLAGS_decimate;
      td->quad_sigma = FLAGS_blur;
      td->nthreads = FLAGS_threads;
      td->debug = FLAGS_debug;
      td->refine_edges = FLAGS_refine_edges;

      wgs84pj_source_ = pj_init_plus(WGS84_TEXT);
      utm_target_ = pj_init_plus(FLAGS_proj4_text.c_str());

      tf2_broadcaster_.reset(new tf2_ros::TransformBroadcaster);

      return Status::OK();
    }

    Status VINSLocalization::Stop()
    {
      timer_.stop();
      apriltag_detector_destroy(td);
      tag36h11_destroy(tf);
      return Status::OK();
    }

    void VINSLocalization::OnTimer(const ros::TimerEvent &event)
    {
      LocalizationEstimate localization;
      AdapterManager::Observe();
     
      // if (AdapterManager::GetChassis()->Empty())
      // {
      //   AERROR << "No Chassis msg yet. ";
      //   return;
      // }
      if (AdapterManager::GetImageShort()->Empty()&&AdapterManager::GetImageFront()->Empty())
      {
        AERROR << "No ImageShort msg yet. ";
        return;
      }
      apriltag_detection_info_t info;
      // chassis_ = AdapterManager::GetChassis()->GetLatestObserved();
      if(!AdapterManager::GetImageShort()->Empty())
      {
        Image_msg_ = AdapterManager::GetImageShort()->GetLatestObservedPtr();
        info.fx = 1951.794795020507;
        info.fy = 1939.447620233487;
        info.cx = 962.6956492117581;
        info.cy = 457.1595628519383;
        AINFO<<"ImageShort";
      }
      else
      {
        Image_msg_ = AdapterManager::GetImageFront()->GetLatestObservedPtr();
        //info.fx = 4032.838046853612;
        //info.fy = 4001.820307183489;
        //info.cx = 921.8927892481082;
        //info.cy = 411.5126764986777;
		info.fx = 1951.794795020507;
        info.fy = 1939.447620233487;
        info.cx = 962.6956492117581;
        info.cy = 457.1595628519383;
        AINFO<<"imagefront";
      }
      
      // Image_msg_ = AdapterManager::GetImageShort()->GetLatestObservedPtr();
      //解析摄像头传来的数据
      std::shared_ptr<Image> image(new Image);
      cv::Mat cv_mat;
      double timestamp = Image_msg_->header.stamp.toSec();
      image->Init(timestamp, jmc_auto::perception::traffic_light::CameraId::SHORT_FOCUS, Image_msg_);
      image->GenerateMat();
      // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(Image_msg_, sensor_msgs::image_encodings::YUV422);
      Mat gray;
      // img_ = cv_ptr->image;
      img_=image->mat();
      //cvtColor(img_, gray, COLOR_BGR2GRAY);
      cvtColor(img_, gray, COLOR_BGR2GRAY);

      // AINFO<<"gray.rows: "<<gray.rows;
      // Make an image_u8_t header for the Mat data
      image_u8_t im = {.width = gray.cols,
                       .height = gray.rows,
                       .stride = gray.cols,
                       .buf = gray.data};
      zarray_t *detections = apriltag_detector_detect(td, &im);
      
      // AINFO<<"detections: "<<zarray_size(detections);
      //在图片上显示线和ID
      for (int i = 0; i < zarray_size(detections); i++)
      {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        // line(img_, Point(det->p[0][0], det->p[0][1]),
        //      Point(det->p[1][0], det->p[1][1]),
        //      Scalar(0, 0xff, 0), 2); // Scalar(0, 0xff, 0), 2)表示线的颜色
        // line(img_, Point(det->p[0][0], det->p[0][1]),
        //      Point(det->p[3][0], det->p[3][1]),
        //      Scalar(0, 0, 0xff), 2);
        // line(img_, Point(det->p[1][0], det->p[1][1]),
        //      Point(det->p[2][0], det->p[2][1]),
        //      Scalar(0xff, 0, 0), 2);
        // line(img_, Point(det->p[2][0], det->p[2][1]),
        //      Point(det->p[3][0], det->p[3][1]),
        //      Scalar(0xff, 0, 0), 2);

        stringstream ss;
        ss << det->id;
        String text = ss.str();
        int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontscale = 1.0;
        int baseline;
        Size textsize = getTextSize(text, fontface, fontscale, 2,
                                    &baseline);
        // putText(img_, text, Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2),
        //         fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        // First create an apriltag_detection_info_t struct using your known parameters.
        // apriltag_detection_info_t info;
        info.det = det;
        info.tagsize = 0.16;
        // info.fx = 4015.903975042527;
        // info.fy = 4015.593038858476;
        // info.cx = 1068.414694179811;
        // info.cy = 446.8659255163414;
        // info.fx = 1;
        // info.fy = 1;
        // info.cx = 0;
        // info.cy = 0;


        // Then call estimate_tag_pose.
        apriltag_pose_t pose;
        double err = estimate_tag_pose(&info, &pose);
        AINFO << "pose_R  " << pose.R->data[0] << " " << pose.R->data[1] << " " << pose.R->data[2] << " " << pose.R->data[3] << " " << pose.R->data[4]
             << " " << pose.R->data[5] << " " << pose.R->data[6] << " " << pose.R->data[7] << " " << pose.R->data[8] << " " << pose.R->data[9] << endl
             << "   pose_t " << pose.t->data[0] << "  " << pose.t->data[1] << " " << pose.t->data[2] << endl;
        Eigen::Matrix3d rotation_matrix;
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

        rotation_matrix << pose.R->data[0], pose.R->data[1], pose.R->data[2], pose.R->data[3], pose.R->data[4], pose.R->data[5], pose.R->data[6], pose.R->data[7], pose.R->data[8];
        T.rotate(rotation_matrix);
        T.pretranslate(Eigen::Vector3d(pose.t->data[0], pose.t->data[1], pose.t->data[2]));
        Eigen::Vector3d v(0, 0, 0);
        Eigen::Vector3d v_transformed = T * v;
        AINFO << "position: x:" << v_transformed[0] << "  y:" << v_transformed[1] << "  z:" << v_transformed[2] << endl;
      }
      apriltag_detections_destroy(detections);
      // tag36h11_destroy(tf);
      // imshow("Tag Detections", img_);
      double time_delay =
          common::time::ToSecond(Clock::Now()) - last_received_timestamp_sec_;
      common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

      if (FLAGS_enable_gps_timestamp &&
          time_delay > FLAGS_gps_time_delay_tolerance)
      {
        buffer.ERROR() << "GPS message time delay: " << time_delay;
        buffer.PrintLog();
      }
      PrepareLocalizationMsg(&localization);

      // publish localization messages
      PublishLocalization();
      // service_started_ = true;
      // watch dog

      last_received_timestamp_sec_ = common::time::ToSecond(Clock::Now());
    }

    void VINSLocalization::PublishLocalization()
    {
      //   LocalizationEstimate localization;
      //   PrepareLocalizationMsg(&localization);

      //   // publish localization messages
      //   AdapterManager::PublishLocalization(localization);
      //   PublishPoseBroadcastTF(localization);
      //   ADEBUG << "[OnTimer]: Localization message publish success!";
      //   RunWatchDog(&localization);
    }

    void VINSLocalization::PrepareLocalizationMsg(LocalizationEstimate *localization)
    {

      //   double x = chassis_.ins_longitude();
      //   double y = chassis_.ins_latitude();
      //   x *= DEG_TO_RAD_LOCAL;
      //   y *= DEG_TO_RAD_LOCAL;
      //   pj_transform(wgs84pj_source_, utm_target_, 1, 1, &x, &y, NULL);

      //   localization->mutable_pose()->mutable_position()->set_x(x);
      //   localization->mutable_pose()->mutable_position()->set_y(y);
      //   localization->mutable_pose()->mutable_position()->set_z(chassis_.ins_locatheight());
      //     // 2. orientation 欧拉角以正北为0，航向角以正东为0
      //   double CarHeadingAngle=chassis_.ins_headingangle()<-90?-(270+chassis_.ins_headingangle()):-(chassis_.ins_headingangle()-90);
      //   Eigen::Quaterniond q =
      //   Eigen::AngleAxisd(CarHeadingAngle* DEG_TO_RAD_LOCAL,
      //                     Eigen::Vector3d::UnitZ()) *
      //   Eigen::AngleAxisd(chassis_.ins_rollangle()* DEG_TO_RAD_LOCAL, Eigen::Vector3d::UnitX()) *
      //   Eigen::AngleAxisd(chassis_.ins_pitchangle()* DEG_TO_RAD_LOCAL, Eigen::Vector3d::UnitY());//欧拉角
      //   localization->mutable_pose()->mutable_orientation()->set_qx(q.x());
      //   localization->mutable_pose()->mutable_orientation()->set_qy(q.y());
      //   localization->mutable_pose()->mutable_orientation()->set_qz(q.z());
      //   localization->mutable_pose()->mutable_orientation()->set_qw(q.w());

      //   localization->mutable_pose()->mutable_linear_velocity()->set_x(chassis_.ins_eastspd());
      //   localization->mutable_pose()->mutable_linear_velocity()->set_y(chassis_.ins_northspd());
      //   localization->mutable_pose()->mutable_linear_velocity()->set_z(-chassis_.ins_togroundspd());
      //    if (localization->pose().has_orientation()) {
      //     Vector3d orig(chassis_.acc_y(),
      //                     chassis_.acc_x(),
      //                     -chassis_.acc_z());
      //       Vector3d vec = common::math::QuaternionRotate(
      //           localization->pose().orientation(), orig);
      //       localization->mutable_pose()->mutable_linear_acceleration()->set_x(vec[0]);
      //       localization->mutable_pose()->mutable_linear_acceleration()->set_y(vec[1]);
      //       localization->mutable_pose()->mutable_linear_acceleration()->set_z(vec[2]);

      //    }
      //   localization->mutable_pose()->mutable_linear_acceleration_vrf()->set_x(chassis_.acc_y());
      //   localization->mutable_pose()->mutable_linear_acceleration_vrf()->set_y(chassis_.acc_x());
      //   localization->mutable_pose()->mutable_linear_acceleration_vrf()->set_z(-chassis_.acc_z());

      //   if (localization->pose().has_orientation()) {
      //         Vector3d orig(chassis_.gyro_y(), chassis_.gyro_x(),
      //                           chassis_.gyro_z());
      //             Vector3d vec = common::math::QuaternionRotate(
      //                 localization->pose().orientation(), orig);
      //             localization->mutable_pose()->mutable_angular_velocity()->set_x(vec[0]);
      //             localization->mutable_pose()->mutable_angular_velocity()->set_y(vec[1]);
      //             localization->mutable_pose()->mutable_angular_velocity()->set_z(vec[2]);
      //   }
      //   localization->mutable_pose()->mutable_angular_velocity_vrf()->set_x(chassis_.gyro_y());
      //   localization->mutable_pose()->mutable_angular_velocity_vrf()->set_y(chassis_.gyro_x());
      //   localization->mutable_pose()->mutable_angular_velocity_vrf()->set_z(chassis_.gyro_z());

      //   localization->mutable_pose()->set_heading(CarHeadingAngle* DEG_TO_RAD_LOCAL);

      //   localization->mutable_pose()->mutable_euler_angles()->set_x(chassis_.ins_rollangle()* DEG_TO_RAD_LOCAL);
      //   localization->mutable_pose()->mutable_euler_angles()->set_y(chassis_.ins_pitchangle()* DEG_TO_RAD_LOCAL);
      //   localization->mutable_pose()->mutable_euler_angles()->set_z(CarHeadingAngle* DEG_TO_RAD_LOCAL);

      //   //localization->mutable_pose()->mutable_uncertainty()->mutable_position_std_dev()->set_x(ins_std_lat);
      //   AdapterManager::FillLocalizationHeader("localization",
      //                                      localization);
      //   localization->set_measurement_time(common::time::TimeUtil::Gps2unix(chassis_.ins_time()));
    }

    void VINSLocalization::RunWatchDog(LocalizationEstimate *localization)
    {
      //   if (!FLAGS_enable_watchdog) {
      //     return;
      //   }

      //   common::monitor::MonitorLogBuffer buffer(&monitor_logger_);

      //   // check ins time stamp against ROS timer
      //   double ins_delay_sec =
      //       common::time::ToSecond(Clock::Now()) -
      //       localization->measurement_time();
      //   int64_t ins_delay_cycle_cnt =
      //       static_cast<int64_t>(ins_delay_sec * FLAGS_localization_publish_freq);

      //   bool msg_lost = false;
      //   if (FLAGS_enable_gps_timestamp &&
      //       (ins_delay_cycle_cnt > FLAGS_report_threshold_err_num)) {
      //     msg_lost = true;

      //     buffer.ERROR() << "Raw GPS Message Lost. GPS message is "
      //                    << ins_delay_cycle_cnt << " cycle " << ins_delay_sec
      //                    << " sec behind current time.";
      //     buffer.PrintLog();
      //   }

      //   // to prevent it from beeping continuously
      //   if (msg_lost && (last_reported_timestamp_sec_ < 1. ||
      //                    common::time::ToSecond(Clock::Now()) >
      //                        last_reported_timestamp_sec_ + 1.)) {
      //     AERROR << "gps/imu frame lost!";
      //     last_reported_timestamp_sec_ = common::time::ToSecond(Clock::Now());
      //   }
    }

  } // namespace localization
} // namespace jmc_auto
