/******************************************************************************
 * Copyright 2018 The JmcAuto Authors. All Rights Reserved.
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

#include "modules/perception/obstacle/onboard/camera_process_subnode.h"
//20210310 add head file
#include "modules/perception/onboard/transform_input.h"
#include "eigen_conversions/eigen_msg.h"
#include "ros/include/ros/ros.h"
//20210310
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"

namespace jmc_auto {
namespace perception {

using jmc_auto::common::adapter::AdapterManager;

bool CameraProcessSubnode::InitInternal() {
  // Subnode config in DAG streaming
  std::unordered_map<std::string, std::string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields);

  if (fields.count("device_id")) device_id_ = fields["device_id"];
  if (fields.count("pb_obj") && stoi(fields["pb_obj"])) pb_obj_ = true;
  if (fields.count("pb_ln_msk") && stoi(fields["pb_ln_msk"])) pb_ln_msk_ = true;

  // Shared Data
  cam_obj_data_ = static_cast<CameraObjectData *>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  // cam_shared_data_ = static_cast<CameraSharedData *>(
  //     shared_data_manager_->GetSharedData("CameraSharedData"));

  InitCalibration();

  InitModules();

  AdapterManager::AddImageFrontCallback(&CameraProcessSubnode::ImgCallback,
                                        this);
  if (pb_obj_) {
    AdapterManager::AddChassisCallback(&CameraProcessSubnode::ChassisCallback,
                                       this);
  }
  ADEBUG<< "CAMMERA INIT SUCC.";
  return true;
}

bool CameraProcessSubnode::InitCalibration() {
  auto ccm = Singleton<CalibrationConfigManager>::get();
  CameraCalibrationPtr calibrator = ccm->get_camera_calibration();

  calibrator->get_image_height_width(&image_height_, &image_width_);
  //camera_to_car_ = calibrator->get_camera_extrinsics();
  camera_to_car_(0,0) =  1;
  camera_to_car_(0,1) =  0;
  camera_to_car_(0,2) =  0;
  camera_to_car_(0,3) =  0;
  camera_to_car_(1,0) =  0;
  camera_to_car_(1,1) =  1;
  camera_to_car_(1,2) =  0;
  camera_to_car_(1,3) =  0;
  camera_to_car_(2,0) =  0;
  camera_to_car_(2,1) =  0;
  camera_to_car_(2,2) =  1;
  camera_to_car_(2,3) =  0;
  camera_to_car_(3,0) =  0;
  camera_to_car_(3,1) =  0;
  camera_to_car_(3,2) =  0;
  camera_to_car_(3,3) =  1;
  
  intrinsics_ = calibrator->get_camera_intrinsic();
  return true;
}

bool CameraProcessSubnode::InitModules() {
  //RegisterFactoryYoloCameraDetector();
  RegisterFactoryJmcCaffeDetector();
  RegisterFactoryJmcCameraTracker();
  RegisterFactoryGeometryCameraConverter();
  RegisterFactoryCascadedCameraTracker();
  RegisterFactoryFlatCameraTransformer();
  RegisterFactoryObjectCameraFilter();

  detector_.reset(
      BaseCameraDetectorRegisterer::GetInstanceByName("JmcCaffeDetector"));
  detector_->Init();
  use_yolo_v5 = true;

  converter_.reset(BaseCameraConverterRegisterer::GetInstanceByName(
      "GeometryCameraConverter"));
  converter_->Init();

  tracker_.reset(
      BaseCameraTrackerRegisterer::GetInstanceByName("CascadedCameraTracker"));
  tracker_->Init();

  jmc_tracker_.reset(
      BaseCameraTrackerRegisterer::GetInstanceByName("JmcCameraTracker"));
  jmc_tracker_->Init();

  transformer_.reset(BaseCameraTransformerRegisterer::GetInstanceByName(
      "FlatCameraTransformer"));
  transformer_->Init();
  transformer_->SetExtrinsics(camera_to_car_);

  filter_.reset(
      BaseCameraFilterRegisterer::GetInstanceByName("ObjectCameraFilter"));
  filter_->Init();

  return true;
}

void CameraProcessSubnode::ImgCallback(const sensor_msgs::Image &message) {
  double timestamp = message.header.stamp.toSec();
  ADEBUG << "CameraProcessSubnode ImgCallback: timestamp: ";
  ADEBUG << std::fixed << std::setprecision(64) << timestamp;
  AINFO << "camera received image : " << GLOG_TIMESTAMP(timestamp)
        << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
  double curr_timestamp = timestamp * 1e9;

  if (FLAGS_skip_camera_frame && timestamp_ns_ > 0.0) {
    if ((curr_timestamp - timestamp_ns_) < (1e9 / FLAGS_camera_hz) &&
        curr_timestamp > timestamp_ns_) {
      ADEBUG << "CameraProcessSubnode Skip frame";
      return;
    }
  }

  timestamp_ns_ = curr_timestamp;
  ADEBUG << "CameraProcessSubnode Process: "
         << " frame: " << ++seq_num_;
  PERF_FUNCTION("CameraProcessSubnode");
  PERF_BLOCK_START();

  cv::Mat img;
  cv::Mat result_vis;
  if (!FLAGS_image_file_debug) {
    MessageToMat(message, &img);
  } else {
    img = cv::imread(FLAGS_image_file_path, CV_LOAD_IMAGE_COLOR);
  }
  //20210407 flip img because of camera rotation
  /*cv::Point center(img.cols/2,img.rows/2);
	double angle = 180.0;
	double scale = 1.0;
	cv::Mat rotMat = getRotationMatrix2D(center,angle,scale);
	cv::warpAffine(img,img,rotMat,img.size());
  */
  //20210407
  cv::resize(img, img, cv::Size(1920, 1080), 0, 0);
  result_vis = img.clone();
  std::vector<std::shared_ptr<VisualObject>> objects;
  cv::Mat mask;

  PERF_BLOCK_END("CameraProcessSubnode_Image_Preprocess");
  std::shared_ptr<SensorObjects> out_objs(new SensorObjects);
  detector_->Multitask(img, CameraDetectorOptions(), &objects, &mask);
  mask = mask*2;
  // if (FLAGS_use_whole_lane_line) {
  //   cv::Mat mask1;
  //   detector_->Lanetask(img, &mask1);
  //   mask += mask1;
  // }
  PERF_BLOCK_END("CameraProcessSubnode_detector_");

  jmc_tracker_->Associate(img, timestamp, &objects);
  PERF_BLOCK_END("CameraProcessSubnode_jmc_tracker_");

  //20210416 add - Get car to world trans
  const double query_time = message.header.stamp.toSec();
  ros::Time query_stamp(query_time);
  ADEBUG << "query_time:"<<query_time;
  const auto& tf2_buffer = common::adapter::AdapterManager::Tf2Buffer();
  const double kTf2BuffSize = 20 / 1000.0;
  std::string err_msg;
  if (!tf2_buffer.canTransform("world",
                               "novatel", query_stamp,
                               ros::Duration(kTf2BuffSize), &err_msg)) {
    AERROR << "Cannot transform frame: " << "world"
           << " to frame " << "novatel"
           << " , err: " << err_msg
           << ". Frames: " << tf2_buffer.allFramesAsString();
  }
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf2_buffer.lookupTransform(
        "world", "novatel", query_stamp);
  } catch (tf2::TransformException& ex) {
    AERROR << "Exception: " << ex.what();
  }
  Eigen::Affine3d affine_camera_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_camera_3d);
  car_to_world_ = affine_camera_3d.matrix();
  ADEBUG << "get " << "novatel" << " to "
         << "world" << " trans: " << car_to_world_;
  camera_to_world_ = car_to_world_ * camera_to_car_;
  camera_to_world_adj_ = car_to_world_;
  ADEBUG << "get " << "camera" << " to "
         << "world" << " trans: " << camera_to_world_;
  //20210416

  converter_->Convert(&objects);
  PERF_BLOCK_END("CameraProcessSubnode_converter_");

  //RemoveObj(objects);

  //if use yolo v2
  if(use_yolo_v5 == false){
    transformer_->SetExtrinsics(camera_to_world_);
    transformer_->Transform(&objects);
    //adjusted_extrinsics_ =
    //transformer_->GetAdjustedExtrinsics(&camera_to_world_adj_);
    PERF_BLOCK_END("CameraProcessSubnode_transformer_");

    tracker_->Associate(img, timestamp, &objects);
    PERF_BLOCK_END("CameraProcessSubnode_tracker_");

    filter_->Filter(timestamp, &objects);
    PERF_BLOCK_END("CameraProcessSubnode_filter_");
  
    //auto ccm = Singleton<CalibrationConfigManager>::get();
    //auto calibrator = ccm->get_camera_calibration();
    //calibrator->SetCar2CameraExtrinsicsAdj(camera_to_car_adj_,
                                         //adjusted_extrinsics_);
    out_objs->timestamp = timestamp;
    VisualObjToSensorObj(objects, &out_objs);
  }

  //if use yolo v5
  if(use_yolo_v5 == true){
    ADEBUG << "USE_YOLO_V5" ;
    for (const auto &obj : objects){
      //obj on the right,heading left
      if(obj->center.x()>1){
        obj->direction = Eigen::Vector3f(-1.0, 0.0, 0.0);
        obj->theta = 3.1416;
        obj->length = obj->width;
        obj->width = 1.5;
        obj->center.x() = obj->center.x()-0.25/*+(0.05*obj->center.y())*/;
        obj->center.y() = obj->center.y()+(obj->width)/2;
        obj->center.z() = obj->height/2;
      }
      //obj on the left,heading right
      if(obj->center.x()<-3){
        obj->direction = Eigen::Vector3f(1.0, 0.0, 0.0);
        obj->theta = 0;
        obj->length = obj->width;
        obj->width = 1.5;
        obj->center.x() = obj->center.x()+0.25/*+(0.05*obj->center.y())*/;
        obj->center.y() = obj->center.y()+(obj->width)/2;
        obj->center.z() = obj->height/2;
      }
      //obj on the road,heading front
      if(obj->center.x() >= -3 && obj->center.x() <= 1){
        obj->direction = Eigen::Vector3f(0.0, 1.0, 0.0);
        obj->theta = 1.5708;
        obj->center.x() = obj->center.x()/*+(0.05*obj->center.y())*/;
        obj->center.y() =obj->center.y()+(obj->length/2);
        obj->center.z() = obj->height/2;
      }
    
    }

    int x_min = 0;
    int y_min = 0;
    int x_max = 0;
    int y_max = 0;
    for(size_t i = 0;i < objects.size();++i){
      x_min = objects[i]-> upper_left.x();
      y_min = objects[i]-> upper_left.y();
      x_max = objects[i]-> lower_right.x();
      y_max = objects[i]-> lower_right.y();
      cv::rectangle(result_vis,cvPoint(x_min,y_min),cvPoint(x_max,y_max),cv::Scalar(0,255,255),1,1,0);
      //cv::putText(result_vis,GetObjectName(objects[i]->type),cvPoint(x_min,y_min),cv::FONT_HERSHEY_PLAIN,3,cvScalar(0,255,255),2,8,false);
      //cv::putText(result_vis,std::to_string(objects[i]-> id),cvPoint(x_min,y_min),cv::FONT_HERSHEY_PLAIN,3,cvScalar(0,255,255),2,8,false);
      cv::putText(result_vis,"y:"+std::to_string(converter_->get_real_distance_y(i)),cvPoint(x_min,y_min),cv::FONT_HERSHEY_PLAIN,3,cvScalar(0,255,255),2,8,false);
      cv::putText(result_vis,"y_min:"+std::to_string(y_min),cvPoint(x_max,y_max),cv::FONT_HERSHEY_PLAIN,3,cvScalar(0,255,255),2,8,false);
    }
    /*
    if (result_vis.data != NULL){
      //show the image
	    ADEBUG << "start_image_show!";
      cv::Size dsize=cv::Size(960,540);
      cv::Mat shrink;
      cv::resize(result_vis,shrink,dsize,0,0,cv::INTER_AREA);
      cv::imshow("camera_obstacle_detector", shrink);
      // imshow之后必须有waitKey函数，否则显示窗内将一闪而过，不会驻留屏幕
      cv::waitKey(1);
    }*/
    cv::string out_path;
    out_path = "obtest/result_vis_"+std::to_string(count)+".jpg";
    cv::imwrite(out_path,result_vis);

    RemoveObj(objects);

    transformer_->SetExtrinsics(camera_to_world_);
    transformer_->Transform(&objects);
    PERF_BLOCK_END("CameraProcessSubnode_transformer_");

    tracker_->Associate(img, timestamp, &objects);
    PERF_BLOCK_END("CameraProcessSubnode_tracker_");

    filter_->Filter(timestamp, &objects);
    PERF_BLOCK_END("CameraProcessSubnode_filter_");

    out_objs->timestamp = timestamp;
    VisualObjToSensorObj(objects, &out_objs);
  }

  SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
  //camera_item_ptr->image_src_mat = img.clone();
  //mask.copyTo(out_objs->camera_frame_supplement->lane_map);
  //PublishDataAndEvent(timestamp, out_objs, camera_item_ptr);
  PERF_BLOCK_END("CameraProcessSubnode publish in DAG");
  ADEBUG << "camera process succ, there are " << (out_objs->objects).size()
         << " objects.";
  
  //sensor_msgs::Image pub_msg;
  //MatToMessage(result_vis,&pub_msg);
  //AdapterManager::PublishImageShort(pub_msg);
  PublishPerceptionPbObj(out_objs);
  count++;
  //20210201
}

void CameraProcessSubnode::RemoveObj(std::vector<std::shared_ptr<VisualObject>> &objects){

  std::vector<std::shared_ptr<VisualObject>>::iterator obj;
  int i = 0;
  for(obj = objects.begin();obj != objects.end();){
    if(objects[i] -> center.y() >= 20 /*|| objects[i] -> center.x() >= 3.5 || objects[i] -> center.x() <= -4 */
    ||objects[i] -> width <= 0 || objects[i] -> length <= 0){
      obj = objects.erase(obj);
      ADEBUG << "Remove 1 object! Size of objects is"<<objects.size() ;
    }
    /*if(objects[i] -> center.x() >= 0 && objects[i] -> center.x() <= 2){
      obj = objects.erase(obj);
      ADEBUG << "Remove 1 object! Size of objects is"<<objects.size();
    }*/
    else{
      ++obj;
      i++;
    }
  }
}

void CameraProcessSubnode::ChassisCallback(
    const jmc_auto::canbus::Chassis &message) {
  
}

bool CameraProcessSubnode::MessageToMat(const sensor_msgs::Image &msg,
                                        cv::Mat *img) {
  *img = cv::Mat(msg.height, msg.width, CV_8UC3);
  int pixel_num = msg.width * msg.height;
  if (msg.encoding.compare("yuyv") == 0) {
    unsigned char *yuv = (unsigned char *)&(msg.data[0]);
    yuyv2bgr(yuv, img->data, pixel_num);
  } else {
    cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    *img = cv_ptr->image;
  }

  return true;
}

bool CameraProcessSubnode::MatToMessage(const cv::Mat& img,
                                        sensor_msgs::Image *msg) {
  if (img.type() == CV_8UC1) {
    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8,
                           img.rows, img.cols,
                           static_cast<unsigned int>(img.step), img.data);
    return true;
  } else if (img.type() == CV_32FC1) {
    cv::Mat uc_img(img.rows, img.cols, CV_8UC1);
    uc_img.setTo(cv::Scalar(0));
    for (int h = 0; h < uc_img.rows; ++h) {
      for (int w = 0; w < uc_img.cols; ++w) {
        if (img.at<float>(h, w) >= ln_msk_threshold_) {
          uc_img.at<unsigned char>(h, w) = 1;
        }
      }
    }

    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8,
                           uc_img.rows, uc_img.cols,
                           static_cast<unsigned int>(uc_img.step), uc_img.data);
    return true;
  } else {
    AERROR << "invalid input Mat type: " << img.type();
    return false;
  }
}

void CameraProcessSubnode::VisualObjToSensorObj(
    const std::vector<std::shared_ptr<VisualObject>> &objects,
    SharedDataPtr<SensorObjects> *sensor_objects) {
  (*sensor_objects)->sensor_type = SensorType::CAMERA;
  (*sensor_objects)->sensor_id = device_id_;
  (*sensor_objects)->seq_num = seq_num_;

  (*sensor_objects)->sensor2world_pose_static = camera_to_world_;
  (*sensor_objects)->sensor2world_pose = camera_to_world_adj_;

  ((*sensor_objects)->camera_frame_supplement).reset(new CameraFrameSupplement);

  if (!CameraFrameSupplement::state_vars.initialized_) {
    CameraFrameSupplement::state_vars.process_noise *= 10;
    CameraFrameSupplement::state_vars.trans_matrix.block(0, 0, 1, 4) << 1.0f,
        0.0f, 0.33f, 0.0f;
    CameraFrameSupplement::state_vars.trans_matrix.block(1, 0, 1, 4) << 0.0f,
        1.0f, 0.0f, 0.33f;
    ADEBUG << "state trans matrix in CameraFrameSupplement is \n"
           << CameraFrameSupplement::state_vars.trans_matrix << std::endl;
    CameraFrameSupplement::state_vars.initialized_ = true;
  }

  for (size_t i = 0; i < objects.size(); ++i) {
    std::shared_ptr<VisualObject> vobj = objects[i];
    std::shared_ptr<Object> obj(new Object());

    obj->id = vobj->id;
    obj->score = vobj->score;
    obj->direction = vobj->direction.cast<double>();
    obj->theta = vobj->theta;
    obj->center = vobj->center.cast<double>();
    obj->length = vobj->length;
    obj->width = vobj->width;
    obj->height = vobj->height;
    obj->type = vobj->type;
    obj->track_id = vobj->track_id;
    obj->tracking_time = vobj->track_age;
    obj->latest_tracked_time = vobj->last_track_timestamp;
    obj->velocity = vobj->velocity.cast<double>();
    obj->anchor_point = obj->center;
    obj->state_uncertainty = vobj->state_uncertainty;

    (obj->camera_supplement).reset(new CameraSupplement());
    obj->camera_supplement->upper_left = vobj->upper_left.cast<double>();
    obj->camera_supplement->lower_right = vobj->lower_right.cast<double>();
    obj->camera_supplement->alpha = vobj->alpha;
    obj->camera_supplement->pts8 = vobj->pts8;

    ((*sensor_objects)->objects).emplace_back(obj);
  }
}

void CameraProcessSubnode::PublishDataAndEvent(
    const double timestamp, const SharedDataPtr<SensorObjects> &sensor_objects,
    const SharedDataPtr<CameraItem> &camera_item) {
  CommonSharedDataKey key(timestamp, device_id_);
  cam_obj_data_->Add(key, sensor_objects);
  //cam_shared_data_->Add(key, camera_item);

  for (size_t idx = 0; idx < pub_meta_events_.size(); ++idx) {
    const EventMeta &event_meta = pub_meta_events_[idx];
    Event event;
    event.event_id = event_meta.event_id;
    event.timestamp = timestamp;
    event.reserve = device_id_;
    event_manager_->Publish(event);
  }
}

void CameraProcessSubnode::PublishPerceptionPbObj(
    const SharedDataPtr<SensorObjects> &sensor_objects) {
  PerceptionObstacles obstacles;

  // Header
  AdapterManager::FillPerceptionObstaclesHeader(
      "perception_obstacle", &obstacles);
  common::Header *header = obstacles.mutable_header();
  header->set_lidar_timestamp(0);
  header->set_camera_timestamp(timestamp_ns_);
  header->set_radar_timestamp(0);
  obstacles.set_error_code(sensor_objects->error_code);

  // Serialize each Object
  for (const auto &obj : sensor_objects->objects) {
    PerceptionObstacle *obstacle = obstacles.add_perception_obstacle();
    obj->Serialize(obstacle);
  }

  // Relative speed of objects + latest ego car speed in X
  for (auto obstacle : obstacles.perception_obstacle()) {
    obstacle.mutable_velocity()->set_x(obstacle.velocity().x() +
                                       chassis_.speed_mps());
    ADEBUG << "obstacle_type:" << obstacle.type();
  }

  AdapterManager::PublishPerceptionObstacles(obstacles);
  ADEBUG << "PublishPerceptionObstacles: " << obstacles.ShortDebugString();
  if(obstacles.perception_obstacle().size() > 0){
    ADEBUG << "Time Check";
  } 
}

void CameraProcessSubnode::PublishPerceptionPbLnMsk(
  const cv::Mat& mask, const sensor_msgs::Image &message) {
  sensor_msgs::Image lane_mask_msg;
  lane_mask_msg.header = message.header;
  lane_mask_msg.header.frame_id = "lane_mask";
  MatToMessage(mask, &lane_mask_msg);

  AdapterManager::PublishPerceptionLaneMask(lane_mask_msg);
  ADEBUG << "PublishPerceptionLaneMask";
}

}  // namespace perception
}  // namespace jmc_auto
