#include "modules/perception/obstacle/onboard/roundview_camera_subnode.h"
#include "modules/perception/onboard/transform_input.h"
#include "eigen_conversions/eigen_msg.h"
#include "ros/include/ros/ros.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/onboard/hdmap_input.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_avm.h"

namespace jmc_auto {
namespace perception {

using jmc_auto::common::adapter::AdapterManager;

bool RoundviewCameraSubnode::InitInternal() {
  // Subnode config in DAG streaming
  std::unordered_map<std::string, std::string> fields;
  SubnodeHelper::ParseReserveField(reserve_, &fields);

  if (fields.count("device_id")) device_id_ = fields["device_id"];

  // Shared Data
  cam_obj_data_ = static_cast<CameraObjectData *>(
      shared_data_manager_->GetSharedData("CameraObjectData"));
  // cam_shared_data_ = static_cast<CameraSharedData *>(
  //     shared_data_manager_->GetSharedData("CameraSharedData"));

  InitCalibration();

  InitModules();

  AdapterManager::AddImageRoundviewCallback(&RoundviewCameraSubnode::ImgCallback,
                                        this);
  AdapterManager::AddChassisCallback(&RoundviewCameraSubnode::ChassisCallback,
                                       this);
  //timer_ = AdapterManager::CreateTimer(ros::Duration(1.0 / 20.0), &RoundviewCameraSubnode::OnTimer, this);
  
  ADEBUG<< "ROUNDVIEW_CAMMERA INIT SUCC.";
  return true;
}

bool RoundviewCameraSubnode::InitCalibration() {
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

bool RoundviewCameraSubnode::InitModules() {
  RegisterFactoryJmcCaffeDetector();
  RegisterFactoryFlatCameraTransformer();
  RegisterFactoryJmcCameraTracker();
  RegisterFactoryCascadedCameraTracker();
  RegisterFactoryObjectCameraFilter();

  detector_.reset(
      BaseCameraDetectorRegisterer::GetInstanceByName("JmcCaffeDetector"));
  detector_->Init();

  transformer_.reset(BaseCameraTransformerRegisterer::GetInstanceByName(
      "FlatCameraTransformer"));
  transformer_->Init();
  transformer_->SetExtrinsics(camera_to_car_);

  tracker_.reset(
      BaseCameraTrackerRegisterer::GetInstanceByName("CascadedCameraTracker"));
  tracker_->Init();
  filter_.reset(
      BaseCameraFilterRegisterer::GetInstanceByName("ObjectCameraFilter"));
  filter_->Init();


  avm_inst_.reset(new jmc_auto::perception::JmcAvm());
  avm_inst_->Init();

  return true;
}

void RoundviewCameraSubnode::ImgCallback(const sensor_msgs::Image &message) {
  double timestamp = message.header.stamp.toSec();
  ADEBUG << "RoundviewCameraSubnode ImgCallback: timestamp: ";
  ADEBUG << std::fixed << std::setprecision(64) << timestamp;
  AINFO << "camera received image : " << GLOG_TIMESTAMP(timestamp)
        << " at time: " << GLOG_TIMESTAMP(TimeUtil::GetCurrentTime());
  double curr_timestamp = timestamp * 1e9;

  if (FLAGS_skip_camera_frame && timestamp_ns_ > 0.0) {
    if ((curr_timestamp - timestamp_ns_) < (1e9 / FLAGS_camera_hz) &&
        curr_timestamp > timestamp_ns_) {
      ADEBUG << "RoundviewCameraSubnode Skip frame";
      return;
    }
  }

  timestamp_ns_ = curr_timestamp;
  ADEBUG << "RoundviewCameraSubnode Process: "
         << " frame: " << ++seq_num_;
  PERF_FUNCTION("RoundviewCameraSubnode");
  PERF_BLOCK_START();

  cv::Mat img;
  cv::Mat result_vis;
  if (!FLAGS_image_file_debug) {
    MessageToMat(message, &img);
  } else {
    img = cv::imread(FLAGS_image_file_path, CV_LOAD_IMAGE_COLOR);
  }
  cv::resize(img, img, cv::Size(1920, 1080), 0, 0);
  result_vis = img.clone();
  std::vector<std::shared_ptr<VisualObject>> objects;
  cv::Mat mask;

  PERF_BLOCK_END("RoundviewCameraSubnode_Image_Preprocess");
  std::shared_ptr<SensorObjects> out_objs(new SensorObjects);
  detector_->Multitask(img, CameraDetectorOptions(), &objects, &mask);
  mask = mask*2;
  
  PERF_BLOCK_END("RoundviewCameraSubnode_detector_");
  
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

  for (const auto &obj : objects){
      Eigen::Vector2f mid_point;
      mid_point.x() = (obj->lower_right.x()+obj->upper_left.x())/2;
      mid_point.y() = obj->lower_right.y();
    if (mid_point.x() > 1340 && mid_point.x() < 1540 && mid_point.y() < 540){
      ADEBUG<<"INDEX:RIGHT";
      avm_inst_->img2ground(mid_point,obj->center);
      obj->center.y() = obj->center.x()/1000;
      obj->center.x() = obj->center.z()/1000;
      if(obj->type == ObjectType::CAR){
      obj->length = 4.5;
      obj->width = 1.5;
      obj->height = 1.5;
      }
      if(obj->type == ObjectType::PEDESTRIAN){
      obj->length = 0.5;
      obj->width = 0.5;
      obj->height = 1.7;
      }
      obj->direction = Eigen::Vector3f(-1.0, 0.0, 0.0);
      obj->theta = 3.1416;
      obj->center.x() = obj->center.x()+(obj->length)/2-1.2/*+(0.05*obj->center.y())*/;
      obj->center.y() = obj->center.y()+1.3;
      obj->center.z() = obj->height/2;  
    }
    
    else if(chassis_.gear_location() == 2){
      ADEBUG<<"gear_position:R";
      if (mid_point.x() > 1340 && mid_point.x() < 1540 && mid_point.y() > 540){
      ADEBUG<<"INDEX:REAR";
      avm_inst_->img2ground(mid_point,obj->center);
      obj->center.y() = obj->center.x()/1000;
      obj->center.x() = obj->center.z()/1000;
      if(obj->type == ObjectType::CAR){
      obj->length = 4.5;
      obj->width = 1.5;
      obj->height = 1.5;
      }
      if(obj->type == ObjectType::PEDESTRIAN){
      obj->length = 0.5;
      obj->width = 0.5;
      obj->height = 1.7;
      }
      obj->direction = Eigen::Vector3f(0.0, -1.0, 0.0);
      obj->theta = 1.5708;
      obj->center.x() = obj->center.x();
      obj->center.y() = obj->center.y() - (obj->length)/2 +1.3;
      obj->center.z() = obj->height/2;  
      }
    }

    else{
      obj->center.x() = 0;
      obj->center.y() = 0;
      obj->center.z() = 0;
    }  
  
  }
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

  SharedDataPtr<CameraItem> camera_item_ptr(new CameraItem);
  //camera_item_ptr->image_src_mat = img.clone();
  //mask.copyTo(out_objs->camera_frame_supplement->lane_map);
  //PublishDataAndEvent(timestamp, out_objs, camera_item_ptr);
  PERF_BLOCK_END("CameraProcessSubnode publish in DAG");
  ADEBUG << "camera process succ, there are " << (out_objs->objects).size()
         << " objects.";
  //20210201 - Add Image Visualization
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
    cv::putText(result_vis,GetObjectName(objects[i]->type),cvPoint(x_min,y_min),cv::FONT_HERSHEY_PLAIN,3,cvScalar(0,255,255),2,8,false);
    cv::putText(result_vis,std::to_string(objects[i]-> id),cvPoint(x_min,y_min),cv::FONT_HERSHEY_PLAIN,3,cvScalar(0,255,255),2,8,false);
  }
  cv::string out_path;
  out_path = "test_result/result_vis_"+std::to_string(count)+".jpg";
  cv::imwrite(out_path,result_vis);
  //sensor_msgs::Image pub_msg;
  //MatToMessage(result_vis,&pub_msg);
  //AdapterManager::PublishImageShort(pub_msg);
  PublishPerceptionPbObj(out_objs);
  count++;
}

void RoundviewCameraSubnode::ChassisCallback(
    const jmc_auto::canbus::Chassis &message) {
  chassis_ = message;
}

/*void RoundviewCameraSubnode::OnTimer(const ros::TimerEvent&) {
      auto chassis_adapter = AdapterManager::GetChassis();  
      if (chassis_adapter->Empty()){
        AINFO<< "not chassis msg";
        return;
      }  
      chassis_ = chassis_adapter->GetLatestObserved();  
}*/

void RoundviewCameraSubnode::RemoveObj(std::vector<std::shared_ptr<VisualObject>> &objects){

  std::vector<std::shared_ptr<VisualObject>>::iterator obj;
  int i = 0;
  for(obj = objects.begin();obj != objects.end();){
    if(objects[i] -> center.x() == 0 || objects[i] -> center.x() >= 8){
      obj = objects.erase(obj);
      ADEBUG << "Remove 1 object! Size of objects is"<<objects.size() ;
    }
    /*if(objects[i] -> center.x() >= 6 && objects[i] -> center.x() <= 2){
      obj = objects.erase(obj);
      ADEBUG << "Remove 1 object! Size of objects is"<<objects.size();
    }*/
    else{
      ++obj;
      i++;
    }
  }
}

bool RoundviewCameraSubnode::MessageToMat(const sensor_msgs::Image &msg,
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

bool RoundviewCameraSubnode::MatToMessage(const cv::Mat& img,
                                        sensor_msgs::Image *msg) {
  if (img.type() == CV_8UC1) {
    sensor_msgs::fillImage(*msg, sensor_msgs::image_encodings::MONO8,
                           img.rows, img.cols,
                           static_cast<unsigned int>(img.step), img.data);
    return true;
  } else {
    AERROR << "invalid input Mat type: " << img.type();
    return false;
  }
}

void RoundviewCameraSubnode::VisualObjToSensorObj(
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
    ADEBUG<<"QQQ:"<<obj->center.z();
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

void RoundviewCameraSubnode::PublishDataAndEvent(
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

void RoundviewCameraSubnode::PublishPerceptionPbObj(
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

  AdapterManager::PublishPerceptionRoundviewObstacles(obstacles);
  ADEBUG << "PublishPerceptionObstacles: " << obstacles.ShortDebugString();
  if(obstacles.perception_obstacle().size() > 0){
    ADEBUG << "Time Check";
  } 
}

}  // namespace perception
}  // namespace jmc_auto
