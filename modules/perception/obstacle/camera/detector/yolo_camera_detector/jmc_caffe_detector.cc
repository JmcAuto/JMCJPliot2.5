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

#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_caffe_detector.h"

#include <cmath>
#include <unordered_map>
#include <utility>
#include <ctime>

#include "modules/perception/obstacle/camera/detector/common/proto/tracking_feature.pb.h"

//#include "modules/common/util/file.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/camera/common/util.h"
#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/util.h"
namespace jmc_auto {
namespace perception {
    #if 0
    using namespace cv;
    using namespace std;
    using namespace caffe;
    using std::string;

    using caffe::Blob;
    using caffe::Caffe;
    using caffe::Layer;
    using caffe::Net;
    using caffe::shared_ptr;
    using caffe::string;
    using caffe::vector;
    using std::cout;
    using std::endl;
    using std::ostringstream;

    using jmc_auto::common::util::GetProtoFromFile;
    using std::string;
    using std::unordered_map;
    using std::vector;
    #endif
  
    std::string JmcCaffeDetector::Name() const
    {
        return std::string("jmc_caffe_detector");
    }

	bool JmcCaffeDetector::Init(const CameraDetectorInitOptions &options) 
	{
		bool ret = true;
        if(detect_inst_)
        {
            detect_inst_->Init();
        }
		return ret;
	}

	bool JmcCaffeDetector::Detect(const cv::Mat &frame, const CameraDetectorOptions &options,
		      std::vector<std::shared_ptr<VisualObject>> *objects) 
	{
		bool ret = true;
  
        std::vector<Boundingbox> bboxes;

        if(objects && detect_inst_)
        {
            bboxes.clear();
            if(detect_inst_->Detect(frame,bboxes))
            {
                /*
                cvt bboxes to objects
                */
                caffe::Timer cvt_time;
                cvt_time.Start();
                detect_inst_->cvtToVisualObject(bboxes,objects);
                cvt_time.Stop();
                AINFO << "Running cvtToVisualObject: " << cvt_time.MilliSeconds() << " ms";
                AINFO << "object cout: " << bboxes.size() << " output cout: "<< objects->size();
            }
            else
            {
                AERROR << "Detect failed.";
                ret = false;
            }
        }
        else
        {
          AERROR << "'objects' or 'detect_inst_' is a null pointer.";
          ret = false;
        }
    
		return ret;
	}

	bool JmcCaffeDetector::Multitask(const cv::Mat &frame, const CameraDetectorOptions &options,
			 std::vector<std::shared_ptr<VisualObject>> *objects,
			 cv::Mat *mask)
    {
		bool ret = true;
        if(objects)
        {
            Detect(frame,options,objects);
        }
        else
        {
            AERROR << "'objects' is a null pointer.";
            ret = false;
        }
		return ret;
	}

	bool JmcCaffeDetector::Lanetask(const cv::Mat &frame, cv::Mat *mask)
	{
		bool ret = true;

		return ret;
	}

    void JmcCaffeDetector::RenderVisualObject(cv::Mat &image, std::vector<std::shared_ptr<VisualObject>> *objects)
    {
        for(auto obj:*objects)
        {
           int xmin = obj->upper_left.x();
           int ymin = obj->upper_left.y();
           int xmax = obj->lower_right.x();
           int ymax = obj->lower_right.y();
           int type = static_cast<int>(obj->type);
           float score = obj->type_probs[type];
           #if 0
           float alpha = obj->alpha;
           float w = obj->width;
           float h = obj->height;
           float l = obj->length;
           float cx = obj->center.x();
           float cy = obj->center.y();
           float cz = obj->center.z();
           #endif
           char text[64]={0};
           if(xmin < 0){
               xmin = 0;
           }
           if(ymin < 0){
               ymin = 0;
           }
           if(xmax > image.cols){
               xmax = image.cols - 1;
           }
           if(ymax > image.rows){
               ymax = image.rows - 1;
           }
           //std::cout <<"alpha: " << alpha << std::endl;
           //std::cout <<"w: " << w <<"h: " << h << "l: "<< l<< std::endl;
           //std::cout <<"alpha: " << alpha << std::endl;
           //sprintf(text,"type:%d_s:%.3f_a:%.2f_s(%.1fx%.1fx%.1f)_l(%.1fx%.1fx%.1f)",type,score,alpha,w,h,l,cx,cy,cz);
           sprintf(text,"type:%d_s:%.3f",type,score);
           //cv::rectangle(image, cv::Point(xmin, ymin), cv::Point(xmax, ymax), cv::Scalar(255, 200,0), 1);
           //cv::putText(image, std::to_string(score), cv::Point(xmin, ymin), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,204,255));
           cv::putText(image, std::string(text), cv::Point(xmin, ymin), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,200,255));
            #if 0
           cv::rectangle(image,cv::Point(obj->front_upper_left.x(), obj->front_upper_left.y())
                         ,cv::Point(obj->front_lower_right.x(), obj->front_lower_right.y())
                         , cv::Scalar(0, 0,255), 2);

           cv::rectangle(image,cv::Point(obj->back_upper_left.x(), obj->back_upper_left.y())
                         ,cv::Point(obj->back_lower_right.x(), obj->back_lower_right.y())
                         , cv::Scalar(0, 0,255), 2);

           cv::line(image,cv::Point(obj->front_upper_left.x(), obj->front_upper_left.y())
                         ,cv::Point(obj->back_upper_left.x(), obj->back_upper_left.y())
                         , cv::Scalar(0, 255,255), 1);

           cv::line(image,cv::Point(obj->front_upper_left.x(), obj->front_lower_right.y())
                         ,cv::Point(obj->back_upper_left.x(), obj->back_lower_right.y())
                         , cv::Scalar(0, 255,255), 1);

           cv::line(image,cv::Point(obj->front_lower_right.x(), obj->front_upper_left.y())
                        ,cv::Point(obj->back_lower_right.x(), obj->back_upper_left.y())
                        , cv::Scalar(0, 255,255), 1);

           cv::line(image,cv::Point(obj->front_lower_right.x(), obj->front_lower_right.y())
                        ,cv::Point(obj->back_lower_right.x(), obj->back_lower_right.y())
                        , cv::Scalar(0, 255,255), 1);
          #endif
            #if ENABLE_3D_BBOX
           for(int k = 0; k < 3; k++)
            {
               cv::Point pf0(obj->pts8[4 * k],obj->pts8[4 * k + 1]);
               cv::Point pf1(obj->pts8[4 * (k+1)],obj->pts8[4 * (k+1) + 1]);

               cv::Point pr0(obj->pts8[4 * k + 2],obj->pts8[4 * k + 3]);
               cv::Point pr1(obj->pts8[4 * (k+1) + 2],obj->pts8[4 * (k+1) + 3]);

               cv::line(image,pf0,pf1,cv::Scalar(0, 0,255), 1);
               cv::line(image,pr0,pr1,cv::Scalar(0, 255,255), 1);
               cv::line(image,pf0,pr0,cv::Scalar(0, 255,0), 1);
               cv::line(image,pf1,pr1,cv::Scalar(0, 255,0), 1);
               if(k == 0)
               {
                   cv::Point pf2(obj->pts8[4 * (k+3)],obj->pts8[4 * (k+3) + 1]);
                   cv::Point pr2(obj->pts8[4 * (k+3) + 2],obj->pts8[4 * (k+3) + 3]);
                   cv::line(image,pf0,pf2,cv::Scalar(0, 0,255), 1);
                   cv::line(image,pr0,pr2,cv::Scalar(0, 255,255), 1);

                   //cv::line(image,pf2,pr2,cv::Scalar(0, 255,0), 1);
               }

           }
            #else
           cv::rectangle(image, cv::Point(xmin, ymin), cv::Point(xmax, ymax), cv::Scalar(255, 200,0), 1);
           #endif
        }
    }
}
}
