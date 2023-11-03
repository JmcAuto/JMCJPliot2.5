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
#define JMC_DET_SOLO_TEST 0 // 0: jmcauto 1: solo test

#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_object_detector.h"

#include <cmath>
#include <unordered_map>
#include <utility>
#include <ctime>

namespace jmc_auto {
namespace perception {

    #if JMC_DET_SOLO_TEST  
    //static std::string prototxt_path = "./jmc_model/jmc_v0.0.2.prototxt";
    //static std::string weight_file = "./jmc_model/jmc_v0.0.2.caffemodel";
    //static std::string prototxt_path = "./jmc_model/jmc_v0.0.5_kt.prototxt";
    //static std::string weight_file = "./jmc_model/jmc_v0.0.5_kt.caffemodel";
    static std::string prototxt_path = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/jmc_v0.0.3.prototxt";
    static std::string weight_file = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/jmc_v0.0.3.caffemodel";
    #else
    static std::string prototxt_path = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/jmc_v0.0.3.prototxt";
    static std::string weight_file = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/jmc_v0.0.3.caffemodel";
    #endif

    bool JmcObjectDetector::SetConfigFilePath(const std::string &prototxt_file, const std::string &weight_file, bool is_user)
    {
        prototxt_file_path_ = prototxt_file;
        weight_file_path_ = weight_file;
        if(is_user)
        {
            config_cout_++;
        }
        return true;
    }

    bool JmcObjectDetector::Init() 
    {
        bool ret = true;
        //::google::InitGoogleLogging("caffe"); //初始化日志文件,不调用会给出警告,但不会报错
        caffe::Caffe::set_mode(caffe::Caffe::GPU);
        //caffe::Caffe::set_solver_rank(1); //不进行日志输出
        // init Net
        if(config_cout_ == 0)
        {
            // use default file 
            SetConfigFilePath(prototxt_path,weight_file,false);
            //std::cout << config_cout_ << std::endl;
        }

        net_.reset(new caffe::Net<float>(prototxt_file_path_, caffe::TEST));
        CHECK((net_ != nullptr));
        net_->CopyTrainedLayersFrom(weight_file_path_);

        input_layer_ = net_->input_blobs()[0];

        return ret;
    }

    bool JmcObjectDetector::Detect(const cv::Mat &frame, std::vector<Boundingbox> &bboxes) 
    {
        bool ret = true;
        //caffe::Caffe::SetDevice(caffe::Caffe::GPU);
        caffe::Timer pre_time;
        pre_time.Start();
        if(net_ && frame.data != nullptr)
        {
            // 图片预处理,并加载图片进入blob
            if(input_layer_)
            {
                float *input_data = input_layer_->mutable_cpu_data();
                cv::Mat pre_img = PreprocessImg(frame);
                //cv::imshow("pre_img",pre_img);
                if(data_)
                {
                    #if 1
                    int i = 0;
                    for (int row = 0; row < input_height_; ++row)
                    {
                        uchar* uc_pixel = pre_img.data + row * pre_img.step;
                        for (int col = 0; col < input_width_; ++col) 
                        {
                        #if 1
                            data_[i] = (float)uc_pixel[2] / 255.0;
                            data_[i + input_height_ * input_width_] = (float)uc_pixel[1] / 255.0;
                            data_[i + 2 * input_height_ * input_width_] = (float)uc_pixel[0] / 255.0;
                        #else
                            data_[i] = (float)uc_pixel[0]/ 255.0;
                            data_[i + input_height_ * input_width_] = (float)uc_pixel[1]/ 255.0;
                            data_[i + 2 * input_height_ * input_width_] = (float)uc_pixel[2]/ 255.0;
                        #endif
                            uc_pixel += 3;
                            ++i;
                        }
                    }
                    #else                  
                    cv::Mat inf;
                    cv::cvtColor(pre_img,pre_img,cv::COLOR_BGR2RGB);
                    pre_img.convertTo(inf,CV_32FC3,1.0/255.0);
                    #endif
                    memcpy((float *) (input_data),
                            data_, 3 * input_width_ * input_height_ * sizeof(float));
                }
                else
                {
                    std::cout << "'data_' is a null pointer."<< std::endl;
                    ret = false;
                }    

            }
            else
            {
                std::cout << "'input_layer_' is a null pointer."<< std::endl;
                ret = false;
            }  
        }
        else
        {
            std::cout << "'frame.data' or net is a null pointer."<< std::endl;
            ret = false;
        }
        pre_time.Stop();
        //std::cout << "Pre-processing: " << pre_time.MilliSeconds() << " ms"<< std::endl;

        if(ret)
        {
            caffe::Timer det_time;
            det_time.Start();

            net_->Forward();

            det_time.Stop();
            //std::cout << "Running detection object: " << det_time.MilliSeconds() << " ms"<< std::endl;

            caffe::Timer post_time;
            post_time.Start();
            
            caffe::Blob<float> *output_layer0 = net_->output_blobs()[2];
            const float *output0 = output_layer0->cpu_data();
            std::cout << "output shape: " << output_layer0->shape(0) << " " << output_layer0->shape(1) << " " <<  output_layer0->shape(2) << " " <<  output_layer0->shape(3) <<  " " <<  output_layer0->shape(4) << std::endl;
            
            caffe::Blob<float> *output_layer1 = net_->output_blobs()[0];
            const float *output1 = output_layer1->cpu_data();
            
            caffe::Blob<float> *output_layer2 = net_->output_blobs()[1];
            const float *output2 = output_layer2->cpu_data();
            
            std::vector<float *> cur_output_tensors;
            cur_output_tensors.push_back(const_cast<float *>(output0));
            cur_output_tensors.push_back(const_cast<float *>(output1));
            cur_output_tensors.push_back(const_cast<float *>(output2));          
            bboxes = PostProcess(cur_output_tensors, conf_thresh_, nms_thresh_);       
            Transform(frame.rows, frame.cols, input_height_, input_width_, bboxes, true);
        }

        return ret;
    }

    std::vector<Anchor> JmcObjectDetector::InitAnchors()
    {
        std::vector<Anchor> anchors;
        Anchor anchor;

        // 10,13, 16,30, 33,23, 30,61, 62,45, 59,119, 116,90,  156,198,  373,326
        anchor.width = 10;
        anchor.height = 13;
        anchors.emplace_back(anchor);
        anchor.width = 16;
        anchor.height = 30;
        anchors.emplace_back(anchor);
        anchor.width = 32;
        anchor.height = 23;
        anchors.emplace_back(anchor);
        anchor.width = 30;
        anchor.height = 61;
        anchors.emplace_back(anchor);
        anchor.width = 62;
        anchor.height = 45;
        anchors.emplace_back(anchor);
        anchor.width = 59;
        anchor.height = 119;
        anchors.emplace_back(anchor);
        anchor.width = 116;
        anchor.height = 90;
        anchors.emplace_back(anchor);
        anchor.width = 156;
        anchor.height = 198;
        anchors.emplace_back(anchor);
        anchor.width = 373;
        anchor.height = 326;
        anchors.emplace_back(anchor);
        return anchors;
    }

    void JmcObjectDetector::Transform(const int &ih, const int &iw, const int &oh, const int &ow, std::vector<Boundingbox> &bboxes,
                bool is_padding) 
    {
        if(is_padding)
        {
            float scale = std::min(static_cast<float>(ow) / static_cast<float>(iw), static_cast<float>(oh) / static_cast<float>(ih));
            int nh = static_cast<int>(scale * static_cast<float>(ih));
            int nw = static_cast<int>(scale * static_cast<float>(iw));
            int dh = (oh - nh) / 2;
            int dw = (ow - nw) / 2;
            for (auto &bbox : bboxes)
            {
                bbox.xmin = (bbox.xmin - dw) / scale;
                bbox.ymin = (bbox.ymin - dh) / scale;
                bbox.xmax = (bbox.xmax - dw) / scale;
                bbox.ymax = (bbox.ymax - dh) / scale;
            }
        }
        else
        {
            for (auto &bbox : bboxes)
            {
                bbox.xmin = bbox.xmin * iw / ow;
                bbox.ymin = bbox.ymin * ih / oh;
                bbox.xmax = bbox.xmax * iw / ow;
                bbox.ymax = bbox.ymax * ih / oh;
            }
        }
    }

    cv::Mat JmcObjectDetector::RenderBoundingBox(cv::Mat image, const std::vector<Boundingbox> &bboxes)
    {
        for (auto it: bboxes)
        {
            float score = it.score;
            cv::rectangle(image, cv::Point(it.xmin, it.ymin), cv::Point(it.xmax, it.ymax), cv::Scalar(255, 204,0), 3);
            cv::putText(image, std::to_string(score), cv::Point(it.xmin, it.ymin), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,204,255));
        }
        return image;
    }

    void JmcObjectDetector::Nmscpu(std::vector<Boundingbox> &bboxes, float threshold) 
    {
        if (bboxes.empty())
        {
            return ;
        }
        // 1.之前需要按照score排序
        std::sort(bboxes.begin(), bboxes.end(), [&](Boundingbox b1, Boundingbox b2){return b1.score>b2.score;});
        // 2.先求出所有bbox自己的大小
        std::vector<float> area(bboxes.size());
        for (int i = 0; i < (int)bboxes.size(); ++i)
        {
            area[i] = (bboxes[i].xmax - bboxes[i].xmin + 1) * (bboxes[i].ymax - bboxes[i].ymin + 1);
        }
        // 3.循环
        for (int i = 0; i < (int)bboxes.size(); ++i)
        {
            for (int j = i + 1; j < (int)bboxes.size(); )
            {
                float left = std::max(bboxes[i].xmin, bboxes[j].xmin);
                float right = std::min(bboxes[i].xmax, bboxes[j].xmax);
                float top = std::max(bboxes[i].ymin, bboxes[j].ymin);
                float bottom = std::min(bboxes[i].ymax, bboxes[j].ymax);
                float width = std::max(right - left + 1, 0.f);
                float height = std::max(bottom - top + 1, 0.f);
                float u_area = height * width;
                float iou = (u_area) / (area[i] + area[j] - u_area);
                if (iou>=threshold){
                    bboxes.erase(bboxes.begin()+j);
                    area.erase(area.begin()+j);
                }
                else
                {
                    ++j;
                }
            }
        }
    }

    void JmcObjectDetector::PostProcessParall(const int height, const int width, int scale_idx, float postThres, 
            float * origin_output, std::vector<int> Strides, std::vector<Anchor> Anchors, std::vector<Boundingbox> *bboxes)
    {
        Boundingbox bbox;
        float cx, cy, w_b, h_b, score;
        int cid;
        const float *ptr = (float *)origin_output;
        for(unsigned long a = 0; a < 3; ++a){
            for(unsigned long h = 0; h < (unsigned long)height; ++h){
                for(unsigned long w=0; w <(unsigned long)width; ++w){
                    const float *cls_ptr =  ptr + 5;
                    cid = argmax(cls_ptr, cls_ptr+num_class_);
                    score = sigmoid(ptr[4]) * sigmoid(cls_ptr[cid]);
                    if(score>=postThres){
                        cx = (sigmoid(ptr[0]) * 2.f - 0.5f + static_cast<float>(w)) * static_cast<float>(Strides[scale_idx]);
                        cy = (sigmoid(ptr[1]) * 2.f - 0.5f + static_cast<float>(h)) * static_cast<float>(Strides[scale_idx]);
                        w_b = powf(sigmoid(ptr[2]) * 2.f, 2) * Anchors[scale_idx * 3 + a].width;
                        h_b = powf(sigmoid(ptr[3]) * 2.f, 2) * Anchors[scale_idx * 3 + a].height;
                        bbox.xmin = clip(cx - w_b / 2, 0.F, static_cast<float>(input_width_ - 1));
                        bbox.ymin = clip(cy - h_b / 2, 0.f, static_cast<float>(input_height_ - 1));
                        bbox.xmax = clip(cx + w_b / 2, 0.f, static_cast<float>(input_width_ - 1));
                        bbox.ymax = clip(cy + h_b / 2, 0.f, static_cast<float>(input_height_ - 1));
                        bbox.score = score;
                        bbox.cid = cid;
                        //std::cout<< "bbox.cid : " << bbox.cid << std::endl;
                        bboxes->push_back(bbox);
                    }
                    ptr += 5 + num_class_;
                }
            }
        }
    }

    std::vector<Boundingbox> JmcObjectDetector::PostProcess(std::vector<float *> origin_output, float postThres, float nmsThres) 
    {
        std::vector<Anchor> Anchors = InitAnchors();
        std::vector<Boundingbox> bboxes;
        std::vector<int> Strides = std::vector<int> {8, 16, 32};
        for (int scale_idx=0; scale_idx<3; ++scale_idx) {
            const int stride = Strides[scale_idx];
            const int width = (input_width_ + stride - 1) / stride;
            const int height = (input_height_ + stride - 1) / stride;
            std::cout << "width : " << width << " " << "height : " << height << std::endl;
            float * cur_output_tensor = origin_output[scale_idx];
            PostProcessParall(height, width, scale_idx, postThres, cur_output_tensor, Strides, Anchors, &bboxes);
        }
        //std::cout << "bboxes size: " << bboxes.size();
        Nmscpu(bboxes, nmsThres);
        return bboxes;
    }
    cv::Mat JmcObjectDetector::PreprocessImg(const cv::Mat& img) 
    {
        int w, h, x, y;
        float r_w = input_width_ / (img.cols*1.0); // 640/1280 = 0.5
        float r_h = input_height_ / (img.rows*1.0); // 640/720 = 0.889
        if (r_h > r_w) {
            w = input_width_; // 640
            h = r_w * img.rows; // 720 * 0.5 = 360
            x = 0;
            y = (input_height_ - h) / 2; // (640-360)/2 = 90
        } else {
            w = r_h * img.cols;
            h = input_height_;
            x = (input_width_ - w) / 2;
            y = 0;
        }
        cv::Mat re(h, w, CV_8UC3); // 640*360*3
        cv::resize(img, re, re.size(), 0, 0, cv::INTER_LINEAR);
        //cv::Mat out(input_height_, input_width_, CV_8UC3, cv::Scalar(128, 128, 128));
        cv::Mat out(input_height_, input_width_, CV_8UC3, cv::Scalar(94, 95, 90));
        re.copyTo(out(cv::Rect(x, y, re.cols, re.rows)));
        return out;
    }
    bool JmcObjectDetector::cvtToVisualObject(std::vector<Boundingbox> bboxes,std::vector<std::shared_ptr<VisualObject>> *objects)
    {
        bool ret = true;
        if(objects)
        {
            if(bboxes.size() > 0)
            {
                int id = 0;
                for(int i = 0; i < (int)bboxes.size(); i++)
                {
                    ObjectType type = cvtToJmcobj(bboxes[i].cid);
                    int max_cls = static_cast<int>(ObjectType::MAX_OBJECT_TYPE);
                    if(static_cast<int>(type) < max_cls)
                    {
                        std::shared_ptr<VisualObject> obj(new VisualObject);
                        obj->type = type;
                        obj->id = id++;
                        obj->type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE),0.0f);
                        for (int k = 0; k < max_cls; ++k)
                        {
                            obj->type_probs[k] = 0.0;
                            if(k == static_cast<int>(obj->type))
                            {
                                obj->type_probs[k] = bboxes[i].score;
                            }
                        }
                        obj->upper_left[0] = bboxes[i].xmin;
                        obj->upper_left[1] = bboxes[i].ymin;
                        obj->lower_right[0] = bboxes[i].xmax;
                        obj->lower_right[1] = bboxes[i].ymax;
                        objects->push_back(obj);
                    }
                }
            }
        }
        else
        {
            std::cout << "'objects' is a null pointer."<<std::endl;
        }
        return ret;
    }

    ObjectType JmcObjectDetector::cvtToJmcobj(int cid)
    {
        #if 0
        enum class ObjectType {
        UNKNOWN = 0,
        UNKNOWN_MOVABLE = 1,
        UNKNOWN_UNMOVABLE = 2,
        PEDESTRIAN = 3,
        BICYCLE = 4,
        VEHICLE = 5,
        MAX_OBJECT_TYPE = 6,
        };
        #endif
        ObjectType type = ObjectType::MAX_OBJECT_TYPE;
        if(cid == 0)
        {
            type = ObjectType::PEDESTRIAN;
        }
        else if(cid == 1 || cid == 3)
        {
            type = ObjectType::BICYCLE;
        }
        else if(cid == 4 || cid == 33)
        {
            type = ObjectType::MAX_OBJECT_TYPE;
        }
        else if(cid == 2)
        {
            type = ObjectType::CAR;
        }
        else if(cid == 5)
        {
            type = ObjectType::BUS;
        }
        else if(cid == 7)
        {
            type = ObjectType::TRUCK;
        }
        else if(cid >= 9 && cid <= 13)
        {
            type = ObjectType::UNKNOWN_UNMOVABLE;
        }
        else if(cid == 8 || (cid >= 14 && cid <= 23))
        {
            type = ObjectType::UNKNOWN_MOVABLE;
        }
        /*
        else
        {
        type = ObjectType::UNKNOWN;
        }
        */
        return type;
    }
}
}
#if JMC_DET_SOLO_TEST
int image_test_main()
{
    std::cout << "solo test!!"<< std::endl;
	auto detect_inst_ = std::make_shared<jmc_auto::perception::JmcObjectDetector>();
	if(detect_inst_)
    {
        // 读入图片
        std::vector<jmc_auto::perception::Boundingbox> bboxes;
        cv::Mat img = cv::imread("./jmc_model/img.jpg");
        cv::Mat showImage = img.clone();
        detect_inst_->Init();
        for(int i = 0; i < 5; i++)
        {
            bboxes.clear();
            detect_inst_->Detect(img,bboxes);
        }
        showImage = detect_inst_->RenderBoundingBox(showImage, bboxes);
        cv::imshow("res",showImage);
        cv::waitKey(0);
    }
	return 0;
}

#endif
