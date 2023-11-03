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

#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_psdet.h"
#include "modules/common/util/file.h"
#include <cmath>
#include <unordered_map>
#include <utility>
#include <ctime>

namespace jmc_auto {
namespace perception {
    #if JMC_DET_SOLO_TEST  
    static std::string prototxt_path = "./jmc_model/dmpr_1.1_512_512_211109.prototxt";
    static std::string weight_file = "./jmc_model/dmpr_1.1_512_512_211109.caffemodel";
    #else
    static std::string prototxt_path = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/dmpr.prototxt";
    static std::string weight_file = "modules/perception/obstacle/camera/detector/yolo_camera_detector/jmc_model/dmpr.caffemodel";
    #endif

    bool JmcPsDetector::SetConfigFilePath(const std::string &prototxt_file, const std::string &weight_file, bool is_user)
    {
        prototxt_file_path_ = prototxt_file;
        weight_file_path_ = weight_file;
        if(is_user)
        {
            config_cout_++;
        }
        return true;
    }

    bool JmcPsDetector::Init()
    {
        bool ret = true;
        //::google::InitGoogleLogging("caffe");
        caffe::Caffe::set_mode(caffe::Caffe::GPU);
        //caffe::Caffe::set_solver_rank(1);
        // init Net
        if(config_cout_ == 0)
        {
            // use default file 
            SetConfigFilePath(prototxt_path,weight_file,false);
            //std::cout << config_cout_ << std::endl;
        }

        net_.reset(new caffe::Net<float>(prototxt_file_path_, caffe::TEST));
        CHECK((net_ != nullptr));
        ADEBUG<<"START_COPY";
        net_->CopyTrainedLayersFrom(weight_file_path_);
        ADEBUG<<"COPY_COMPELET";
        input_layer_ = net_->input_blobs()[0];

        return ret;
    }

    bool JmcPsDetector::Detect(cv::Mat &frame, long long timestamp, std::vector<tJmcPsinfo>& psinfos)
    {
        bool ret = true;
        //caffe::Caffe::SetDevice(caffe::Caffe::GPU);
        caffe::Timer pre_time;
        pre_time.Start();
        if(net_ && frame.data != nullptr)
        {
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
                            data_[i] = (float)uc_pixel[0] / 255.0;
                            data_[i + input_height_ * input_width_] = (float)uc_pixel[1] / 255.0;
                            data_[i + 2 * input_height_ * input_width_] = (float)uc_pixel[2] / 255.0;
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
                    cv::Mat inf = cv::Mat(cv::Size(input_width_,input_height_),CV_32FC3,input_data);
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
            //std::cout << "Parking slot inference cost: " << det_time.MilliSeconds() << " ms"<< std::endl;

            caffe::Timer post_time;
            post_time.Start();
            //int out_num = net_->num_outputs();
            caffe::Blob<float> *output_layer0 = net_->output_blobs()[0];
            const float *output0 = output_layer0->cpu_data();
            //std::cout << "output shape: " << output_layer0->shape(0) << " " << output_layer0->shape(1) << " " <<  output_layer0->shape(2) << " " <<  output_layer0->shape(3) << std::endl;
            std::vector<tJmcMkPoint> mk_points = PostProcess(output0,post_thresh_,nms_thresh_,timestamp);
            //std::cout <<"mk_points size "<< mk_points.size()<< std::endl;
            Transform(frame.rows, frame.cols, input_height_, input_width_,mk_points,true);

            MakePairs(mk_points,psinfos);
            //RenderMkPoints(frame,mk_points);
            RenderPsinfo(frame,psinfos);
            post_time.Stop();
            //std::cout << "Parking slot post process cost: " << post_time.MilliSeconds() << " ms"<< std::endl;
            // cv::imshow("frame",frame);
        }
        return ret;
    }

#if 1
    std::vector<tJmcMkPoint> JmcPsDetector::PostProcess(const float *output, float postThres, float nmsThres,long long timestamp)
    {
        // N * C * H * W
        int scale = 32;// down scale 5 times
        int feature_width = input_width_ / scale;
        int feature_height = input_height_ / scale;
        int feature_size = feature_width * feature_height;
        int boundary_xmin = (int)(input_width_ * boundary_thresh);
        int boundary_xmax = (int)(input_width_ - boundary_xmin);
        int boundary_ymin = (int)(input_height_ * boundary_thresh);
        int boundary_ymax = (int)(input_height_  - boundary_ymin);
        const float *data_ptr = output;
        std::vector<tJmcMkPoint> mk_points;

        for(int y = 0; y < feature_height; y++)
        {
            for(int x = 0; x < feature_width; x++)
            {
                float score = sigmoid(data_ptr[0]);
                if(score > postThres)
                {
                    tJmcMkPoint mk_point;
                    mk_point.cx = x * scale + scale * (sigmoid(data_ptr[2 * feature_size]));
                    mk_point.cy = y * scale + scale * (sigmoid(data_ptr[3 * feature_size]));
                    if((boundary_xmin < mk_point.cx)
                       && (boundary_xmax > mk_point.cx)
                       && (boundary_ymin < mk_point.cy)
                       && (boundary_ymax > mk_point.cy))
                    {
                        // in the boundary line
                        float cos_a = tanh(static_cast<double>(data_ptr[4 * feature_size]));
                        float sin_a = tanh(static_cast<double>(data_ptr[5 * feature_size]));
                        mk_point.score = score;
                        mk_point.alpha = atan2(sin_a,cos_a);
                        mk_point.cls = static_cast<int>(sigmoid(data_ptr[1 * feature_size])+ 0.5);
                        mk_points.push_back(mk_point);
                    }
                }
                data_ptr += 1;
            }
        }
        Nmscpu(mk_points,feature_width,feature_height);
        return mk_points;
    }
#else
    std::vector<tJmcMkPoint> JmcPsDetector::PostProcess(const float *output, float postThres, float nmsThres,long long timestamp)
    {
        // N * H * W * C
        int scale = 32;// down scale 5 times
        int feature_width = input_width_ / scale;
        int feature_height = input_height_ / scale;
        const float *data_ptr = output;
        std::vector<tJmcMkPoint> mk_points;

        for(int y = 0; y < feature_height; y++)
        {
            for(int x = 0; x < feature_width; x++)
            {
                float score = sigmoid(data_ptr[0]);
                if(score > postThres)
                {
                    tJmcMkPoint mk_point;
                    float cos_a = tanh(static_cast<double>(data_ptr[4]));
                    float sin_a = tanh(static_cast<double>(data_ptr[5]));
                    mk_point.score = score;
                    mk_point.alpha = atan2(sin_a,cos_a);
                    mk_point.cls = static_cast<int>(sigmoid(data_ptr[1])+ 0.5);
                    mk_point.cx = x * scale + feature_width * (sigmoid(data_ptr[2]));
                    mk_point.cy = y * scale + feature_height * (sigmoid(data_ptr[3]));
                    mk_points.push_back(mk_point);
                }
                data_ptr += num_class_;
            }
        }
        return mk_points;
    }
#endif

    void JmcPsDetector::MakePairs(std::vector<tJmcMkPoint> &mkpts, std::vector<tJmcPsinfo> &psinfos)
    {
        //int pts_size = (int)mkpts.size();
        for (int i = 0; i < (int)mkpts.size(); ++i)
        {
            for (int j = i + 1; j < (int)mkpts.size(); ++j)
            {
                auto p0 = mkpts[i];
                auto p1 = mkpts[j];
                int type = (int)PsType::NONE;
                float dist = sqrt((p0.cx - p1.cx)*(p0.cx - p1.cx) + (p0.cy - p1.cy)*(p0.cy - p1.cy));
                if((dist > slot_width_v * (1.0 - slot_threld)) && (dist < slot_width_v * (1.0 + slot_threld)) )
                {
                    type = (int)PsType::VER;
                }
                /*else if((dist > slot_width_h * (1.0 - slot_threld)) && (dist < slot_width_h * (1.0 + slot_threld)) )
                {
                    type = (int)PsType::HOR;
                }*/

                if((int)PsType::NONE != type)
                {
                    if(false == IsPassThirdPoint(p0,p1,mkpts) )
                    {
                        // do not pass the third point
                        tJmcPsinfo psinfo;
                        int res = PairMkpts(p0,p1);
                        if((int)StDir::ST_POS == res)
                        {
                            GenPsinfo(p0,p1,type,psinfo);
                            psinfos.push_back(psinfo);
                        }
                        else if((int)StDir::ST_NEG == res)
                        {
                            GenPsinfo(p1,p0,type,psinfo);
                            psinfos.push_back(psinfo);
                        }
                    }
                }
            }
        }
    }

    void JmcPsDetector::GenPsinfo(const tJmcMkPoint &p0, const tJmcMkPoint &p1, const int type, tJmcPsinfo &psinfo)
    {
        if((int)PsType::VER == type)
        {
            cv::Vec2f vec((p1.cx - p0.cx),(p1.cy - p0.cy));
            cv::normalize(vec,vec);

            psinfo.time_stamp = p0.time_stamp;
            psinfo.type = type;
            psinfo.p0x = p0.cx;
            psinfo.p0y = p0.cy;
            psinfo.p1x = p1.cx;
            psinfo.p1y = p1.cy;

            psinfo.p2x = p0.cx + slot_depth_v * vec(1);
            psinfo.p2y = p0.cy - slot_depth_v * vec(0);
            psinfo.p3x = p1.cx + slot_depth_v * vec(1);
            psinfo.p3y = p1.cy - slot_depth_v * vec(0);

        }
    }

    bool JmcPsDetector::IsPassThirdPoint(const tJmcMkPoint &p0, const tJmcMkPoint &p1, std::vector<tJmcMkPoint> &mkpts)
    {
        bool ret = false;
        for(auto &mkpt : mkpts)
        {
            cv::Vec2f ab((p0.cx-mkpt.cx),(p0.cy-mkpt.cy));
            cv::normalize(ab,ab);
            cv::Vec2f bc((mkpt.cx-p1.cx),(mkpt.cy-p1.cy));
            //cv::Vec2f bc((p1.cx-mkpt.cx),(p1.cy-mkpt.cy));
            cv::normalize(bc,bc);
            if(fabs(ab(1)) > 0.0001 && fabs(bc(1)) > 0.0001 )
            {
                //float alpha_1 = tanh(ab(0)/ab(1)) * 180 / PI;
                //float alpha_2 = tanh(bc(0)/bc(1)) * 180 / PI;
                //std::cout << alpha_1 << std::endl;
                //std::cout << alpha_2 << std::endl;
                //std::cout << ab.dot(bc) << std::endl;
                //if((alpha_1 - alpha_2) < 1.0 && (alpha_1 - alpha_2) > -1.0)
                if(ab.dot(bc) > 0.8)
                {
                    ret = true;
                }
            }
        }
        return ret;
    }

    float JmcPsDetector::DiffDir(float a, float b)
    {
        float diff = fabs(a - b);
        if(diff >= PI)
        {
            diff = 2 * PI - diff;
        }
        return diff;
    }

    int JmcPsDetector::CheckPointShape(const tJmcMkPoint &pt, const cv::Vec2f &vec)
    {
        int ret = 0;
        float dir = atan2(vec(1),vec(0));
        float dir_up = atan2(-vec(0),vec(1));
        float dir_down = atan2(vec(0),-vec(1));
        if((int)PtType::TYPE_T == pt.cls)
        {
            if(DiffDir(dir, pt.alpha) < BRIFGE_ANGLE)
            {
                ret = (int)PtShape::SP_T_MIDDLE;
            }
            else if(DiffDir(dir_up, pt.alpha) < SEPARATOR_ANGLE)
            {
                ret = (int)PtShape::SP_T_UP;
            }
            else if(DiffDir(dir_down, pt.alpha) < SEPARATOR_ANGLE)
            {
                ret = (int)PtShape::SP_T_DOWN;
            }
        }
        else if((int)PtType::TYPE_L == pt.cls)
        {
            if(DiffDir(dir, pt.alpha) < BRIFGE_ANGLE)
            {
                ret = (int)PtShape::SP_L_DOWN;
            }
            else if(DiffDir(dir_up, pt.alpha) < SEPARATOR_ANGLE)
            {
                ret = (int)PtShape::SP_L_UP;
            }
        }
        return ret;
    }

    int JmcPsDetector::PairMkpts(const tJmcMkPoint &p0,const tJmcMkPoint &p1)
    {
        int ret = (int)StDir::ST_POS;
        cv::Vec2f ab((p1.cx - p0.cx),(p1.cy - p0.cy));
        cv::normalize(ab,ab);
        int shape_a = (int)CheckPointShape(p0,ab);
        int shape_b = (int)CheckPointShape(p1,-ab);

        if( ((int)PtShape::SP_NONE == shape_a ||(int)PtShape::SP_NONE == shape_b)
            || ((int)PtShape::SP_T_MIDDLE == shape_a && (int)PtShape::SP_T_MIDDLE == shape_b)
            || ((int)PtShape::SP_T_MIDDLE < shape_a && (int)PtShape::SP_T_MIDDLE < shape_b)
            || ((int)PtShape::SP_T_MIDDLE > shape_a && (int)PtShape::SP_T_MIDDLE > shape_b)
          )
        {
            ret = (int)StDir::ST_NONE;
        }

        if(ret != 0)
        {
            if((int)PtShape::SP_T_MIDDLE == shape_a)
            {
                if((int)PtShape::SP_T_MIDDLE > shape_b)
                {
                    ret = (int)StDir::ST_POS;
                }
                else if((int)PtShape::SP_T_MIDDLE < shape_b)
                {
                    ret = (int)StDir::ST_NEG;
                }
            }
            else
            {
                if((int)PtShape::SP_T_MIDDLE < shape_a)
                {
                    ret = (int)StDir::ST_POS;
                }
                else if((int)PtShape::SP_T_MIDDLE > shape_a)
                {
                    ret = (int)StDir::ST_NEG;
                }
            }
        }
        return ret;
    }

    void JmcPsDetector::Transform(const int &ih, const int &iw, const int &oh, const int &ow, std::vector<tJmcMkPoint> &mk_points,
                bool is_padding)
    {
        if(is_padding)
        {
            float scale = std::min(static_cast<float>(ow) / static_cast<float>(iw), static_cast<float>(oh) / static_cast<float>(ih));
            int nh = static_cast<int>(scale * static_cast<float>(ih));
            int nw = static_cast<int>(scale * static_cast<float>(iw));
            int dh = (oh - nh) / 2;
            int dw = (ow - nw) / 2;
            for (auto &mkpt : mk_points)
            {
                mkpt.cx = (mkpt.cx - dw) / scale;
                mkpt.cy = (mkpt.cy - dh) / scale;
            }
        }
        else
        {
            for (auto &mkpt : mk_points)
            {
                mkpt.cx = mkpt.cx * iw / ow;
                mkpt.cy = mkpt.cy * ih / oh;
            }
        }
    }

    void JmcPsDetector::Nmscpu(std::vector<tJmcMkPoint>& mkpts, float fw, float fh)
    {
        if(mkpts.size())
        {
            std::sort(mkpts.begin(), mkpts.end(),
                      [&](tJmcMkPoint p1, tJmcMkPoint p2){return p1.score > p2.score;});

            for (int i = 0; i < (int)mkpts.size(); ++i)
            {
                for (int j = i + 1; j < (int)mkpts.size();)
                {
                    int ix = mkpts[i].cx;
                    int iy = mkpts[i].cy;
                    int jx = mkpts[j].cx;
                    int jy = mkpts[j].cy;
                    if((fw > abs(ix - jx)) && (fh > abs(iy -jy)))
                    {
                        mkpts.erase(mkpts.begin() + j);
                    }
                    else
                    {
                        ++j;
                    }
                }
            }
        }
    }

    void JmcPsDetector::RenderMkPoints(cv::Mat &frame, std::vector<tJmcMkPoint> mkpts)
    {
        for(auto &mkpt : mkpts)
        {
            cv::line(frame,cv::Point(mkpt.cx,mkpt.cy),cv::Point(mkpt.cx,mkpt.cy),
                       CV_RGB(255,0,0),5,CV_AA,0);
        }
    }

    void JmcPsDetector::RenderPsinfo(cv::Mat &frame, std::vector<tJmcPsinfo> &psinfos)
    {
        for(auto &psinfo : psinfos)
        {
            cv::line(frame,cv::Point(psinfo.p0x,psinfo.p0y),cv::Point(psinfo.p1x,psinfo.p1y),
                       CV_RGB(255,0,0),3,CV_AA,0);
            cv::line(frame,cv::Point(psinfo.p0x,psinfo.p0y),cv::Point(psinfo.p2x,psinfo.p2y),
                       CV_RGB(0,255,0),3,CV_AA,0);
            cv::line(frame,cv::Point(psinfo.p3x,psinfo.p3y),cv::Point(psinfo.p1x,psinfo.p1y),
                       CV_RGB(0,0,255),3,CV_AA,0);
        }
    }

    cv::Mat JmcPsDetector::PreprocessImg(const cv::Mat& img)
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

}
}
#if JMC_DET_SOLO_TEST
int ps_test_main()
{
    std::cout << "solo test!!"<< std::endl;
    auto detect_inst_ = std::make_shared<jmc_auto::perception::JmcPsDetector>();
	if(detect_inst_)
    {
        std::vector<jmc_auto::perception::tJmcPsinfo> psinfos;
        cv::Mat img = cv::imread("./jmc_model/img.jpg");
        cv::Mat showImage = img.clone();
        detect_inst_->Init();
        for(int i = 0; i < 5; i++)
        {
            psinfos.clear();
            detect_inst_->Detect(img,0,psinfos);
        }
        cv::imshow("res",showImage);
        cv::waitKey(0);
    }
	return 0;
}

#endif
