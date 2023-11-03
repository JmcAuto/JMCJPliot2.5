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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_PS_DETECTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_PS_DETECTOR_H_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
//#include <opencv2/highgui/highgui.hpp>
#include "caffe/caffe.hpp"
 
namespace jmc_auto {
namespace perception {

    typedef struct JmcPsinfo{
        long long time_stamp;
        int type;
        float score;
        float p0x; // right_up x
        float p0y; // right_up y
        float p1x; // left_up x
        float p1y; // left_up y
        float p2x; // right_down x
        float p2y; // right_down y
        float p3x; // left_down x
        float p3y; // left_down y
    }tJmcPsinfo;

    typedef struct JmcMkPoint
    {
        long long time_stamp;
        float score;
        int cls;
        int cx;
        int cy;
        float alpha;
    }tJmcMkPoint;

    enum class PsType{
        NONE = 0,
        HOR = 1,
        VER = 2,
        TYPE_MAX,
    };

    enum class PtType{
        TYPE_T = 0,
        TYPE_L,
        TYPE_MAX,
    };

    enum class PtShape{
        SP_NONE = 0,
        SP_L_DOWN = 1,
        SP_T_DOWN,
        SP_T_MIDDLE,
        SP_T_UP,
        SP_L_UP,
        SP_MAX,
    };

    enum class StDir{
        ST_NONE = 0,
        ST_NEG = 1,
        ST_POS = 2,

    };

    const float PI = 3.1415926;
    const float BRIFGE_ANGLE = 0.097571 + 0.134059;
    const float SEPARATOR_ANGLE = 0.284968 + 0.134059;

    class JmcPsDetector{
    public:
    JmcPsDetector()
    {
        data_ = new float[3 * input_width_ * input_height_];
        Calibrate(0.1);
    }

    virtual ~JmcPsDetector()
    {
        if(data_)
        {
            delete[] data_;
            data_=nullptr;
        }
    }

    bool SetConfigFilePath(const std::string &prototxt_file, const std::string &weight_file, bool is_user = true);

    bool Init();
    
    bool Detect(cv::Mat &frame, long long timestamp, std::vector<tJmcPsinfo>& psinfos);

    protected:

    template <typename T>
    T clip(const T &n, const T &lower, const T &upper)
    {
        return std::max(lower, std::min(n, upper));
    }

    template<class ForwardIterator>
    inline size_t argmax(ForwardIterator first, ForwardIterator last)
    {
        return std::distance(first, std::max_element(first, last));
    }

    template <typename T>
    T sigmoid(const T &n) {
        return 1 / (1 + exp(-n));
    }

    cv::Mat PreprocessImg(const cv::Mat& img);

    virtual std::vector<tJmcMkPoint> PostProcess(const float * output, float postThres, float nmsThres,long long timestamp);

    virtual void RenderMkPoints(cv::Mat &frame, std::vector<tJmcMkPoint> mkpts);
    void Transform(const int &ih, const int &iw, const int &oh, const int &ow, std::vector<tJmcMkPoint> &mk_points,
                bool is_padding);

    void Nmscpu(std::vector<tJmcMkPoint> &mk_points, float fw, float fh);

    void MakePairs(std::vector<tJmcMkPoint> &mk_points,std::vector<tJmcPsinfo> &psinfos);

    void GenPsinfo(const tJmcMkPoint &p0, const tJmcMkPoint &p1, const int type, tJmcPsinfo &psinfo);

    void RenderPsinfo(cv::Mat &frame, std::vector<tJmcPsinfo> &psinfos);

    bool IsPassThirdPoint(const tJmcMkPoint &p0, const tJmcMkPoint &p1, std::vector<tJmcMkPoint> &mkpts);

    int CheckPointShape(const tJmcMkPoint &p0, const cv::Vec2f &vec);

    float DiffDir(float a, float b);

    int PairMkpts(const tJmcMkPoint &p0,const tJmcMkPoint &p1);

    void Calibrate(float scale)
    {
        world_to_pix = scale;
        slot_width_v *= world_to_pix;
        slot_width_h *= world_to_pix;
        slot_depth_v *= world_to_pix;
        slot_depth_h *= world_to_pix;
    }

    protected:
    std::string prototxt_file_path_;
    std::string weight_file_path_;
    int config_cout_{0};
    boost::shared_ptr<caffe::Net<float>> net_{nullptr};

    caffe::Blob<float> *input_layer_{nullptr};

    float *data_{nullptr};

    int input_width_{512};
    int input_height_{512};

    bool is_padding_{true};
    int num_class_{6};
    float nms_thresh_{0.6};
    float post_thresh_{0.5};
    float boundary_thresh{0.05};
    float world_to_pix{0.1};
    float slot_threld{0.15};
    float slot_width_v{2500.0};
    float slot_width_h{5000.0};
    float slot_depth_v{5500.0};
    float slot_depth_h{2500.0};
  };

}  // namespace perception
}  // namespace jmc_auto
#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_OBJECT_DETECTOR_H_
