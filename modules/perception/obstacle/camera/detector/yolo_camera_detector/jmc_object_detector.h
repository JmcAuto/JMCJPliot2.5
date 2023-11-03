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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_OBJECT_DETECTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_OBJECT_DETECTOR_H_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/obstacle/camera/common/visual_object.h"

#include "caffe/caffe.hpp"
namespace jmc_auto {
namespace perception {
    typedef struct _t_JmcPts{
        float x;
        float y;
    }tJmcPts;

    struct Boundingbox{
        float xmin;
        float ymin;
        float xmax;
        float ymax;
        float score;
        int cid;
        int with_3d; // 1: with 3d info
        float center_x;
        float center_y;
        float center_z;
        float dim_w;
        float dim_h;
        float dim_l;
        float alpha;
        float ray;
        tJmcPts front[4];
        tJmcPts rear[4];
    };

  struct Anchor{
      float width;
      float height;
  };

  class JmcObjectDetector{
    public:
    JmcObjectDetector()
    {
      data_ = new float[3 * input_width_ * input_height_];
    }

    virtual ~JmcObjectDetector() 
    {
      if(data_)
      {
          delete[] data_;
          data_=nullptr;
      }
    }

    bool SetConfigFilePath(const std::string &prototxt_file, const std::string &weight_file, bool is_user = true);

    bool Init();
    
    bool Detect(const cv::Mat &frame, std::vector<Boundingbox> &bboxes);

    cv::Mat RenderBoundingBox(cv::Mat image, const std::vector<Boundingbox> &bboxes);

    virtual bool cvtToVisualObject(std::vector<Boundingbox> bboxes,std::vector<std::shared_ptr<VisualObject>> *objects);

    protected:

    virtual std::vector<Anchor> InitAnchors();

    void Transform(const int &ih, const int &iw, const int &oh, const int &ow, std::vector<Boundingbox> &bboxes,
                  bool is_padding);

    void Nmscpu(std::vector<Boundingbox> &bboxes, float threshold);

    virtual void PostProcessParall(const int height, const int width, int scale_idx, float postThres,
                  float * origin_output, std::vector<int> Strides, std::vector<Anchor> Anchors, std::vector<Boundingbox> *bboxes);

    virtual std::vector<Boundingbox> PostProcess(std::vector<float *> origin_output, float postThres, float nmsThres);

    virtual cv::Mat PreprocessImg(const cv::Mat& img);

    virtual ObjectType cvtToJmcobj(int cid);

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

    protected:
    std::string prototxt_file_path_;
    std::string weight_file_path_;
    int config_cout_{0};
    //std::shared_ptr<CNNAdapter> cnnadapter_;
    //std::shared_ptr<CNNAdapter> cnnadapter_lane_;
    boost::shared_ptr<caffe::Net<float>> net_{nullptr};

    caffe::Blob<float> *input_layer_{nullptr};

    float *data_{nullptr};

    int input_width_{640};
    int input_height_{640};

    bool is_padding_{true};
    int num_class_{80};
    float nms_thresh_{0.6};
    float conf_thresh_{0.5};
  };

}  // namespace perception
}  // namespace jmc_auto
#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_JMC_OBJECT_DETECTOR_H_
