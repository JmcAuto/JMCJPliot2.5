#ifndef JMC_DEEPSORT_TRACKER_H
#define JMC_DEEPSORT_TRACKER_H

#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "modules/perception/obstacle/camera/interface/base_camera_tracker.h"

#include <opencv2/opencv.hpp>

#include "modules/perception/obstacle/camera/tracker/deepsort/hc_kfhg_track.h"
namespace jmc_auto {
namespace perception {

class JmcDeepsortTracker : public BaseCameraTracker
{
public:
    JmcDeepsortTracker(): BaseCameraTracker() {}

    virtual ~JmcDeepsortTracker(){
        if(sort_tracker_)
        {
            sort_tracker_ = nullptr;
        }
    }

    bool Init() override;

    bool Associate(const cv::Mat& img, const double timestamp,
                   std::vector<std::shared_ptr<VisualObject>>* objects) override;

    std::string Name() const override;

    bool UnInit();

    void SetTrakerTarget(std::vector<std::shared_ptr<VisualObject>>* objects,const cv::Mat& img);
    void UpdateTarget(std::vector<std::shared_ptr<VisualObject>>* objects,const cv::Mat& img);

    //void SetTrackerParam(std::vector<cv::Point2d> tracker_roi);
private:


    void TansToTrkbox(const VisualObject& vsobj, TrackingBox& trkbox);
    void TansToVsobj(const TrackingBox& trkbox,VisualObject &vsobj);

#if 0
    void Fusion(std::vector<std::shared_ptr<VisualObject>>* objects,const std::vector<TrackingBox> &tgs);
    void MergeRectToVsobj(const cv::Rect &rect,std::shared_ptr<VisualObject> vsobj);
    void GetCipv(const std::vector<std::shared_ptr<VisualObject>>& objects
                    ,std::vector<std::shared_ptr<VisualObject>>& cipvs);

    // Computes IOU between two bounding boxes
    float GetIou(const cv::Rect& bb_best, const cv::Rect& bb_gt);

    float GetVsIou(const VisualObject& bb_best, const cv::Rect& bb_gt);
#endif
    void PrintVsobj(std::vector<std::shared_ptr<VisualObject>>* objects,std::string tags);

    int frame_idx_{0};
    bool set_default_param_{false};
    bool set_target_{false};
    float iou_threld_{0.6};
    float alpha_{0.35};
    int update_fre_{30};
    cv::Rect tracker_tgs_;
    std::shared_ptr<TrackKfhg> sort_tracker_{nullptr};
    int img_width_{1920};
    int img_height_{1080};

};

REGISTER_CAMERA_TRACKER(JmcDeepsortTracker);

}
}

#endif // JMC_DEEPSORT_TRACKER_H
