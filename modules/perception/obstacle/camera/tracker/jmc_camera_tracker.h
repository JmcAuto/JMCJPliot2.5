#ifndef JMC_CAMERA_TRACKER_H
#define JMC_CAMERA_TRACKER_H

#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"
#include "modules/perception/obstacle/camera/interface/base_camera_tracker.h"

#include <opencv2/opencv.hpp>
//#include "./third_party/kcf/kcftracker.hpp"
#include "modules/perception/obstacle/camera/tracker/jmc_kcf/kcftracker.hpp"
namespace jmc_auto {
namespace perception {

class JmcCameraTracker : public BaseCameraTracker
{
public:
    JmcCameraTracker(): BaseCameraTracker() {}

    virtual ~JmcCameraTracker(){
        if(kcf_tracker_)
        {
            kcf_tracker_ = nullptr;
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
    void Fusion(std::vector<std::shared_ptr<VisualObject>>* objects,const std::vector<cv::Rect> &tgs);
    void MergeRectToVsobj(const cv::Rect &rect,std::shared_ptr<VisualObject> vsobj);

    void TansVsobjToRect(const VisualObject &vsobj,cv::Rect &rect);
    void TansRectToVsobj(const cv::Rect &rect,VisualObject &vsobj);

    void GetCipv(const std::vector<std::shared_ptr<VisualObject>>& objects
                    ,std::vector<std::shared_ptr<VisualObject>>& cipvs);

    // Computes IOU between two bounding boxes
    float GetIou(const cv::Rect& bb_best, const cv::Rect& bb_gt);

    float GetVsIou(const VisualObject& bb_best, const cv::Rect& bb_gt);

    void PrintVsobj(std::vector<std::shared_ptr<VisualObject>>* objects,std::string tags);

    int frame_idx_{0};
    bool set_default_param_{false};
    bool set_target_{false};
    float iou_threld_{0.6};
    float alpha_{0.35};
    int update_fre_{30};
    cv::Rect tracker_tgs_;
    std::shared_ptr<KCFTracker> kcf_tracker_{nullptr};
    int img_width_{1920};
    int img_height_{1080};

};

REGISTER_CAMERA_TRACKER(JmcCameraTracker);

}
}

#endif // JMC_CAMERA_TRACKER_H
