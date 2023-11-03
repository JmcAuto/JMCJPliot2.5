#include "modules/perception/obstacle/camera/tracker/jmc_camera_tracker.h"

namespace jmc_auto {
namespace perception {
    bool JmcCameraTracker::Init()
    {
        bool init_flag = true;
        //kcf_tracker_.reset()
        if(!kcf_tracker_)
        {
            kcf_tracker_ = std::make_shared<KCFTracker>(true,false,true,false);
        }

        return init_flag;
    }

    bool JmcCameraTracker::UnInit()
    {
        if(kcf_tracker_)
        {
            kcf_tracker_ = nullptr;
        }
        return true;
    }

    bool JmcCameraTracker::Associate(const cv::Mat& img, const double timestamp,
                std::vector<std::shared_ptr<VisualObject>>* objects)
    {
        if (!objects)
        {
            return false;
        }

        frame_idx_++;
        if(frame_idx_ % update_fre_ == 2)
        {
            set_target_ = false;
        }

        if(!set_target_ && (objects->size() > 0))
        {
            SetTrakerTarget(objects,img);
            std::cout<< "set target"<<std::endl;
        }
        else if(set_target_)
        {
            PrintVsobj(objects,std::string("before track"));
            UpdateTarget(objects,img);
            PrintVsobj(objects,std::string("after track"));
        }
        return true;
    }

    std::string JmcCameraTracker::Name() const
    {
        return std::string("JmcCameraTracker");
    }

    void JmcCameraTracker::SetTrakerTarget(std::vector<std::shared_ptr<VisualObject>>* objects,const cv::Mat& img)
    {
        std::vector<std::shared_ptr<VisualObject>> cipvs;
        GetCipv(*objects,cipvs);
        cv::Rect tg;
        for(auto item : cipvs)
        {
            TansVsobjToRect(*item.get(),tg);
        }

        if(!kcf_tracker_)
        {
            kcf_tracker_ = std::make_shared<KCFTracker>(true,false,true,false);
        }

        if(cipvs.size()>0)
        {
            kcf_tracker_->init(tg,img);
            img_width_ = img.cols;
            img_height_ = img.rows;
            set_target_ = true;
        }
    }

    void JmcCameraTracker::UpdateTarget(std::vector<std::shared_ptr<VisualObject>>* objects,const cv::Mat& img)
    {
        if(kcf_tracker_)
        {
            cv::Rect tg = kcf_tracker_->update(img);
            std::vector<cv::Rect> tgs;
            tgs.push_back(tg);
            Fusion(objects,tgs);
        }
    }

    void JmcCameraTracker::Fusion(std::vector<std::shared_ptr<VisualObject>>* objects,const std::vector<cv::Rect> &tgs)
    {
        for(auto tg : tgs)
        {
            for(auto obj : *objects)
            {
                if(GetVsIou(*obj.get(),tg) > iou_threld_)
                {
                    MergeRectToVsobj(tg,obj);
                }
            }
        }
    }

    void JmcCameraTracker::MergeRectToVsobj(const cv::Rect &rect,std::shared_ptr<VisualObject> vsobj)
    {
        if(rect.width > 0 && rect.height > 0)
        {
            vsobj->upper_left.x() = (int)(alpha_ * vsobj->upper_left.x() + (1.0 - alpha_) * rect.x);
            vsobj->upper_left.y() =(int)(alpha_ * vsobj->upper_left.y() + (1.0 - alpha_) * rect.y);
            int lrx = ((rect.x + rect.width) < img_width_ )? (rect.x + rect.width):(img_width_-1);
            int lry = ((rect.y + rect.height) < img_height_) ? (rect.y + rect.height) : (img_height_-1);
            vsobj->lower_right.x() = (int)(alpha_ * vsobj->lower_right.x() + (1.0 - alpha_) * lrx);
            vsobj->lower_right.y() = (int)(alpha_ * vsobj->lower_right.y() + (1.0 - alpha_) * lry);
        }
    }

    void JmcCameraTracker::TansVsobjToRect(const VisualObject& vsobj,cv::Rect& rect)
    {
        int xmin = vsobj.upper_left.x();
        int ymin = vsobj.upper_left.y();
        int xmax = vsobj.lower_right.x();
        int ymax = vsobj.lower_right.y();
        rect.x = xmin;
        rect.y = ymin;
        rect.width = (xmax - xmin) > 0 ? (xmax - xmin):1;
        rect.height = (ymax - ymin)> 0 ? (ymax - ymin):1;
    }

    void JmcCameraTracker::TansRectToVsobj(const cv::Rect &rect,VisualObject &vsobj)
    {
        if(rect.width > 0 && rect.height > 0)
        {
            vsobj.upper_left.x() = rect.x;
            vsobj.upper_left.y() = rect.y;
            vsobj.lower_right.x() = ((rect.x + rect.width) < img_width_ )? (rect.x + rect.width):(img_width_-1);
            vsobj.lower_right.y() = ((rect.y + rect.height) < img_height_) ? (rect.y + rect.height) : (img_height_-1);
        }
    }

    void JmcCameraTracker::GetCipv(const std::vector<std::shared_ptr<VisualObject>> &objects
                    ,std::vector<std::shared_ptr<VisualObject>> &cipvs)
    {
        if(objects.size() > 0)
        {
            //cipvs.push_back(objects[0]);
            unsigned int dist_max = 0xfffffff;
            std::shared_ptr<VisualObject> cipv_obj = nullptr;
            for(auto obj : objects)
            {
                unsigned int center_x = abs(img_width_ / 2 - (obj->upper_left.x() + obj->lower_right.x()) / 2);
                unsigned int center_y = abs(img_height_ - obj->lower_right.y());
                if(center_x * center_y < dist_max)
                {
                    cipv_obj = obj;
                    dist_max = center_x * center_y;
                }
            }
            if(cipv_obj)
            {
                cipvs.push_back(cipv_obj);
            }
        }
    }

    float JmcCameraTracker::GetVsIou(const VisualObject& bb_best, const cv::Rect& bb_gt)
    {
        cv::Rect vs_best;
        TansVsobjToRect(bb_best,vs_best);
        return GetIou(vs_best,bb_gt);
    }

    float JmcCameraTracker::GetIou(const cv::Rect& bb_best, const cv::Rect& bb_gt)
    {
        float fiou = 0.0;
        float fMinRight = std::min((bb_best.x + bb_best.width), (bb_gt.x + bb_gt.width));
        float fMinBottom = std::min((bb_best.y + bb_best.height), (bb_gt.y + bb_gt.height));
        float fMaxLeft = std::max(bb_best.x, bb_gt.x);
        float fMaxTop = std::max(bb_best.y, bb_gt.y);

        float in = (fMinRight - fMaxLeft)*(fMinBottom - fMaxTop);
        float un = bb_best.height*bb_best.width + bb_gt.height*bb_gt.width - in;
        if (un > DBL_EPSILON)
        {
            fiou = (float)(in / un);
        }
        return fiou;
    }

    void JmcCameraTracker::PrintVsobj(std::vector<std::shared_ptr<VisualObject>>* objects,std::string tags)
    {
        for(auto obj : *objects)
        {
            std::cout << tags << "obj: ( "<< obj->upper_left.x()<< ","<< obj->upper_left.y()<< ","<< obj->lower_right.x()<< ","
                           << obj->lower_right.x()<< " ) "<< std::endl;
        }

    }
}
}
