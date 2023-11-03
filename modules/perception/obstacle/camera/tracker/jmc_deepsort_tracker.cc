#include "modules/perception/obstacle/camera/tracker/jmc_deepsort_tracker.h"

namespace jmc_auto {
namespace perception {
    bool JmcDeepsortTracker::Init()
    {
        bool init_flag = true;
        //sort_tracker_.reset()
        if(!sort_tracker_)
        {
            sort_tracker_ = std::make_shared<TrackKfhg>();
        }
        return init_flag;
    }

    bool JmcDeepsortTracker::UnInit()
    {
        if(sort_tracker_)
        {
            sort_tracker_ = nullptr;
        }
        return true;
    }

    bool JmcDeepsortTracker::Associate(const cv::Mat& img, const double timestamp,
                std::vector<std::shared_ptr<VisualObject>>* objects)
    {
        if (!objects)
        {
            return false;
        }

        frame_idx_++;
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

    std::string JmcDeepsortTracker::Name() const
    {
        return std::string("JmcDeepsortTracker");
    }

    void JmcDeepsortTracker::SetTrakerTarget(std::vector<std::shared_ptr<VisualObject>>* objects,const cv::Mat& img)
    {
        if(!sort_tracker_)
        {
            sort_tracker_ = std::make_shared<TrackKfhg>();
        }

        img_width_ = img.cols;
        img_height_ = img.rows;
        set_target_ = true;

        if(sort_tracker_)
        {
            sort_tracker_->SetSize(img_width_,img_height_);
        }
    }

    void JmcDeepsortTracker::UpdateTarget(std::vector<std::shared_ptr<VisualObject>>* objects,const cv::Mat& img)
    {
        if(sort_tracker_)
        {
            //int num = (int)sizeof(*objects);
            std::vector<TrackingBox> det_boxes;
            for(auto item : *objects)
            {
                TrackingBox det;
                TansToTrkbox(*item.get(),det);
                det.iFrame = frame_idx_;
                det_boxes.push_back(det);
            }
            std::vector<TrackingBox> tgs = sort_tracker_->TrackProcessVsobjs(det_boxes,objects);
            //std::vector<TrackingBox> tgs = sort_tracker_->TrackProcess(det_boxes);
            //Fusion(objects,tgs);
            std::cout <<"tgs.size: " << tgs.size()<<std::endl;
            std::cout <<"objects.size: " << objects->size()<<std::endl;
        }
    }

    void JmcDeepsortTracker::TansToTrkbox(const VisualObject& vsobj, TrackingBox& trkbox)
    {
        int xmin = vsobj.upper_left.x();
        int ymin = vsobj.upper_left.y();
        int xmax = vsobj.lower_right.x();
        int ymax = vsobj.lower_right.y();
        trkbox.sBox.x = xmin;
        trkbox.sBox.y = ymin;
        trkbox.sBox.width = (xmax - xmin) > 0 ? (xmax - xmin):1;
        trkbox.sBox.height = (ymax - ymin)> 0 ? (ymax - ymin):1;
        trkbox.iD = vsobj.id;
    }

    void JmcDeepsortTracker::TansToVsobj(const TrackingBox& trkbox,VisualObject &vsobj)
    {
        if(trkbox.sBox.width > 0 && trkbox.sBox.height > 0)
        {
            vsobj.upper_left.x() = trkbox.sBox.x;
            vsobj.upper_left.y() = trkbox.sBox.y;
            vsobj.lower_right.x() = ((trkbox.sBox.x + trkbox.sBox.width) < img_width_ )? (trkbox.sBox.x + trkbox.sBox.width):(img_width_-1);
            vsobj.lower_right.y() = ((trkbox.sBox.y + trkbox.sBox.height) < img_height_) ? (trkbox.sBox.y + trkbox.sBox.height) : (img_height_-1);
            vsobj.id = trkbox.iD;
        }
    }
#if 0
    void JmcDeepsortTracker::Fusion(std::vector<std::shared_ptr<VisualObject>>* objects,const std::vector<TrackingBox> &tgs)
    {
        if(sort_tracker_)
        {
            std::vector<cv::Point> *matched = sort_tracker_->GetMatchedPair();
            for(auto item : *matched)
            {

            }
        }
        for(auto tg : tgs)
        {
            for(auto obj : *objects)
            {

            }
        }
    }
    void JmcDeepsortTracker::MergeRectToVsobj(const cv::Rect &rect,std::shared_ptr<VisualObject> vsobj)
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
    void JmcDeepsortTracker::GetCipv(const std::vector<std::shared_ptr<VisualObject>> &objects
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

    float JmcDeepsortTracker::GetVsIou(const VisualObject& bb_best, const cv::Rect& bb_gt)
    {
        cv::Rect vs_best;
        //TansVsobjToRect(bb_best,vs_best);
        return GetIou(vs_best,bb_gt);
    }

    float JmcDeepsortTracker::GetIou(const cv::Rect& bb_best, const cv::Rect& bb_gt)
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
#endif
    void JmcDeepsortTracker::PrintVsobj(std::vector<std::shared_ptr<VisualObject>>* objects,std::string tags)
    {
        for(auto obj : *objects)
        {
            std::cout << tags << "obj: ( "<< obj->upper_left.x()<< ","<< obj->upper_left.y()<< ","<< obj->lower_right.x()<< ","
                           << obj->lower_right.x()<< " ) "<< std::endl;
        }

    }
}
}
