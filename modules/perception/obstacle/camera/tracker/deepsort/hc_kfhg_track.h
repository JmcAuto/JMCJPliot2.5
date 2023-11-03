#ifndef _HC_KFHG_TRACK_H_
#define _HC_KFHG_TRACK_H_
#include <set>
#include "kalman_tracker.h"
#include "hungarian.h"
#include "modules/perception/obstacle/camera/common/visual_object.h"

#define UMATDET 32
#define MAXUMATDET 64

//namespace jmc_auto {
//namespace perception {
    class TrackKfhg
    {
    public:
        TrackKfhg() {
            trkNum = 0;
            detNum = 0;
            frame_count = 0;
            min_hits = 3; // min
            max_age = 4; // max loss
            iouThreshold = 0.7;
        }
        ~TrackKfhg() {}

        std::vector<TrackingBox> TrackProcess(std::vector<TrackingBox> &detBoxes);

        std::vector<TrackingBox> TrackProcessVsobjs(std::vector<TrackingBox> &detBoxes
                                          ,std::vector<std::shared_ptr<jmc_auto::perception::VisualObject>>* objects);
        // Computes IOU between two bounding boxes
        double GetIOU(TRACK_OBJ_RECT_S bb_test, TRACK_OBJ_RECT_S bb_gt)
        {

            float fMinRight = min((bb_test.x + bb_test.width), (bb_gt.x + bb_gt.width));
            float fMinBottom = min((bb_test.y + bb_test.height), (bb_gt.y + bb_gt.height));
            float fMaxLeft = max(bb_test.x, bb_gt.x);
            float fMaxTop = max(bb_test.y, bb_gt.y);

            float in = (fMinRight - fMaxLeft)*(fMinBottom - fMaxTop);
            float un = bb_test.height*bb_test.width + bb_gt.height*bb_gt.width - in;
            if (un < DBL_EPSILON)
                return 0;

            return (double)(in / un);
        }

        std::vector<cv::Point>* GetMatchedPair()
        {
            return &matchedPairs;
        }

        void SetSize(int w, int h)
        {
            width = w;
            height = h;
        }

    private:
        int frame_count;
        int max_age;
        int min_hits;
        double iouThreshold;
        std::vector<KalmanTracker<jmc_auto::perception::VisualObject>> trackers;
                                     // variables used in the for-loop
        std::vector<TRACK_OBJ_RECT_S> predictedBoxes;
        std::vector<TrackingBox> trackingResult;
        std::vector<vector<double>> iouMatrix;
        std::vector<int> assignment;
        TrackingBox sTrackBox[MAXUMATDET];
        std::set<int> unmatchedDetections;
        std::set<int> unmatchedTrajectories;
        std::set<int> allItems;
        std::set<int> matchedItems;
        std::vector<cv::Point> matchedPairs;
        std::vector<jmc_auto::perception::VisualObject> vsobjResult;

        unsigned int trkNum;
        unsigned int detNum;
        int width;
        int height;
    };
//}
//}


#endif
