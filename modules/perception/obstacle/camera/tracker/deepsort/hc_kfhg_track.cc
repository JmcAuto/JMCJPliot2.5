#include <iostream>
#include <stdio.h>
#include "hc_kfhg_track.h"

//using namespace std;
//namespace jmc_auto {
//namespace perception {
std::vector<TrackingBox> TrackKfhg::TrackProcess(std::vector<TrackingBox> &detBoxes)
{
	HungarianAlgorithm HungAlgo;
    int num = (int)detBoxes.size();
	frame_count++;

    std::cout << "-----Total Frame is " << frame_count << endl;
    trackingResult.clear();
	
    for (unsigned int i = 0; i < num; i++)
	{
        std::cout << "src["<< i <<"]: "<<detBoxes[i].sBox.x<<","<<detBoxes[i].sBox.y<<std::endl;
	}
	if (trackers.size() == 0)
	{
		// initialize kalman trackers using first detections.
        for (unsigned int i = 0; i < num; i++)
		{
            KalmanTracker<jmc_auto::perception::VisualObject> trk(detBoxes[i].sBox);
			trackers.push_back(trk);
            std::cout << "[" << detBoxes[i].iFrame << "]" << "    " << "Id is: " << detBoxes[i].iD << endl;
		}
	}
	else
	{
		predictedBoxes.clear();
		for (auto it = trackers.begin(); it != trackers.end();)
		{
			TRACK_OBJ_RECT_S pBox = (*it).predict();
			if (pBox.x >= 0 && pBox.y >= 0)
			{
				predictedBoxes.push_back(pBox);
				it++;
			}
			else
			{
				it = trackers.erase(it);
				//cerr << "Box invalid at frame: " << frame_count << endl;
			}
		}

		//3.2 associate detections to tracked object (both represented as bounding boxes)

		trkNum = predictedBoxes.size();
        detNum = num;

		iouMatrix.clear();
		iouMatrix.resize(trkNum, vector<double>(detNum, 0));

		for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
		{
			for (unsigned int j = 0; j < detNum; j++)
			{
				// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
                iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detBoxes[j].sBox);
			}
		}
		// solve the assignment problem using hungarian algorithm.
		// the resulting assignment is [track(prediction) : detection], with len=preNum
		assignment.clear();
		HungAlgo.Solve(iouMatrix, assignment);
		unmatchedTrajectories.clear();
		unmatchedDetections.clear();
		allItems.clear();
		matchedItems.clear();
		
		if (detNum > trkNum) //	there are unmatched detections
		{
			for (unsigned int n = 0; n < detNum; n++)
            {
				allItems.insert(n);
            }

			for (unsigned int i = 0; i < trkNum; ++i)
            {
				matchedItems.insert(assignment[i]);
            }

			/*set_difference(allItems.begin(), allItems.end(),
			matchedItems.begin(), matchedItems.end(),
			insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));*/
		}
		else if (detNum < trkNum) // there are unmatched trajectory/predictions
		{
			for (unsigned int i = 0; i < trkNum; ++i)
				if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
					unmatchedTrajectories.insert(i);
		}

		matchedPairs.clear();
		for (unsigned int i = 0; i < trkNum; ++i)
		{
			if (assignment[i] == -1) // pass over invalid values
				continue;
			if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
			{
				unmatchedTrajectories.insert(i);
				unmatchedDetections.insert(assignment[i]);
			}
			else
            {
				matchedPairs.push_back(cv::Point(i, assignment[i]));
            }
		}

		///////////////////////////////////////
		// 3.3. updating trackers
		// update matched trackers with assigned detections.
		// each prediction is corresponding to a tracker

		int detIdx, trkIdx;
		for (unsigned int i = 0; i < matchedPairs.size(); i++)
		{
			trkIdx = matchedPairs[i].x;
			detIdx = matchedPairs[i].y;
            trackers[trkIdx].update(&(detBoxes[i].sBox));
		}
		// create and initialise new trackers for unmatched detections
		for (auto umd : unmatchedDetections)
		{
            KalmanTracker<jmc_auto::perception::VisualObject> tracker(detBoxes[umd].sBox);
			trackers.push_back(tracker);
		}
		//final get trackers' output
		std::cout<<"tracker.size: "<< trackers.size()<<std::endl;
		for (auto it = trackers.begin(); it != trackers.end();)
		{
            std::cout <<"id: " << (*it).m_id << " age: "<< (*it).m_time_since_update<<" hit:"<< (*it).m_hit_streak <<std::endl;
			if (((*it).m_time_since_update >= 1 && (*it).m_time_since_update <= max_age) ||
				((*it).m_hit_streak >= min_hits || frame_count <= min_hits))
			{
				TrackingBox res;
				res.sBox = (*it).get_state();
                res.iD = (*it).m_id;
				res.iFrame = frame_count;
                trackingResult.push_back(res);
			}
			
			// remove dead tracklet
			if (it != trackers.end() && (*it).m_time_since_update > max_age)
			{
				it = trackers.erase(it);
			}
			else
			{
				it++;
			}
		}
        for (auto tb : trackingResult)
		{
            std::cout<<"dst: "<< tb.iFrame << "id: " << tb.iD << "x: " << tb.sBox.x << "y: " << tb.sBox.y << "w: " << tb.sBox.width << "h: " << tb.sBox.height << std::endl;
        }
	}
    return trackingResult;
}

std::vector<TrackingBox> TrackKfhg::TrackProcessVsobjs(std::vector<TrackingBox> &detBoxes
                                            ,std::vector<std::shared_ptr<jmc_auto::perception::VisualObject>> *objects)
{
    HungarianAlgorithm HungAlgo;
    int num = (int)detBoxes.size();
    frame_count++;

    std::cout << "-----Total Frame is " << frame_count << endl;
    trackingResult.clear();

    for (unsigned int i = 0; i < num; i++)
    {
        std::cout << "src["<< i <<"]: "<<detBoxes[i].sBox.x<<","<<detBoxes[i].sBox.y<<std::endl;
    }
    if (trackers.size() == 0)
    {
        // initialize kalman trackers using first detections.
        for (unsigned int i = 0; i < num; i++)
        {
            KalmanTracker<jmc_auto::perception::VisualObject> trk(detBoxes[i].sBox);
            trk.set_data(*(*objects)[i].get());
            trackers.push_back(trk);
            std::cout << "[" << detBoxes[i].iFrame << "]" << "    " << "Id is: " << detBoxes[i].iD << endl;
        }
    }
    else
    {
        predictedBoxes.clear();
        for (auto it = trackers.begin(); it != trackers.end();)
        {
            TRACK_OBJ_RECT_S pBox = (*it).predict();
            if (pBox.x >= 0 && pBox.y >= 0)
            {
                predictedBoxes.push_back(pBox);
                it++;
            }
            else
            {
                it = trackers.erase(it);
                //cerr << "Box invalid at frame: " << frame_count << endl;
            }
        }

        //3.2 associate detections to tracked object (both represented as bounding boxes)

        trkNum = predictedBoxes.size();
        detNum = num;

        iouMatrix.clear();
        iouMatrix.resize(trkNum, vector<double>(detNum, 0));

        for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
        {
            for (unsigned int j = 0; j < detNum; j++)
            {
                // use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
                iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detBoxes[j].sBox);
            }
        }
        // solve the assignment problem using hungarian algorithm.
        // the resulting assignment is [track(prediction) : detection], with len=preNum
        assignment.clear();
        HungAlgo.Solve(iouMatrix, assignment);
        unmatchedTrajectories.clear();
        unmatchedDetections.clear();
        allItems.clear();
        matchedItems.clear();

        if (detNum > trkNum) //	there are unmatched detections
        {
            for (unsigned int n = 0; n < detNum; n++)
            {
                allItems.insert(n);
            }

            for (unsigned int i = 0; i < trkNum; ++i)
            {
                matchedItems.insert(assignment[i]);
            }

            /*set_difference(allItems.begin(), allItems.end(),
            matchedItems.begin(), matchedItems.end(),
            insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));*/
        }
        else if (detNum < trkNum) // there are unmatched trajectory/predictions
        {
            for (unsigned int i = 0; i < trkNum; ++i)
                if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
                    unmatchedTrajectories.insert(i);
        }

        matchedPairs.clear();
        for (unsigned int i = 0; i < trkNum; ++i)
        {
            if (assignment[i] == -1) // pass over invalid values
                continue;
            if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
            {
                unmatchedTrajectories.insert(i);
                unmatchedDetections.insert(assignment[i]);
            }
            else
            {
                matchedPairs.push_back(cv::Point(i, assignment[i]));
            }
        }

        ///////////////////////////////////////
        // 3.3. updating trackers
        // update matched trackers with assigned detections.
        // each prediction is corresponding to a tracker

        int detIdx, trkIdx;
        for (unsigned int i = 0; i < matchedPairs.size(); i++)
        {
            trkIdx = matchedPairs[i].x;
            detIdx = matchedPairs[i].y;
            trackers[trkIdx].update(&(detBoxes[i].sBox));
            trackers[trkIdx].set_data(*(*objects)[i].get());
        }
        // create and initialise new trackers for unmatched detections
        for (auto umd : unmatchedDetections)
        {
            KalmanTracker<jmc_auto::perception::VisualObject> trk(detBoxes[umd].sBox);
            trk.set_data(*(*objects)[umd].get());
            trackers.push_back(trk);
        }
        //final get trackers' output
        std::cout<<"tracker.size: "<< trackers.size()<<std::endl;

        objects->clear();
        for (auto it = trackers.begin(); it != trackers.end();)
        {
            std::cout <<"id: " << (*it).m_id << " age: "<< (*it).m_time_since_update<<" hit:"<< (*it).m_hit_streak <<std::endl;
            if (((*it).m_time_since_update >= 1 && (*it).m_time_since_update <= max_age) ||
                ((*it).m_hit_streak >= min_hits || frame_count <= min_hits))
            {
                TrackingBox res;
                res.sBox = (*it).get_state();
                res.iD = (*it).m_id;
                res.iFrame = frame_count;
                trackingResult.push_back(res);
#if 1
                auto vsobj = std::make_shared<jmc_auto::perception::VisualObject>();
                (*it).get_data(*vsobj.get());
                vsobj->upper_left.x() = res.sBox.x;
                vsobj->upper_left.y() = res.sBox.y;
                int lrx = ((res.sBox.x + res.sBox.width) < width )? (res.sBox.x + res.sBox.width):(width-1);
                int lry = ((res.sBox.y + res.sBox.height) < height) ? (res.sBox.y + res.sBox.height) : (height-1);
                vsobj->lower_right.x() = lrx;
                vsobj->lower_right.y() = lry;
                objects->push_back(vsobj);
#endif
            }

            // remove dead tracklet
            if (it != trackers.end() && (*it).m_time_since_update > max_age)
            {
                it = trackers.erase(it);
            }
            else
            {
                it++;
            }
        }
        for (auto tb : trackingResult)
        {
            std::cout<<"dst: "<< tb.iFrame << " id: " << tb.iD << " x: " << tb.sBox.x << " y: " << tb.sBox.y << " w: " << tb.sBox.width << " h: " << tb.sBox.height << std::endl;
        }
    }
    return trackingResult;

}
//}
//}
