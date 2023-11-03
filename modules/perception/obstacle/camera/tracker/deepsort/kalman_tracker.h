
#ifndef _KALMANTRACKER_H_
#define _KALMANTRACKER_H_

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

typedef struct
{
	float x;
	float y;
	float width;
	float height;
}TRACK_OBJ_RECT_S;

typedef struct TrackingBox
{
	int iFrame;
	int iD;
	TRACK_OBJ_RECT_S sBox;
}TrackingBox;

//template<typename T> int KalmanTracker<T>::kf_count = 0;
static int kf_count = 0;

// This class represents the internel state of individual tracked objects observed as bounding box.
template<class T>
class KalmanTracker
{
public:
    using data_type = T;
	KalmanTracker(TRACK_OBJ_RECT_S _pstInitRect)
	{
		init_kf(_pstInitRect);
		m_time_since_update = 0;
		m_hits = 0;
		m_hit_streak = 0;
		m_age = 0;
		m_id = kf_count;
		kf_count++;
	}

	~KalmanTracker()
	{
		m_history.clear();
	}

    // initialize Kalman filter
    void init_kf(TRACK_OBJ_RECT_S _pstStateMat)
    {
        int stateNum = 7;
        int measureNum = 4;
        kf = KalmanFilter(stateNum, measureNum, 0);

        measurement = Mat::zeros(measureNum, 1, CV_32F);
    #if 0
        kf.transitionMatrix = *(Mat_<float>(stateNum, stateNum) <<
            1, 0, 0, 0, 1, 0, 0,
            0, 1, 0, 0, 0, 1, 0,
            0, 0, 1, 0, 0, 0, 1,
            0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 1);
    #else
        float mm[]={1, 0, 0, 0, 1, 0, 0,
            0, 1, 0, 0, 0, 1, 0,
            0, 0, 1, 0, 0, 0, 1,
            0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 0, 1};
        cv::Mat(stateNum,stateNum,CV_32FC1,mm).copyTo(kf.transitionMatrix);
        std::cout << kf.transitionMatrix << std::endl;
    #endif
        setIdentity(kf.measurementMatrix);
        setIdentity(kf.processNoiseCov, Scalar::all(1e-2));
        setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
        setIdentity(kf.errorCovPost, Scalar::all(1));

        // initialize state vector with bounding box in [cx,cy,s,r] style
        kf.statePost.at<float>(0, 0) = _pstStateMat.x + _pstStateMat.width / 2;
        kf.statePost.at<float>(1, 0) = _pstStateMat.y + _pstStateMat.height / 2;
        kf.statePost.at<float>(2, 0) = _pstStateMat.height * _pstStateMat.width;
        kf.statePost.at<float>(3, 0) = _pstStateMat.width / _pstStateMat.height;
    }

    // Predict the estimated bounding box.
    TRACK_OBJ_RECT_S predict()
    {
        // predict
        Mat p = kf.predict();
        TRACK_OBJ_RECT_S sTrackRect = { 0 };
        m_age += 1;

        if (m_time_since_update > 0)
            m_hit_streak = 0;
        m_time_since_update += 1;

        TRACK_OBJ_RECT_S predictBox = get_rect_xysr(p.at<float>(0, 0), p.at<float>(1, 0), p.at<float>(2, 0), p.at<float>(3, 0));

        m_history.push_back(predictBox);
        sTrackRect.x = predictBox.x;
        sTrackRect.y = predictBox.y;
        sTrackRect.height = predictBox.height;
        sTrackRect.width = predictBox.width;
        return sTrackRect;
    }


    // Update the state vector with observed bounding box.
    void update(TRACK_OBJ_RECT_S* _StateMat)
    {
        m_time_since_update = 0;
        m_history.clear();
        m_hits += 1;
        m_hit_streak += 1;

        // measurement
        measurement.at<float>(0, 0) = _StateMat->x + _StateMat->width / 2;
        measurement.at<float>(1, 0) = _StateMat->y + _StateMat->height / 2;
        measurement.at<float>(2, 0) = (_StateMat->width * _StateMat->height);
        measurement.at<float>(3, 0) = _StateMat->width / _StateMat->height;

        // update
        kf.correct(measurement);
    }


    // Return the current state vector
    TRACK_OBJ_RECT_S get_state()
    {
        Mat s = kf.statePost;
        return get_rect_xysr(s.at<float>(0, 0), s.at<float>(1, 0), s.at<float>(2, 0), s.at<float>(3, 0));
    }


    // Convert bounding box from [cx,cy,s,r] to [x,y,w,h] style.

    TRACK_OBJ_RECT_S get_rect_xysr(float cx, float cy, float s, float r)
    {
        TRACK_OBJ_RECT_S st = {0};
        float w = sqrt(s * r);
        float h = s / w;
        float x = (cx - w / 2);
        float y = (cy - h / 2);

        if (x < 0 && cx > 0)
            x = 0;
        if (y < 0 && cy > 0)
            y = 0;
        st.x = x;
        st.y = y;
        st.width = w;
        st.height = h;
        return st;
    }

    void set_data(const data_type &data)
    {
        //std::cout<<"data size: "<<sizeof(data_type)<<std::endl;
        //std::cout<<"m_data size: "<<sizeof(m_data)<<std::endl;
        //std::cout<<"*data size: "<<sizeof(*data)<<std::endl;
        //memcpy((void*)&m_data,(void*)data,sizeof(data));
        m_data.clear();
        m_data.push_back(data);
    }

    void get_data(data_type &data)
    {
        data = m_data[0];
    }

	int m_time_since_update;
	int m_hits;
	int m_hit_streak;
	int m_age;
	int m_id;

private:
	cv::KalmanFilter kf;
	cv::Mat measurement;

	std::vector<TRACK_OBJ_RECT_S> m_history;
    std::vector<T> m_data;
};

#endif
