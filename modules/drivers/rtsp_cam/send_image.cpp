#include <sstream>
#include "ros/ros.h"
#include <mutex>
#include "std_msgs/String.h"
#include "transdata/transdata.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <thread>

Transdata transdata;
std::mutex mImage_buf;
int	ready_flag=-1;

void	Reci_Deal(void)
{
	while(1)
	{
			//;
		ready_flag=transdata.Transdata_Recdata();
		//cout << "thread \n";
		usleep(1000*30);
	}
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"rtsp_cam_node");

    ros::NodeHandle n;

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("/jmc_auto/sensor/camera/roundview", 1);
    ros::Rate loop_rate(30);

    int count = 0;
    sensor_msgs::ImagePtr msg;

    if(transdata.Transdata_init() < 0)
    {
        //cout <<"init error !" << endl;
		ROS_ERROR("transdata init error.\n");
        return -1;
    }
    //std::thread t1(Reci_Deal);
	ROS_INFO("start rtsp decode.\n");
    while(ros::ok())
    {
        //?�收图像并显示
	auto time = ros::Time::now();
        ready_flag=transdata.Transdata_Recdata();
	#if	1
        //mImage_buf.lock();
        if(ready_flag==0&&(!transdata.image_test.empty()))
        {
	    	ready_flag=-1;
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", transdata.image_test).toImageMsg();
	    msg->header.stamp = time;
            pub.publish(msg);
            //cout << " send image " << count << endl;
            count ++;
            transdata.image_test.release();
        }
        //mImage_buf.unlock();
	#endif
        ros::spinOnce();
	loop_rate.sleep();
    }
    //t1.join();
    return 0;
}
