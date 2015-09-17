/*
原点からの変位を求める
位置座標の原点は？カメラの位置では？？

*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "humans_msgs/Humans.h"
#include "geometry_msgs/Point.h"

//ファイル処理
#include <string>
#include <fstream>
#include <sstream>
#include "picojson.h"

//描画
//#include <boost/thread.hpp>
//#include <functional>
//#include <algorithm>
//#include <numeric>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define HEAD 3
#define MAXSIZE 24

static const std::string OPENCV_WINDOW = "Image window";
using namespace std;


class MyClass
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher dispub;  
  //boost::thread* thread;
  geometry_msgs::Point p;
  double micro;
  bool pm;
  int index;
  vector<double> diff;

public:
  MyClass()
  {
    sub = nh.subscribe("/humans/kinect_v2", 1, &MyClass::callback, this);
    dispub =  nh.advertise<std_msgs::Float64>("/displacement", 1); 
    
    p.x = 2.0;
    p.y = 0;
    p.z = 0;
    cv::namedWindow(OPENCV_WINDOW);

    micro = 5;
    pm = true;
    index = 0;

    diff.push_back(6);
    diff.push_back(6);
    diff.push_back(6);
    diff.push_back(6);
    diff.push_back(6);
    diff.push_back(6);
    diff.push_back(6);
    diff.push_back(6);
    diff.push_back(5);
    diff.push_back(5);
    diff.push_back(5);
    diff.push_back(5);
    diff.push_back(4);
    diff.push_back(4);
    diff.push_back(3);
    diff.push_back(2);
    diff.push_back(1);
    diff.push_back(-1);
    diff.push_back(-2);
    diff.push_back(-3);
    diff.push_back(-4);
    diff.push_back(-4);
    diff.push_back(-5);
    diff.push_back(-5);
    diff.push_back(-5);
    diff.push_back(-5);
    diff.push_back(-6);
    diff.push_back(-6);
    diff.push_back(-6);
    diff.push_back(-6);
    diff.push_back(-6);
    diff.push_back(-6);
    diff.push_back(-6);
    diff.push_back(-6);
  }
  
  ~MyClass()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void callback(const humans_msgs::Humans::ConstPtr& msg)
  {
    for(int i=0; i<msg->human.size() ; ++i)
      {
	//ジョイントが入ってないと計算不可
	if( msg->human[i].body.joints.size() )
	  {
	    long long t_id = msg->human[i].body.tracking_id;

	    p = msg->human[i].body.joints[HEAD].position;


	    cout<<"head---x: "<<p.x<<", y: "<<p.y<<", z: "<<p.z<<endl;

	    //ループ回すのにかかる時間
	    //ros::Duration diff =  PointTable[t_id].now_time -  PointTable[t_id].prev_time;
	    //double time = diff.toSec();
	    //cout << "time: "<<time << endl;
	    
	  }
      }
    if(msg->human.size() == 0)
      {
	p.x = 2.0;
	p.y = 0;
	p.z = 0;
      }
  }

  void threadCb()
  {
    cv::Mat img = cv::Mat::zeros(500, 1000, CV_8UC3);
    
    double ox = img.cols/2;
    double oy = img.rows;
    int lg = 300;

    int d_index;
    if(p.x <= 0.8)
      d_index = 4;
    else if(p.x > 0.8 && p.x <= 1)
      d_index = 3;
    else if(p.x > 1 && p.x <= 1.2)
      d_index = 2;
    else if(p.x > 1.2 && p.x <= 1.5)
      d_index = 1;
    else if(p.x > 1.5) 
      d_index = 0;

    if( pm )
      {
	index = index + (1+d_index);
	if(index > diff.size())
	  {
	    index = diff.size()-1;
	    pm = false;
	  }
      }
    else
      {
	index = index - (1+d_index);
	if(index < 0)
	  {
	    index = 0;
	    pm = true;
	  }
      }

    //cout << "index:" << index <<", d_index:" << d_index << ", p.x:" << p.x << endl;
    double m = diff[index];

    double theta = atan2(p.x, p.y) + m*M_PI/180;
    double diff_x = lg*cos(theta);
    double diff_y = lg*sin(theta);

    cv::line(img, cv::Point(ox, oy), cv::Point(ox+diff_x, oy-diff_y), cv::Scalar(0,0,200), 6, 4); 
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(1);
  }

  int GetRandom(int min,int max)
  {
    return min + (int)(rand()*(max-min+1.0)/(1.0+RAND_MAX));
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "displacement");
  MyClass mc;
  //ros::spin();
  while(ros::ok())
    {
      ros::spinOnce();
      mc.threadCb();
    }
  return 0;
}
