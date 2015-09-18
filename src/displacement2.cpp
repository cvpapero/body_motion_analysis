/*
2015.9.19
特定の人物に中止する仕組みを考える


パラメータを設定してみる
あと、今の角度って、どうなんだろう。パラパラしすぎな気がする。もっとなめらかにできないか？
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

#include "humans_msgs/DatabaseSrv.h"

#define HEAD 3
#define MAXSIZE 24

static const std::string OPENCV_WINDOW = "Image window";
using namespace std;


class MyClass
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle p_nh;

  ros::Subscriber sub;
  ros::Subscriber recog_sub;

  ros::ServiceServer srv;

  ros::Publisher dispub;  
  geometry_msgs::Point p;

  int index;
  int width, height;
  map<int, long long> okao_t_id;

public:
  MyClass()
    : p_nh("~")
  {
    sub = nh.subscribe("/humans/kinect_v2", 1, &MyClass::callback, this);
    recog_sub = nh.subscribe("/humans/recog_info", 1, &MyClass::recogCallback, this);
    dispub =  nh.advertise<std_msgs::Float64>("/displacement", 1); 

    srv = nh.advertiseService("displace", &MyClass::Service, this);    

    p.x = 5.0;
    p.y = 0;
    p.z = 0;
    cv::namedWindow(OPENCV_WINDOW);

    index = 0;

    p_nh.param("width", width, 1000);
    p_nh.param("height", height, 500);

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
	    cout<<"nomal head---x: "<<p.x<<", y: "<<p.y<<", z: "<<p.z<<endl;
	    //ループ回すのにかかる時間
	    //ros::Duration diff =  PointTable[t_id].now_time -  PointTable[t_id].prev_time;
	    //double time = diff.toSec();
	    //cout << "time: "<<time << endl;	    
	  }
      }
    if(msg->human.size() == 0)
      {
	p.x = 5.0;
	p.y = 0;
	p.z = 0;
      }
  }

  void recogCallback(const humans_msgs::Humans::ConstPtr& msg)
  {
    for(int i=0; i<msg->human.size() ; ++i)
      {
	//ジョイントが入ってないと計算不可
	if( msg->human[i].body.joints.size() )
	  {
	    int o_id = msg->human[i].max_okao_id;
	    long long t_id = msg->human[i].body.tracking_id;
	    okao_t_id[o_id] = t_id;
	    //p = msg->human[i].body.joints[HEAD].position;
	    //cout<<"recog head---x: "<<p.x<<", y: "<<p.y<<", z: "<<p.z<<endl;	    
	  }
      }
    //cout << test << endl;
  }

  bool Service(humans_msgs::DatabaseSrv::Request &req,
	       humans_msgs::DatabaseSrv::Response &res)
  {
/*
    if( req.rule == "p" )
      {
	//particularPersonPosition( req.person.okao_id ); 
      }
    else if( req.rule == "l" )
      {
	//lastPeoplePosition();
      }
    else
      {
	ROS_INFO("no match rule!");
	return false;
      }
*/
cout << "okao_id:"<< req.person.okao_id << okao_t_id[req.person.okao_id]<<endl;
    return true;
  }

  void threadCb()
  {
    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
    
    double ox = img.cols/2;
    double oy = img.rows;
    int lg = img.rows*3/5;

    double amp = 5;
    double feq = 20/p.x;

    //double micro_amp = 5;
    //double micro_feq = 10;

    double m = amp*sin(feq*index*M_PI/180);

    //double micro = micro_amp*sin(micro_feq*index*M_PI/180);

    double theta = atan2(p.x, p.y) + m*M_PI/180;
    double diff_x = lg*cos(theta);
    double diff_y = lg*sin(theta);

    cv::line(img, cv::Point(ox, oy), cv::Point(ox + diff_x, oy - diff_y), cv::Scalar(0,0,200), 6, 4); 
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(1);
    ++index;
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
