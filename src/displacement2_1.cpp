/*
2015.9.22
delayを生じさせる
stack構造(vector)を使ってみる
もしかしたら、これ失敗かもしれん

わからん、、、どうすればいいんだろうか

2015.9.19
特定の人物に注視する仕組みを考える
パラメータを設定してみる
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
  //geometry_msgs::Point p;
  geometry_msgs::Point pt;
  vector<geometry_msgs::Point> p_buf;

  int index;
  int width, height;
  map<int, long long> okao_t_id;
  bool srv_switch;
  long long attention_t_id;
  int o_id;

  ros::Time prev_time;
  ros::Time now_time;
  double count_time;

public:
  MyClass()
    : p_nh("~")
  {
    sub = nh.subscribe("/humans/kinect_v2", 1, &MyClass::callback, this);
    recog_sub = nh.subscribe("/humans/recog_info", 1, &MyClass::recogCallback, this);
    dispub =  nh.advertise<std_msgs::Float64>("/displacement", 1); 

    srv = nh.advertiseService("displace", &MyClass::Service, this);    

    pt.x = 5.0;
    pt.y = 0;
    pt.z = 0;
    cv::namedWindow(OPENCV_WINDOW);

    index = 0;
    attention_t_id = 0;
    srv_switch = false;
    p_nh.param("width", width, 1000);
    p_nh.param("height", height, 500);

    o_id = 0;

    prev_time = ros::Time::now();
    now_time = ros::Time::now();
    count_time = 0;
  }
  
  ~MyClass()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void callback(const humans_msgs::Humans::ConstPtr& msg)
  {
    geometry_msgs::Point p;
    for(int i=0; i<msg->human.size() ; ++i)
      {
	//ジョイントがないと不可
	if( msg->human[i].body.joints.size() )
	  {
	    long long t_id = msg->human[i].body.tracking_id;

	    //注目しているtracking_idか
	    if( attention_t_id == t_id )
	      {
		p = msg->human[i].body.joints[HEAD].position;
		p_buf.push_back(p);
		cout<<"nomal head---x: "<<p.x<<", y: "<<p.y<<", z: "<<p.z<<endl;

		ros::Duration diff = now_time - prev_time;
		//cout << diff.toSec() << endl;
		count_time += diff.toSec();
		cout << count_time << endl;
 
		if(count_time > 5.0)
		  {
		    pt = p_buf.front();
		    if(!p_buf.empty())
		    {
		      p_buf.erase( p_buf.begin()+1 );
		    }
		    cout << "p_buf size:"<< p_buf.size() << endl;
		  }

	      }	      	  	    
	  }
      }
    if(msg->human.size() == 0)
      {
	pt.x = 5.0;
	pt.y = 0;
	pt.z = 0;
	attention_t_id = 0;
	srv_switch = false;
	count_time = 0;

      }

    prev_time = now_time;
    now_time = msg->header.stamp;

 
  }

  void recogCallback(const humans_msgs::Humans::ConstPtr& msg)
  {
    for(int i=0; i<msg->human.size() ; ++i)
      {
	//ジョイントがないと不可
	if( msg->human[i].body.joints.size() )
	  {
	    if( o_id == msg->human[i].face.persons[0].okao_id )
	      {
		attention_t_id = msg->human[i].body.tracking_id;
	      }  
	  }
      }
  }

  bool Service(humans_msgs::DatabaseSrv::Request &req,
	       humans_msgs::DatabaseSrv::Response &res)
  {
    //ここではokao_idを受けるのみ
    cout << "okao_id:"<< req.person.okao_id << endl;
    o_id = req.person.okao_id;
    srv_switch = true;
    return true;
  }

  
  void time_checker()
  {
    if(count_time > 5.0)
      {
	pt = p_buf.front();
	if(!p_buf.empty())
	  p_buf.erase( p_buf.begin()+1 );
      }
  }

  void threadCb()
  {

    //time_checker();
    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
    
    double ox = img.cols/2;
    double oy = img.rows;
    int lg = img.rows*3/5;

    double amp = 5;
    double feq = 15/pt.x;

    //double micro_amp = 5;
    //double micro_feq = 10;

    double m = amp*sin(feq*index*M_PI/180);

    //double micro = micro_amp*sin(micro_feq*index*M_PI/180);

    double theta = atan2(pt.x, pt.y) + m*M_PI/180;
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
