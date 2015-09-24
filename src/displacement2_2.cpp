/*
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

class Store
{
public:
  vector<geometry_msgs::Point> prev;
  vector<geometry_msgs::Point> now;
  ros::Time prev_time;
  ros::Time now_time;
};


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
  bool srv_switch;
  long long attention_t_id;
  int o_id;

  double now_vel;
  //速度
  map<long long, Store> PointTable;

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
    attention_t_id = 0;
    srv_switch = false;
    p_nh.param("width", width, 1000);
    p_nh.param("height", height, 500);

    o_id = 1;
    now_vel = 0;
  }
  
  ~MyClass()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void callback(const humans_msgs::Humans::ConstPtr& msg)
  {
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
		cout<<"nomal head---x: "<<p.x<<", y: "<<p.y<<", z: "<<p.z<<endl;

		Store ps;
		//注目する関節のindex
		int ji = 10;
		ps.now.push_back(msg->human[i].body.joints[ji].position);
		//ps.now.push_back(msg->human[i].body.joints[].position);
		PointTable[t_id].now = ps.now;
		PointTable[t_id].now_time = ros::Time::now();
		//prevに要素が入っていたら、速度を計算する
		if(!PointTable[t_id].prev.empty())
		  {
		    vector<geometry_msgs::Point> vel;
		    diff_calc(PointTable[t_id], &vel);
		    //一時的にvel[0]
		    now_vel = norm_calc( vel[0] );
		    cout << "now_vel:"<< now_vel <<endl;
		  }
	    
		PointTable[t_id].prev = PointTable[t_id].now;
		PointTable[t_id].prev_time = PointTable[t_id].now_time;
	      }	      	    
	  }
      }
    if(msg->human.size() == 0)
      {
	p.x = 5.0;
	p.y = 0;
	p.z = 0;
	now_vel = 0;
	attention_t_id = 0;
	srv_switch = false;
      }
  }

  void diff_calc(Store ps, vector<geometry_msgs::Point> *vel)
  {
    ros::Duration diff = ps.now_time - ps.prev_time;
    double time = diff.toSec();

    for(int i=0; i<ps.now.size(); ++i)
      {
	geometry_msgs::Point v;
	v.x = (ps.now[i].x - ps.prev[i].x)/time;
	v.y = (ps.now[i].y - ps.prev[i].y)/time;
	v.z = (ps.now[i].z - ps.prev[i].z)/time;
	vel->push_back(v);
      }
  }

  double norm_calc(geometry_msgs::Point p)
  {
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
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

  void threadCb()
  {
    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
    
    double ox = img.cols/2;
    double oy = img.rows;
    double lg = img.rows*2/5+now_vel*100;

    if(lg > img.rows)
      lg = img.rows - 1;

    double amp = 5;
    double feq = 16/(int)p.x;
    cout << "feq:"<< feq <<endl; 
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
