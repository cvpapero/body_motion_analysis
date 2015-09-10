/*
加速度
1.速度を求める
2.加速度を求める
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "humans_msgs/Humans.h"
#include "geometry_msgs/Point.h"

//ファイル処理
#include <string>
#include <fstream>
#include <sstream>
#include "picojson.h"

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
  ros::Subscriber sub;
  ros::Publisher pub;  

  map<long long, Store> PointTable;
  map<long long, Store> VelTable;


public:
  MyClass()
  {
    sub = nh.subscribe("/humans/kinect_v2", 1, &MyClass::callback, this);
    pub =  nh.advertise<std_msgs::String>("/pub_topic", 1); 
  }
  
  ~MyClass()
  {
  }

  void callback(const humans_msgs::Humans::ConstPtr& msg)
  {

    for(int i=0; i<msg->human.size() ; ++i)
      {
	//ジョイントが入ってないと計算不可
	if( msg->human[i].body.joints.size() )
	  {
	    Store ps;
	    long long t_id = msg->human[i].body.tracking_id;
	    //要素を突っ込む
	    for(int j=0; j<msg->human[i].body.joints.size(); ++j)
	      {
		ps.now.push_back(msg->human[i].body.joints[j].position);
	      }
	    PointTable[t_id].now = ps.now;
	    PointTable[t_id].now_time = ros::Time::now();
	    //prevに要素が入っていたら、速度を計算する
	    if(!PointTable[t_id].prev.empty())
	      {
		//cout << PointTable[t_id].now[0].x << "," << PointTable[t_id].prev[0].x << endl;
		Store vs;
		vector<geometry_msgs::Point> vel;
		diff_calc(PointTable[t_id], &vel);
		VelTable[t_id].now = vel;
		VelTable[t_id].now_time = PointTable[t_id].now_time;
		cout << "vel:" << endl;
		output(vel);
		//加速度の計算
		if(!VelTable[t_id].prev.empty())
		  {
		    vector<geometry_msgs::Point> acc;
		    diff_calc(VelTable[t_id], &acc);
		    cout << "acc:" << endl;
		    output(acc);
		  }

		VelTable[t_id].prev = VelTable[t_id].now;
		VelTable[t_id].prev_time = VelTable[t_id].now_time;
		//VelTable[t_id] = vs;
	      }

	    PointTable[t_id].prev = PointTable[t_id].now;
	    PointTable[t_id].prev_time = PointTable[t_id].now_time;
	  }
      }
  }

  void diff_calc(Store ps, vector<geometry_msgs::Point> *vel)
  {
    ros::Duration diff = ps.now_time - ps.prev_time;
    double time = diff.toSec();
    //vs->time = time;

    //vector<double> vel;
    //vector<geometry_msgs::Point> vel;
    //cout << "time:" << time << endl;
    for(int i=0; i<ps.now.size(); ++i)
      {
	//geometry_msgs::Point v;
	double vx, vy, vz;
	geometry_msgs::Point v;
	v.x = (ps.now[i].x - ps.prev[i].x)/time;
	v.y = (ps.now[i].y - ps.prev[i].y)/time;
	v.z = (ps.now[i].z - ps.prev[i].z)/time;
	//double velocty = sqrt(vx*vx+vy*vy+vz*vz)/time; 
	//cout <<"vel[" << i <<"]: "<< velocty <<endl; 
	vel->push_back(v);
      }
    //vs->now = vel;
  }

  void output(vector<geometry_msgs::Point> input)
  {
    for(int i=0; i<input.size(); ++i)
      {
	cout <<"[" << i <<"]: "<<input[i].x<<", "<<input[i].y<<", "<<input[i].z<<endl;   
      }
  }
  /*
  void vel_calc(VelStore ps, vector<geometry_msgs::Point> *acc)
  {

  }
  */


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "acceleration");
  MyClass mc;
  ros::spin();
  return 0;
}
