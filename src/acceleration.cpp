/*
加速度
1.速度を求める
2.加速度を求める


もっとも速い速さを出力する
速さの平均を出力する
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
  ros::Publisher velpub;  
  ros::Publisher accpub;

  map<long long, Store> PointTable;
  map<long long, Store> VelTable;

  int joint_index;

public:
  MyClass()
  {
    sub = nh.subscribe("/humans/kinect_v2", 1, &MyClass::callback, this);
    velpub =  nh.advertise<std_msgs::Float64>("/speed", 1); 
    accpub =  nh.advertise<std_msgs::Float64>("/acceler", 1); 

    joint_index = 10;
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
		//cout << "vel:" << endl;
		//output(vel);
		//加速度の計算
		if(!VelTable[t_id].prev.empty())
		  {
		    vector<geometry_msgs::Point> acc;
		    diff_calc(VelTable[t_id], &acc);
		    //cout << "acc:" << acc[joint_index].x << ", " 
		    //	 << acc[joint_index].y << ", " << acc[joint_index].z << endl;
		    //output(acc);

		    /*
		    std_msgs::Float64 ac_msg;
		    ac_msg.data = 0;
		    int max_acc_index = 25;
		    for(int k=0; k<acc.size(); ++k)
		      {
			double tmp_acc = norm_calc( acc[k] );
			if(fabs(ac_msg.data) < fabs(tmp_acc))
			  {
			    // ac_msg.data = tmp_acc;
			    max_acc_index = k;
			  } 
		      }
		    //ac_msg.data = sum_calc( acc );
		    cout << "max_acc_index:" << max_acc_index <<", max_acc"<< ac_msg.data<< endl;
		    accpub.publish(ac_msg);
		    */
		  }

		VelTable[t_id].prev = VelTable[t_id].now;
		VelTable[t_id].prev_time = VelTable[t_id].now_time;
		//VelTable[t_id] = vs;

		std_msgs::Float64 sp_msg;
		sp_msg.data = 0;
		//最大値を探す
		int max_index = 25;
		for(int k=0; k<VelTable[t_id].now.size(); ++k)
		  {
		    if(index_check(k))
		      {
			double tmp_vel = norm_calc( VelTable[t_id].now[k] );
			if(sp_msg.data < tmp_vel)
			  {
			    sp_msg.data = tmp_vel;
			    max_index = k;
			  }
		      }
		  }
		cout << "t_id:" << t_id<< ", max_vel_index:" << max_index <<", max_vel:"<< sp_msg.data<< endl;

		//もし合計値でやるなら
		//sp_msg.data = sum_calc( VelTable[t_id].now );
		//もし値を指定するなら
		//sp_msg.data = norm_calc( VelTable[t_id].now[18] );

		velpub.publish(sp_msg);
	      }

	    //ループ回すのにかかる時間
	    ros::Duration diff =  PointTable[t_id].now_time -  PointTable[t_id].prev_time;
	    double time = diff.toSec();
	    cout << "time: "<<time << endl;
	    
	    PointTable[t_id].prev = PointTable[t_id].now;
	    PointTable[t_id].prev_time = PointTable[t_id].now_time;
	  }
      }
  }

  bool index_check(int k)
  {
    if(k==15 || k== 19 || k==21 || k==22 || k==23 || k==24)
      {
	//cout << "this index is:"<<k<<endl;
	return false;
      }
    else
      return true;
  }
  double norm_calc(geometry_msgs::Point p)
  {
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
  }

  double sum_calc(vector<geometry_msgs::Point> ps)
  {
    double sum;
    for(int i=0; i<ps.size(); ++i)
      {
	sum += norm_calc(ps[i]);
      }

    return sum;
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
