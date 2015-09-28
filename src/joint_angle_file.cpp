/*
2015.9.28---
ファイルに出力されたkinectv2のデータを、関節角に変換する
その際、tracking_idごとにjson形式でファイル出力する
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "humans_msgs/Humans.h"

//ファイル処理
#include <string>
#include <fstream>
#include <sstream>
#include "picojson.h"

using namespace std;

class IndexArr
{
public:
  int a;
  int b;
  int c;
};

class MyClass
{
private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;  

  vector< vector<int> > joint_table;

  vector< vector<double> > thetas_frame;

  map<long long, vector< vector<double> > > person_thetas;

  map<int, int> test;

  string filename;


public:
  MyClass()
  {
    sub = nh.subscribe("/humans/kinect_v2", 1, &MyClass::callback, this);
    pub =  nh.advertise<std_msgs::String>("/pub_topic", 1); 
    filename = "joint_index.txt";
    init_table(filename);
  }
  
  ~MyClass()
  {
    output_table();
  }

  //ファイルから関節角度のインデックスを取得する
  void init_table(string filename)
  {
    ifstream ifs(filename.c_str());
    string str;
    int p;

    if( ifs.fail() )
      {
	cerr << "Error: Input file not found!" << endl;
	return;
      }

    while(getline(ifs, str))
      {
	if ((p=str.find("//")) != str.npos)
	  continue;
	vector<int> inner;

	while( (p=str.find(",")) !=str.npos )
	  {
	    inner.push_back(atoi(str.substr(0, p).c_str()));
	    str = str.substr(p+2);
	  }
	inner.push_back(atoi(str.c_str()));
	joint_table.push_back(inner);
      }

    //for(int j=0; j<joint_table.size(); j++)
    // cout<<joint_table[j][0]<<", "<<joint_table[j][1]<<", "<<joint_table[j][2]<< endl;
  }
  
  //データ構造を持つので、jsonを使って保存する
  //データベースは大げさ過ぎる
  void output_table()
  {
    string output;

    //picojson::object rows;
    picojson::object obj;
    picojson::array rows;
    for(int i=0; i<thetas_frame.size(); ++i)
      {
	picojson::array col;
	for(int j=0; j<thetas_frame[i].size(); ++j)
	  {
	    col.push_back(picojson::value(thetas_frame[i][j]));
	  }
	stringstream ss;
	ss << i;
	//rows.insert(make_pair(ss.str().c_str(), col));
	//rows.insert(col);
	rows.push_back(picojson::value(col));
      }
    obj.insert(make_pair("data",rows));
    //picojson::value data = picojson::value(obj);
    picojson::value data = picojson::value(obj);
    output = data.serialize();

    ofstream ofs("output.json");
    ofs << output;
  }

  void callback(const humans_msgs::Humans::ConstPtr& msg)
  {

    for(int i=0; i<msg->human.size() ; ++i)
      {
	//ジョイントが入ってないと計算不可
	if( msg->human[i].body.joints.size() )
	  {
	    //longlong t_id = msg->human[i].body.tracking_id;
	    //persons_theta[t_id]
	    vector<double> thetas;
	    joint_calc(msg->human[i].body, &thetas);
	    thetas_frame.push_back(thetas);
	  }
      }
  }

  //角関節の角度を計算する
  void joint_calc(humans_msgs::Body body, vector<double> *thetas)
  {
    vector<geometry_msgs::Point> jpoints;
    //vector<double> thetas;
    jpoints = pnt_store( body.joints );

    for(int i=0; i<joint_table.size(); ++i)
      {
	geometry_msgs::Point a_v, c_v;
	double cos_theta, theta;
	a_v = vec_gen(jpoints[joint_table[i][0]], jpoints[joint_table[i][1]]);
	c_v = vec_gen(jpoints[joint_table[i][2]], jpoints[joint_table[i][1]]);
	cos_theta 
	  = (a_v.x*c_v.x + a_v.y*c_v.y + a_v.z*c_v.z)
	  / (sqrt(a_v.x*a_v.x + a_v.y*a_v.y + a_v.z*a_v.z)
	     *sqrt(c_v.x*c_v.x + c_v.y*c_v.y + c_v.z*c_v.z));
	theta = acos(cos_theta);
	//cout << i << ", " << theta * (180/M_PI) << endl;
	thetas->push_back(theta);
      }
    //thetas_frame.push_back(thetas);
  }

  vector<geometry_msgs::Point> pnt_store(vector<humans_msgs::Joints> jt)
  {
    vector<geometry_msgs::Point> jps;
    for(int i=0; i<jt.size(); ++i)
      {
	jps.push_back( jt[i].position );
      }
    return jps;
  }

  geometry_msgs::Point vec_gen(geometry_msgs::Point v1, geometry_msgs::Point v2)
  {
    geometry_msgs::Point vec; 
    vec.x = v1.x - v2.x;
    vec.y = v1.y - v2.y;
    vec.z = v1.z - v2.z;
    return vec;
  }
  /*
  vector<geomtry_msgs::Point> vec_store(vector<geomtry_msgs::Point> ps)
  {
    vector<geomtry_msgs::Point> pa;

    for(int i=0; i<joint_table.size(); ++i)
      {

      }
    return pa;
  }
  */

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_angle");
  MyClass mc;
  ros::spin();
  return 0;
}
