/*
2015.9.28---
ファイルに出力されたkinectv2のデータを、関節角に変換する
その際、tracking_idごとにjson形式でファイル出力する

時間合わせをする。
つまり、ファイルに出力する機能を作る

終了できない！なぜ？
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "humans_msgs/Humans.h"

//ファイル処理
#include <iostream>
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

class Data
{
public:
  vector<double> data;
  double secs;
};


class JData
{
public:
  vector<geometry_msgs::Point> data;
  double secs;
};


class MyClass
{
private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Subscriber sub;
  ros::Publisher pub;  

  vector< vector<int> > joint_table;

  vector< vector<double> > thetas_frame;

  map<long long, vector< vector<double> > > person_thetas;
  map< long long, vector<Data> > persons;
  map< long long, vector<JData> > jpersons;

  map<int, int> test;

  string joint_filename;
  string output_filename;
  stringstream out_file;

public:
  MyClass()
    :pnh("~")
  {
    sub = nh.subscribe("/humans/kinect_v2", 1, &MyClass::callback, this);
    pub =  nh.advertise<std_msgs::String>("/pub_topic", 1); 

    pnh.param<std::string>("output", output_filename, "test.json");
    out_file << output_filename << ".json";

    joint_filename = "joint_index.txt";
    init_table(joint_filename);
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

    picojson::array users;

    map< long long, vector<Data> >::iterator it =  persons.begin();
    map< long long, vector<JData> >::iterator jit =  jpersons.begin();

    while(it != persons.end())
      { 
	picojson::object objs;
	picojson::array rows;
	cout << "second.size(): "<< it->second.size()
	     << ", second[0].data.size(): "<< it->second[0].data.size() << endl;
	for(int i=0; i<it->second.size(); ++i)
	  {
	    picojson::object obj;
	    picojson::array col;
	    picojson::array jcol;
	    for(int j=0; j<it->second[i].data.size(); ++j)
	      {
		col.push_back(picojson::value(it->second[i].data[j]));

		picojson::array jjcol;
		jjcol.push_back(picojson::value(jit->second[i].data[j].x));
		jjcol.push_back(picojson::value(jit->second[i].data[j].y));
		jjcol.push_back(picojson::value(jit->second[i].data[j].z));
		jcol.push_back(picojson::value(jjcol));
	      }

	    obj.insert(make_pair("data",col));
	    obj.insert(make_pair("jdata",jcol));
	    obj.insert(make_pair("time",it->second[i].secs));
	    rows.push_back(picojson::value(obj));
	  }
	objs.insert(make_pair("datas",rows));

	stringstream ss;
	ss << it->first;
	cout << ss.str() << endl;
	objs.insert(make_pair("t_id",ss.str()));

	users.push_back(picojson::value(objs));
	++it;
	++jit;
      }

    picojson::value data = picojson::value(users);
    output = data.serialize();

    ofstream ofs(out_file.str().c_str());
    ofs << output;

    cout << "output_filename:"<<out_file.str()<<endl;
  }

  void callback(const humans_msgs::Humans::ConstPtr& msg)
  {
    double now_sec = ros::Time::now().toSec(); 
    for(int i=0; i<msg->human.size() ; ++i)
      {
	//ジョイントが入ってないと計算不可
	if( msg->human[i].body.joints.size() )
	  {
	    long long t_id = msg->human[i].body.tracking_id;
	    cout << "t_id: "<< t_id << endl;
	    vector<double> thetas;
	    vector<geometry_msgs::Point> joints;

	    joint_calc(msg->human[i].body , &thetas, &joints);
	    Data data;
	    data.data = thetas;
	    data.secs = now_sec;
	    persons[t_id].push_back(data);

	    //vector<geometry_msgs::Point> joints;
	    //joint_store(msg->human[i].body, &joints);
	    JData jdata;
	    jdata.data = joints;
	    data.secs = now_sec;
	    jpersons[t_id].push_back(jdata);
	  }
      }
  }

  /*
  void joint_store(humans_msgs::Body body, vector<double> *thetas)
  {

  }
  */

  //角関節の角度を計算する
  void joint_calc(humans_msgs::Body body, vector<double> *thetas, vector<geometry_msgs::Point> *joints)
  {
    vector<geometry_msgs::Point> jpoints;
    //vector<double> thetas;
    jpoints = pnt_store( body.joints );
    *joints = jpoints;

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
