/*
2015.9.30
jsonファイルを読み込んで時間軸ごとにjointのマーカをrvizで可視化する
と同時に、corrの出力結果ファイルも読み込んでいろいろしたい


フレーム数を受け付ける
ファイルからjointデータを取得する


*/

#include <ros/ros.h>
#include <humans_msgs/Humans.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include "picojson.h"
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

class JointsClient
{
private:
  ros::NodeHandle n;
  ros::NodeHandle pnh;
  ros::Publisher viz_pub;

  vector<std_msgs::ColorRGBA> user_color;
  string filename;
  vector< vector< vector<geometry_msgs::Point> > > data_frame;
  vector< double > times;

public:
  JointsClient()
    :pnh("~")
  {
    viz_pub 
      = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    std_msgs::ColorRGBA col;
    col.r = 1.0f;
    col.a = 1.0;
    user_color.push_back(col);
    col.g = 1.0f;
    col.a = 1.0;
    user_color.push_back(col);
    col.b = 1.0f;
    col.a = 1.0;
    user_color.push_back(col);
    col.r = 1.0f;
    col.g = 1.0f;
    col.a = 1.0;
    user_color.push_back(col);
    col.g = 1.0f;
    col.b = 1.0f;
    col.a = 1.0;
    user_color.push_back(col);
    col.b = 1.0f;
    col.r = 1.0f;
    col.a = 1.0;
    user_color.push_back(col);

    pnh.param<std::string>("input", filename, "test.json");

    init_data(filename, &data_frame, &times);
  }

  ~JointsClient()
  {

  }

  void init_data(string filename, vector< vector< vector<geometry_msgs::Point> > > *data_frame, 
		 vector< double > *times)
  {
    
    ifstream ifs(filename.c_str());
    if( ifs.fail() )
      {
	cerr << "Error: Input file not found!" << endl;
	return;
      }
    stringstream ss;
    string thisline;
    
    while(getline(ifs, thisline))
      {
	ss << thisline;
      }
    
    picojson::value val;
    picojson::parse(val, ss);
    picojson::array& all = val.get<picojson::array>();
    
    
    for(picojson::array::iterator it = all.begin(); 
	it != all.end(); ++it)
      {
	picojson::object& datas = it->get<picojson::object>();
	picojson::array& datas_array = datas["datas"].get<picojson::array>();
	
	vector< vector<geometry_msgs::Point> > thetas;
	for(picojson::array::iterator it2 = datas_array.begin(); 
	    it2 != datas_array.end(); ++it2)
	  {
	    picojson::object& data = it2->get<picojson::object>();
	    picojson::array& data_array = data["jdata"].get<picojson::array>();
	    
	    vector<geometry_msgs::Point> theta;
	    for(picojson::array::iterator it3 = data_array.begin(); 
		it3 != data_array.end(); ++it3)
	      {

		picojson::array pa = it3->get<picojson::array>();
		geometry_msgs::Point p;
		p.x = pa[0].get<double>();
		p.y = pa[1].get<double>();
		p.z = pa[2].get<double>();
		
		theta.push_back(p);
	      }
	    thetas.push_back(theta);
	    
	    times->push_back(data["time"].get<double>());
	  }
	
	data_frame->push_back(thetas);
      }
  }
  
  /*
  void jointInput(vector<geometry_msgs::Point> joints, vector<geometry_msgs::Point> *line_points)
  {

    line_points->push_back(joints[0]);
    line_points->push_back(joints[1]);

    line_points->push_back(joints[1]);
    line_points->push_back(joints[20]);

    line_points->push_back(joints[20]);
    line_points->push_back(joints[2]);

    line_points->push_back(joints[2]);
    line_points->push_back(joints[3]);

    line_points->push_back(joints[20]);
    line_points->push_back(joints[8]);

    line_points->push_back(joints[8]);
    line_points->push_back(joints[9]);

    line_points->push_back(joints[9]);
    line_points->push_back(joints[10]);

    line_points->push_back(joints[10]);
    line_points->push_back(joints[11]);

    line_points->push_back(joints[11]);
    line_points->push_back(joints[23]);

    line_points->push_back(joints[11]);
    line_points->push_back(joints[24]);

    line_points->push_back(joints[20]);
    line_points->push_back(joints[4]);

    line_points->push_back(joints[4]);
    line_points->push_back(joints[5]);

    line_points->push_back(joints[5]);
    line_points->push_back(joints[6]);

    line_points->push_back(joints[6]);
    line_points->push_back(joints[7]);

    line_points->push_back(joints[7]);
    line_points->push_back(joints[21]);

    line_points->push_back(joints[7]);
    line_points->push_back(joints[22]);

    line_points->push_back(joints[0]);
    line_points->push_back(joints[16]);

    line_points->push_back(joints[16]);
    line_points->push_back(joints[17]);

    line_points->push_back(joints[17]);
    line_points->push_back(joints[18]);

    line_points->push_back(joints[18]);
    line_points->push_back(joints[19]);

    line_points->push_back(joints[0]);
    line_points->push_back(joints[12]);

    line_points->push_back(joints[12]);
    line_points->push_back(joints[13]);

    line_points->push_back(joints[13]);
    line_points->push_back(joints[14]);

    line_points->push_back(joints[14]);
    line_points->push_back(joints[15]);   

  }
  */

  void jointsCb(int *frame)
  {
    visualization_msgs::MarkerArray points;
    visualization_msgs::MarkerArray lines;
    //int frame = 0;
    for(int user = 0; user < data_frame.size(); ++user)
      {

	if(data_frame[user].size() <= *frame)
	  *frame = 0; 

	visualization_msgs::Marker point;
	visualization_msgs::Marker line;
  
	point.header.frame_id = "camera_link";
	point.header.stamp = ros::Time::now();
	point.action = visualization_msgs::Marker::ADD;

	line.header.frame_id = "camera_link";
	line.header.stamp = ros::Time::now();
	line.action = visualization_msgs::Marker::ADD;

	stringstream pss;
	pss << "point_"<< user;
	point.ns = pss.str();
	point.id = user;

	stringstream lss;
	lss << "line_"<< user;
	line.ns = lss.str();
	line.id = user;

	point.type = visualization_msgs::Marker::POINTS;
	line.type = visualization_msgs::Marker::LINE_LIST;

	point.scale.x = 0.1;
	point.scale.y = 0.1;
	point.color = user_color[user];

	line.scale.x = 0.1;
	//.scale.y = 0.1;
	line.color = user_color[user];

	vector<geometry_msgs::Point> joints;
	for(int i=0; i<data_frame[user][*frame].size(); ++i)
	  {
	    geometry_msgs::Point p;
	    p.x = data_frame[user][*frame][i].x;
	    p.y = data_frame[user][*frame][i].y;
	    p.z = data_frame[user][*frame][i].z;
	    point.points.push_back( p );

	    joints.push_back( p );
	  } 

	point.pose.orientation.w = 1;
	points.markers.push_back( point );

	/*
	cout << "user:" << user << ", data_frame.size():"<<data_frame.size()
	     << ", data_frame[0].size:"<< data_frame[0][*frame].size() << endl;
	*/
      }

    viz_pub.publish( points );
  
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joints_time_viz");
  JointsClient jc;
  ros::Rate loop(5);
  int frame = 0;
  while(ros::ok())
    {
      jc.jointsCb(&frame);
      loop.sleep();
      cout << "frame:"<<frame<<endl; 
      ++frame;
    }
  
  return 0;
}
