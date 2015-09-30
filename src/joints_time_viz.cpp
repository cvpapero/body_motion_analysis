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

  void jointsCb(int frame)
  {
    visualization_msgs::MarkerArray points;
    //int frame = 0;
    for(int user = 0; user < data_frame.size(); ++user)
      {

	if(data_frame[user].size() <= frame)
	  return; 

	visualization_msgs::Marker point;
  
	point.header.frame_id = "camera_link";
	point.header.stamp = ros::Time::now();
	point.action = visualization_msgs::Marker::ADD;

	stringstream ss;
	ss << "mk"<< user;
	point.ns = ss.str();
	point.id = user;

	point.type = visualization_msgs::Marker::POINTS;

	point.scale.x = 0.1;
	point.scale.y = 0.1;
	point.color = user_color[user];

	for(int i=0; i<data_frame[user][frame].size(); ++i)
	  {
	    geometry_msgs::Point p;
	    p.x = data_frame[user][frame][i].x;
	    p.y = data_frame[user][frame][i].y;
	    p.z = data_frame[user][frame][i].z;
	    point.points.push_back( p );
	  } 
	point.pose.orientation.w = 1;
 
	points.markers.push_back( point );

	/*
	cout << "user:" << user << ", data_frame.size():"<<data_frame.size()
	     << ", data_frame[0].size:"<< data_frame[0].size() << endl;
	*/
      }

    viz_pub.publish( points );

    /*
	visualization_msgs::Marker points, line_list;
	
	points.header.frame_id = line_list.header.frame_id  = "camera_link";
	points.header.stamp = line_list.header.stamp = ros::Time::now();
	
	stringstream ss;
	ss << "mk"<< i;
	points.ns = line_list.ns = ss.str();
	points.action = line_list.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
	points.id = 0;
	line_list.id = 1;
	points.type = visualization_msgs::Marker::POINTS;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	points.scale.x = 0.1;
	points.scale.y = 0.1;
	points.color = user_color[i];
	//points.color.a = 1.0;
	
	line_list.scale.x = 0.1;
	line_list.color.g = 1.0f;
	line_list.color.a = 1.0;
	
	vector<geometry_msgs::Point> joints;
	for(int j = 0; j < hm->human[i].body.joints.size(); ++j)
	  {
	    points.points.push_back( hm->human[i].body.joints[ j ].position );
	    joints.push_back( hm->human[i].body.joints[ i ].position );
	  }
 
	vector<geometry_msgs::Point> line_points;
	jointInput( joints, &line_points );

	for(int j=0; j<line_points.size(); ++j)
	  {
	    std_msgs::ColorRGBA col;
	    col.g = 1.0f;
	    line_list.colors.push_back(col);
	  }

	line_list.points = line_points; 
	//sleep(1);
	viz_pub.publish( points );
	viz_pub.publish( line_list );	
      }
    */
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joints_time_viz");
  JointsClient jc;
  ros::Rate loop(10);
  int frame = 0;
  while(ros::ok())
    {
      jc.jointsCb(frame);
      loop.sleep();
      cout << "frame:"<<frame<<endl; 
      ++frame;
    }
  
  return 0;
}
