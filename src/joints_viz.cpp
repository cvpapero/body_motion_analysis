#include <ros/ros.h>
#include <humans_msgs/Humans.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

using namespace std;

class JointsClient
{
private:
  ros::NodeHandle n;
  ros::Publisher viz_pub;
  ros::Subscriber kinect_sub;
  vector<std_msgs::ColorRGBA> user_color;

public:
  JointsClient()
  {
    viz_pub 
      = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    kinect_sub = n.subscribe("/humans/kinect_v2", 1, 
			    &JointsClient::jointsCb, this);

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
  }

  ~JointsClient()
  {

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

  void jointsCb(const humans_msgs::Humans::ConstPtr& hm)
  {
    //joints.clear();

    for(int i=0; i<hm->human.size(); ++i)
      {
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
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joints_viz");
  JointsClient jc;
  ros::spin();
  return 0;
}
