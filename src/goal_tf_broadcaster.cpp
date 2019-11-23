#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

float global_x = 0.0;
float global_y = 0.0;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		global_x = msg->pose.position.x;
		global_y = msg->pose.position.y;
	}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goal_tf_publisher");

	ros::NodeHandle n;

	ros::Rate rate(10.0);
	ros::Time start = ros::Time::now();

	ros::Subscriber sub = n.subscribe("/move_base_simple/goal", 1000, goalCallback);
	// goal_sub = n.subscribe<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 100, goalCallback);

	static tf::TransformBroadcaster br;
  tf::Transform transform;

	while(ros::ok())
	{
		ros::spinOnce();
		transform.setOrigin( tf::Vector3(global_x, global_y, 0.0) );
	  transform.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0));
	  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "target"));


		rate.sleep();
	}
}
