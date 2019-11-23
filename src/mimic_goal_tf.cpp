#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

class Mimic
{
  public:
    Mimic();

  private:
    void goalCallback(const geometry_msgs::PoseStampedConstPtr& poses);
    // ros::Publisher markerarray_pub_;
    ros::Subscriber goal_sub_;
};

Mimic::Mimic()
{
  ros::NodeHandle input_nh("input");
  // ros::NodeHandle output_nh("output");
  // markerarray_pub_ = output_nh.advertise<visualization_msgs::MarkerArray> ("/target_human/prediction/marker", 10);
  goal_sub_ = input_nh.subscribe<geometry_msgs::PoseStamped> ("/move_base_simple/goal", 10, &Mimic::goalCallback, this);
}

void Mimic::goalCallback(const geometry_msgs::PoseStampedConstPtr& poses)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(poses->pose.position.x, poses->pose.position.y, 0.0) );
  // transform.setOrigin(poses->pose.position);
  transform.setRotation( tf::Quaternion(poses->pose.orientation.x, poses->pose.orientation.y, poses->pose.orientation.z, poses->pose.orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "target", "map"));

}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "mimic_goal_tf");

  Mimic mimic;

  ros::spin();

}
