#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
// distance threshold to identify if the robot reach the target or not
const float dist_thres = 0.6;

class ObjectPose
{
public:
    ObjectPose();
    float pickup_local[2];
    float dropoff_local[2];
    bool pickup = false;
    bool dropoff = false;
    ros::Publisher marker_pub;
    ros::Subscriber pose_sub;
    ros::NodeHandle n;
    visualization_msgs::Marker marker;
    void pose_callback(const nav_msgs::Odometry::ConstPtr &msg);
};

ObjectPose::ObjectPose(){
    // Publish marker
    marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    // Subscribe amcl_pose node to get the robot position
    pose_sub = n.subscribe("/odom", 1000, &ObjectPose::pose_callback,this);
}

void ObjectPose::pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    //get the robot position information
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    //calculate the distance of robot's current position to the target
    float distance2target = sqrt(pow(x-this->pickup_local[0],2) + pow(y-this->pickup_local[1],2));
    //display position information on terminal
    ROS_INFO("X position %f Y position %f", x, y);

    if ((!this->pickup) && (!this->dropoff) && (distance2target > dist_thres))
    {
        ROS_INFO("Approcahing Pick up location");
    }
    
    if ((!this->pickup) && (distance2target < dist_thres))
    {
        ROS_INFO("Reached Picked up location");
        //Delte the marker after the robot reach the pickup location
        this->marker.action = visualization_msgs::Marker::DELETE;
        sleep(5);
        this->marker_pub.publish(this->marker);
        this->pickup = true;
    }

    if ((this->pickup) && (!this->dropoff) && (distance2target > dist_thres))
    {
        ROS_INFO("Approcahing Drop off location");
    }

    if ((!this->dropoff) && (distance2target < dist_thres))
    {
        ROS_INFO("Reached Drop off location");
        //show up the marker after the robot reach the drop off loaction
        this->marker.action = visualization_msgs::Marker::ADD;
        this->marker.pose.position.x = this->dropoff_local[0];
        this->marker.pose.position.y = this->dropoff_local[1];
        this->marker.color.r = 0.0f;
        this->marker.color.g = 1.0f;
        this->marker.color.b = 0.0f;
        this->marker.color.a = 1.0;
        this->marker_pub.publish(this->marker);
        this->dropoff = true;        

    }


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_markers");
    ObjectPose robot_pose;
    //identify the pick up and drop off location in the gazebo
    robot_pose.pickup_local[0] = 0.8;
    robot_pose.pickup_local[1] = 4.6;
    robot_pose.dropoff_local[0] = -4;
    robot_pose.dropoff_local[1] = -0.2;

    // Initialize the ROS node
    

    // Set shape type to be a cylinder
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
    while (ros::ok())
    {
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        robot_pose.marker.header.frame_id = "/map";
        robot_pose.marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        robot_pose.marker.ns = "add_markers";
        robot_pose.marker.id = 0;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        robot_pose.marker.type = shape;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        robot_pose.marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        robot_pose.marker.pose.position.x = robot_pose.pickup_local[0];
        robot_pose.marker.pose.position.y = robot_pose.pickup_local[1];
        robot_pose.marker.pose.position.z = 0.0;
        robot_pose.marker.pose.orientation.x = 0.0;
        robot_pose.marker.pose.orientation.y = 0.0;
        robot_pose.marker.pose.orientation.z = 0.0;
        robot_pose.marker.pose.orientation.w = 1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        robot_pose.marker.scale.x = 0.25;
        robot_pose.marker.scale.y = 0.25;
        robot_pose.marker.scale.z = 0.25;

        // Set the color -- be sure to set alpha to something non-zero!
        robot_pose.marker.color.r = 0.0f;
        robot_pose.marker.color.g = 0.0f;
        robot_pose.marker.color.b = 1.0f;
        robot_pose.marker.color.a = 1.0;

        robot_pose.marker.lifetime = ros::Duration();

        while (robot_pose.marker_pub.getNumSubscribers() < 1)
        {
            if (!ros::ok())
            {
                return 0;
            }
          ROS_WARN_ONCE("Please create a subscriber to the marker");
          sleep(1);
        }
        robot_pose.marker_pub.publish(robot_pose.marker);
        ros::spin();
    }
}
