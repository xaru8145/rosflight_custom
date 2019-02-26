
/**
 * @file x_vel_step_test.cpp
 * @brief Adaptation of the offb_node.cpp script. Commands the
 *        vehicle to a takeoff position, then a maneuver start position,
 *        then it commands a step velocity input, halts, and returns to land.
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

// Vicon Pose Callback
geometry_msgs::PoseStamped vicon_pose;
void vicon_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    vicon_pose = *msg;
}

// Vicon Velocity Callback
geometry_msgs::TwistStamped vicon_vel;
void vicon_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    vicon_vel = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber vicon_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/vrpn_client_node/RadarQuad/pose", 10, vicon_pose_cb);
    ros::Subscriber vicon_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/vrpn_client_node/RadarQuad/twist", 10, vicon_vel_cb);

    // Publishers
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>
            ("odom", 10);
    
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // Start timer and choose initial setpoint type
    ros::Time last_request = ros::Time::now();

    nav_msgs::Odometry odom_msgs;
    odom_msgs.header.frame_id = "world";
    // MAIN CONTROL LOOP
    while(ros::ok()){

        // Create full message with all topics
        odom_msgs.header.stamp = vicon_pose.header.stamp;
        odom_msgs.pose.pose.position.x = vicon_pose.pose.position.x;
        odom_msgs.pose.pose.position.y = -vicon_pose.pose.position.y;
        odom_msgs.pose.pose.position.z = -vicon_pose.pose.position.z;
	odom_msgs.pose.pose.orientation.x = vicon_pose.pose.orientation.x;
	odom_msgs.pose.pose.orientation.y = vicon_pose.pose.orientation.y;
	odom_msgs.pose.pose.orientation.z = vicon_pose.pose.orientation.z;
	odom_msgs.pose.pose.orientation.w = vicon_pose.pose.orientation.w;

        // Convert Quaternion to RPY
        double roll;
	double pitch;
        double yaw;
        tf::Quaternion tf_quat;
        tf::quaternionMsgToTF(vicon_pose.pose.orientation, tf_quat);
        tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
	yaw = -yaw;

        odom_msgs.twist.twist.linear.x = cos(yaw) * vicon_vel.twist.linear.x - sin(yaw) * vicon_vel.twist.linear.y;
	odom_msgs.twist.twist.linear.y = -1*(sin(yaw) * vicon_vel.twist.linear.x + cos(yaw) * vicon_vel.twist.linear.y);
	odom_msgs.twist.twist.linear.z = -vicon_vel.twist.linear.z;
	
	odom_msgs.twist.twist.angular.x = vicon_vel.twist.angular.x;
	odom_msgs.twist.twist.angular.y = -vicon_vel.twist.angular.y;
	odom_msgs.twist.twist.angular.z = -vicon_vel.twist.angular.z;

        odom_pub.publish(odom_msgs);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
