/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"

#define PICKUP_ZONE_X 4.0
#define PICKUP_ZONE_Y 7.0
#define DROP_OFF_ZONE_X 3.0
#define DROP_OFF_ZONE_Y 0.0
#define TOL 0.25

#define NS "virtual_obj"

ros::Publisher marker_pub;
volatile bool pickup_reached = false;
volatile bool drop_off_reached = false;

int put_marker(ros::Publisher marker_pub, int id, uint32_t shape, double x, double y)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = NS;
    marker.id = id;

    // Set the marker type.
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return -1;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    return 0;
}

int hide_marker(ros::Publisher marker_pub, int id)
{
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = NS;
    marker.id = id;

    // Set the marker type.
    //marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::DELETE;
    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return -1;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    return 0;
}

void goalCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //ROS_INFO("Seq: [%d]", msg->header.seq);
  //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

  if (!pickup_reached) {
    if (PICKUP_ZONE_X - TOL < msg->pose.pose.position.x &&
        msg->pose.pose.position.x < PICKUP_ZONE_X + TOL &&
        PICKUP_ZONE_Y - TOL < msg->pose.pose.position.y &&
        msg->pose.pose.position.y < PICKUP_ZONE_Y + TOL)
    {
      ROS_INFO("PICKUP!! Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
      pickup_reached = true;
      hide_marker(marker_pub, 0);
    }
  } else if (!drop_off_reached) {
    if (DROP_OFF_ZONE_X - TOL < msg->pose.pose.position.x &&
        msg->pose.pose.position.x < DROP_OFF_ZONE_X + TOL &&
        DROP_OFF_ZONE_Y - TOL < msg->pose.pose.position.y &&
        msg->pose.pose.position.y < DROP_OFF_ZONE_Y + TOL)
    {
      ROS_INFO("DROP_OFF!! Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
      drop_off_reached = true;
      put_marker(marker_pub, 0, visualization_msgs::Marker::CUBE, DROP_OFF_ZONE_X, DROP_OFF_ZONE_Y);
    }
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::NodeHandle n2;
  ros::Rate r(1);
  //ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  //ROS_INFO("Subscribing to odom!");
  ros::Subscriber sub = n2.subscribe("odom", 1, goalCallback);
  if (sub)

  put_marker(marker_pub, 0, visualization_msgs::Marker::CUBE, PICKUP_ZONE_X, PICKUP_ZONE_Y);
  //ros::Duration(5.0).sleep();

  //ROS_INFO("Before ros::spin loop 1");
  //while (ros::ok() && !pickup_reached)
    //ros::spin();

  //ROS_INFO("Out of ros::spin loop 1, hiding...");
  //hide_marker(marker_pub, 0);

  //ros::Duration(5.0).sleep();

  //ROS_INFO("Before ros::spin loop 2");
  //while (ros::ok() && !drop_off_reached)
    //ros::spin();

  while (ros::ok())
    ros::spin();

  //ROS_INFO("Out of ros::spin loop 2, showing...");
  //put_marker(marker_pub, 0, visualization_msgs::Marker::CUBE, DROP_OFF_ZONE_X, DROP_OFF_ZONE_Y);

  while (ros::ok())
    r.sleep();
}
