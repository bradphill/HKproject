/*
  * Lumbermill node that receives logs, cuts them to planks and sends them to a shop.
  * It's also okay, but it NEVER SLEEPS
*/

#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Int64.h"

// publisher and subscriber objects
ros::Publisher planks_pub;
ros::Subscriber lumber_sub;

std_msgs::Int64 planks_msg;

// function to cut logs
int log_cutter(int logs)
{
  // the lumbermill is extremely wasteful so we only get 10 planks per log
  int planks = 10*logs;
  ROS_DEBUG("cut %d logs into %d planks", logs, planks);
  return planks;
}

// callback for the subscriber
void lumber_callback(const std_msgs::Int64::ConstPtr &lumber_msg)
{
  // unpack the message
  int logs = lumber_msg->data;
  ROS_DEBUG("received %d logs",logs);
  // cut the logs by calling a separate function and package to planks_msg
  planks_msg.data = log_cutter(logs);

  // publish the planks
  planks_pub.publish(planks_msg);
  ROS_DEBUG("sent %d planks to the shop",(int)planks_msg.data);
}

// pubs and subs don't need to be init'd in the main loop, just check that the object scopes are correct
void node_init()
{
  ros::NodeHandle n; // can be risky to limit handle's scope to the init function only
  planks_pub = n.advertise<std_msgs::Int64>("planks_to_shop", 10);

  // has to subscribe to an existing topic (ours is created by lumberjack node)
  // calls back to lumber_callback when it receives a message
  lumber_sub = n.subscribe("lumber_to_mill", 10, lumber_callback);
}

// pretty clean main loop
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lumbermill_node");
  node_init();
  ros::Rate loop_rate(10); // "never sleeps" so 10 Hz :^)

  // below is pretty much equal to ros::spin()
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
