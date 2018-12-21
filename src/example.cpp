/*
 * Copyright (C) 2018, Sammy Pfeiffer
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>

/**
 * This tutorial demonstrates how to use the control_toolbox::pid class
 * taking advantage of its dynamic reconfigure server and its
 * debugging topic.
 */


class ThingController
{
public:
  ThingController(ros::NodeHandle nh_)
  {
    // The dynamic reconfigure server is only instanced
    // if the param server contains at least the 'p' gain
    // in this case, /my_namespace/my_pid/p
    // Note that with this setup, the parameters that
    // stay in the param server from previous runs
    // will be the initial values of the PID
    // This is useful to just add them to a launchfile/yaml
    // You could choose to initialize your values
    // in some other way, but if you want to have access to the
    // dynamic reconfigure you must set the 'p' parameter
    if (!nh_.hasParam("my_pid/p"))
        nh_.setParam("my_pid/p", 100.0);
    if (!nh_.hasParam("my_pid/i"))
        nh_.setParam("my_pid/i", 10.0);
    if (!nh_.hasParam("my_pid/d"))
        nh_.setParam("my_pid/d", 0.0);
    if (!nh_.hasParam("my_pid/i_clamp_min"))
        nh_.setParam("my_pid/i_clamp_min", -10.0);
    if (!nh_.hasParam("my_pid/i_clamp_max"))
        nh_.setParam("my_pid/i_clamp_max", 10.0);
    // If the param server contains 'publish_state' to true
    // a topic publishing the state of the PID will be available
    // in this case /my_namespace/my_pid/state of type control_msgs/PidState.msg
    // Here I'm forcing it to true to show it
    nh_.setParam("my_pid/publish_state", true);
    // With this init call the p, i, d & i_clamp variables are taken from the param server
    my_pid_.init(ros::NodeHandle(nh_, "my_pid"), false);
    // Check the docs for other init possibilities:
    // http://docs.ros.org/api/control_toolbox/html/classcontrol__toolbox_1_1Pid.html
    // it can be initialized without dynamic reconfigure, or with initial values
    // and a dynamic reconfigure

    // Demo stuff
    vel_sub_ = nh_.subscribe("cmd_vel", 10, &ThingController::command_cb, this);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 10);
    x_to_control_ = 0.0;
    ROS_INFO_STREAM("Node ready");
  }

  void command_cb(const geometry_msgs::Twist& msg){
    ROS_INFO_STREAM("Got a command: " << msg.linear.x);
    last_cmd_ = msg;
  }

  void do_stuff(){
    ROS_INFO_STREAM("Doing stuff");
    while (ros::ok()){
        ros::Time tnow = ros::Time::now();
        // If any problem arises, make sure dt is > 0.0
        // as if its 0.0 the computedCommand will be 0.0
        ros::Duration dt = tnow - last_cmd_time_;

        double error = last_cmd_.linear.x - this->get_system_state();
        double command = my_pid_.computeCommand(error, dt);

        this->command_system(command);

        ros::spinOnce();
        last_cmd_time_ = tnow;
        ros::Duration(0.1).sleep();
    }
  }

double get_system_state(){
    return x_to_control_;
}

// fake we are commanding a system
void command_system(double command){
    x_to_control_ = x_to_control_ + command/100.0;
    if (x_to_control_ > last_cmd_.linear.x + 0.05)
        x_to_control_ = last_cmd_.linear.x + 0.05;
    else if(x_to_control_ < last_cmd_.linear.x - 0.05)
        x_to_control_ = last_cmd_.linear.x - 0.05;
    geometry_msgs::Twist t;
    t.linear.x = x_to_control_;
    vel_pub_.publish(t);
    ROS_INFO_STREAM("Publishing: " << x_to_control_);
}

private:
  control_toolbox::Pid my_pid_;
  ros::Subscriber vel_sub_;
  ros::Publisher vel_pub_;
  geometry_msgs::Twist last_cmd_;
  ros::Time last_cmd_time_;

  double x_to_control_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_example");
  ros::NodeHandle nh_("my_namespace");

  ThingController thing_controller = ThingController(nh_);
  thing_controller.do_stuff();

  return 0;
}