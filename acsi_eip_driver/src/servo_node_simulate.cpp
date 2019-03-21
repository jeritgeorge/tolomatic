/**
 * @file servo_node.cpp
 * @brief Tolomatic ACSI servo interface using Ethernet/IP - ROS node to publish data & handle service calls.
 *
 * @author Bill McCormick <wmccormick@swri.org>
 * @date Feb 13, 2019
 * @version 0.1
 * @bug Implicit messaging not functional
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/JointState.h>

#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"

#include "servo_simulate.h"

using std::cout;
using std::endl;
using boost::shared_ptr;

using namespace acsi_eip_driver;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "servo");
  ros::NodeHandle nh("~");

  ros::Time::init();

  ros::Rate throttle(10);

  SimulatedACSI servo;

  float default_accel;
  nh.param<float>("default_accel", default_accel, 100.0);
  servo.so.accel = default_accel;

  float default_decel;
  nh.param<float>("default_decel", default_decel, 100.0);
  servo.so.decel = default_decel;

  float default_force;
  nh.param<float>("default_force", default_force, 30.0);
  servo.so.force = default_force;

  float default_velocity;
  nh.param<float>("default_velocity", default_velocity, 0);
  servo.so.velocity = default_velocity;



  // publisher for stepper status
  // ros::Publisher servo_pub = nh.advertise<acsi_inputs>("inputs", 1);
  ros::Publisher status_pub = nh.advertise<acsi_status>("status", 1);

  //services
  ros::ServiceServer moveVelocity_service = nh.advertiseService("moveVelocity", &SimulatedACSI::onMoveVelocity, &servo);
  ros::WallTime start(0), end(0);
  float elapsed_secs = 0;

  while (ros::ok())
  {
    try
    {
      end = ros::WallTime::now();
      if(!start.isZero()) {
        elapsed_secs = (end - start).toSec();
      }
      servo.updateDriveStatus(elapsed_secs);
      start = ros::WallTime::now();

      //publish stepper inputs
      // servo_pub.publish(servo.si);

      //publish stepper status
      status_pub.publish(servo.ss);

    }
    catch (std::runtime_error& ex)
    {
      ROS_ERROR_STREAM("Exception caught requesting scan data: " << ex.what());
    }
    catch (std::logic_error& ex)
    {
      ROS_ERROR_STREAM("Problem parsing return data: " << ex.what());
    }


    ros::spinOnce();

    throttle.sleep();

  }

  return 0;
}
