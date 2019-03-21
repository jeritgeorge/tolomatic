/**
 * @file servo.cpp
 * @brief Tolomatic ACSI servo interface using Ethernet/IP - servo functions class.
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

#include <cmath>        // std::abs
#include <ros/ros.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/asio.hpp>
#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/front/state_machine_def.hpp>

#include "servo_simulate.h"
#include <ctime>

using std::cout;
using std::endl;

using boost::shared_ptr;
using boost::make_shared;
using boost::asio::buffer;


namespace acsi_eip_driver {

//update the ROS drive status message
//requires regs are remapped:Remappable Reg1=Commanded Position and Remappable Reg2=Position Error


void SimulatedACSI::updateDriveStatus(float time_elapsed)
{
  if(time_elapsed == 0)
  {
    ss.current_position = 0;
  }
  else
  {
    ss.current_position = ss_last.current_position + so.velocity * time_elapsed;
    cout << "Time Elapsed: " << time_elapsed << endl;
    cout << "Last Position: " << ss_last.current_position << endl;
    cout << "Current Position: " << ss.current_position << endl;

  }

  memcpy(&ss_last,&ss, sizeof(ss));

}

bool SimulatedACSI::onMoveVelocity(acsi_eip_driver::acsi_moveVelocity::Request &req,
                        acsi_eip_driver::acsi_moveVelocity::Response &res)
{
  if(req.velocity > 15) {
    req.velocity = 15;
  } else if (req.velocity < -15) {
    req.velocity = -15;
  }

  so.velocity = req.velocity;
  res.success = true;

  return true;
}


} // namespace os32c
