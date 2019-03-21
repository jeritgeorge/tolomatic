/**
 * @file servo.h
 * @brief Tolomatic ACSI servo interface using Ethernet/IP - servo functions class definition.
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

#ifndef ACSI_EIP_DRIVER_H
#define ACSI_EIP_DRIVER_H

#include <boost/shared_ptr.hpp>

#include "odva_ethernetip/session.h"
#include "odva_ethernetip/socket/socket.h"

#include "input_assembly.h"
#include "output_assembly.h"

#include <acsi_eip_driver/acsi_inputs.h>
#include <acsi_eip_driver/acsi_outputs.h>
#include <acsi_eip_driver/acsi_status.h>

#include <acsi_eip_driver/acsi_moveVelocity.h>



using boost::shared_ptr;
using eip::Session;
using eip::socket::Socket;

namespace acsi_eip_driver {

//const double IN_POSITION_TOLERANCE = 0.10;

/**
 * Main interface for the Tolomatic stepper controller. 
 * Produces methods to access the stepper controller from a high level.
 */
class SimulatedACSI
{
public:
  /**
   * Construct a new instance.
   * @param socket Socket instance to use for communication with the stepper controller
   */
  SimulatedACSI()
  {
  }

  //drive interface functions
  // void servoControlCallback(const acsi_outputs::ConstPtr& oa);
  void updateDriveStatus(float time_elapsed);

  //ROS service callback handler functions
  bool onMoveVelocity(acsi_eip_driver::acsi_moveVelocity::Request  &req,
                      acsi_eip_driver::acsi_moveVelocity::Response &res);

  acsi_inputs si;
  acsi_outputs so;
  acsi_status ss;
  acsi_status ss_last;

};

} // namespace acsi_eip_driver

#endif  // ACSI_EIP_DRIVER_H
