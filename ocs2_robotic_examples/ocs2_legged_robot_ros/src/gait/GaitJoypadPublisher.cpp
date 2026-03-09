/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_legged_robot_ros/gait/GaitJoypadPublisher.h"

#include <algorithm>

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_msgs/mode_schedule.h>

#include "ocs2_legged_robot_ros/gait/ModeSequenceTemplateRos.h"
#include <sensor_msgs/Joy.h>

std::string gaitCommandPrev = "";
namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitJoypadPublisher::GaitJoypadPublisher(ros::NodeHandle nodeHandle, const std::string& gaitFile, const std::string& robotName,
                                             bool verbose) {
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
  loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);

  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void GaitJoypadPublisher::getJoyMsgCommand(const sensor_msgs::JoyConstPtr& joy)
{
  std::string gaitCommand = "";
  if (joy -> buttons[3] == 1 || joy -> buttons[2] == 1){
    if(joy -> buttons[3] == 1){
      gaitCommand = "trot";
    }
    else if(joy -> buttons[2] == 1){
      gaitCommand = "stance";
    }
    if (gaitCommandPrev != gaitCommand){
      try {
        ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
        modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSequenceTemplate));
        gaitCommandPrev = gaitCommand;
      } catch (const std::out_of_range& e) {
        std::cout << "Gait \"" << gaitCommand << "\" not found.\n";        
      }
    }
  }

  
}

}  // namespace legged_robot
}  // end of namespace ocs2
