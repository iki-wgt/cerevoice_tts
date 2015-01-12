/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Hochschule Ravensburg-Weingarten
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*     Author: Benjamin Reiner (reineben@hs-weingarten.de)
*********************************************************************/

#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>

#include "cerevoice_tts/CerevoiceTts.h"
#include "cerevoice_tts_msgs/TtsAction.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tts_node");

  cerevoice_tts::CerevoiceTts tts;
  bool initialized = tts.init();

  if(!initialized)
    return EXIT_FAILURE;
  else
	ROS_INFO("Cerevoice initialized successfully.");

  ros::spinOnce();

  ros::NodeHandle private_node_handle("~");
  std::string startup_sentence;
  private_node_handle.param("startup_sentence", startup_sentence, std::string(""));

  if(!startup_sentence.empty())
  {
    ROS_INFO("Will now say the startup sentence '%s'", startup_sentence.c_str());
    ROS_INFO("Will wait for server");

    // Output this startup sentence
    actionlib::SimpleActionClient<cerevoice_tts_msgs::TtsAction> action_client("TTS", true);
    while(!action_client.isServerConnected())
    {
      ros::spinOnce();  // this looped spinning is required because we call an action from within the same node
    }

    ROS_INFO("Server ready");

    cerevoice_tts_msgs::TtsGoal goal;
    goal.text = startup_sentence;

    action_client.sendGoal(goal);
    while(!action_client.waitForResult(ros::Duration(0.0001)))
    {
      ros::spinOnce();
    }

    if(action_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_ERROR("Synthesizing the startup sentence failed!");
  }
  else
	ROS_DEBUG("No startup sentence specified.");

  ros::spin();

  return EXIT_SUCCESS;
}

