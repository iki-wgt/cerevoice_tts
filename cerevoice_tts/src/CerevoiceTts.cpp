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
#include "XmlRpcException.h"
#include "XmlRpcValue.h"

#include <cerevoice_aud.h>

#include "cerevoice_tts/CerevoiceTts.h"

namespace cerevoice_tts
{

CerevoiceTts::CerevoiceTts() : channel_handle_(0)
{
  engine_ = CPRCEN_engine_new();
  ROS_ASSERT_MSG(engine_ != NULL, "CPRCEN_engine_new returned a NULL pointer");
}

CerevoiceTts::~CerevoiceTts()
{
  CPRCEN_engine_delete(engine_);
}

void CerevoiceTts::channelCallback(CPRC_abuf * audio_buffer, void * user_data)
{

}

bool CerevoiceTts::init()
{
  ros::NodeHandle private_node_handle("~");

  // parse rosparams
  XmlRpc::XmlRpcValue xml_value;

  private_node_handle.param("voices", xml_value, xml_value);

  try
  {
    ROS_INFO("Found %d voices!", xml_value.size());

    // The first voice should be the default voice. The default voice has to be added last.
    // so iterate reversed
    for (int i = xml_value.size() - 1; i >= 0; --i)
    {
      std::string path = xml_value[i]["path"];
      std::string license = xml_value[i]["license"];

      ROS_DEBUG("voice no. %d: %s", i + 1, path.c_str());
      ROS_DEBUG("license no. %d: %s", i + 1, license.c_str());

      if(path.empty())
      {
        ROS_ERROR("Empty voice path in list element %d!", i + 1);
        return false;
      }

      if(license.empty())
      {
        ROS_ERROR("Empty license path in list element %d!", i + 1);
        return false;
      }
    }
  }
  catch(XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("Caught an XmlRpcException! Message: %s, Code: %d", ex.getMessage().c_str(), ex.getCode());
    ROS_ERROR_COND(ex.getCode() == -1, "Have you forgot to specify a namespace in your launch file?");
    return false;
  }

  return true;
}

} /* namespace cerevoice_tts */
