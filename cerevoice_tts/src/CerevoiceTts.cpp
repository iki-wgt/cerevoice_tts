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

#include "cerevoice_tts/CerevoiceTts.h"

namespace cerevoice_tts
{

CerevoiceTts::CerevoiceTts() : channel_handle_(0), player_(NULL),
    action_server_(
        node_handle_,
        "TTS",
        boost::bind(&CerevoiceTts::executeCB, this, _1),
        false)
{
  engine_ = CPRCEN_engine_new();
  ROS_ASSERT_MSG(engine_ != NULL, "CPRCEN_engine_new returned a NULL pointer");

  action_server_.registerPreemptCallback(boost::bind(&CerevoiceTts::preemptCB, this));
}

CerevoiceTts::~CerevoiceTts()
{
  if(player_ != NULL)
    CPRC_sc_player_delete(player_);

  CPRCEN_engine_delete(engine_);
}

void CerevoiceTts::channelCallback(CPRC_abuf * audio_buffer, void * user_data)
{
  CerevoiceTts *tts_object = static_cast<CerevoiceTts *>(user_data);

  // Transcription buffer, holds information on phone timings, markers etc.
  const CPRC_abuf_trans * transcription_buffer = NULL;
  CPRC_sc_audio * audio_sound_cue = NULL;
  int wav_mk, wav_done;

  wav_mk = CPRC_abuf_wav_mk(audio_buffer);
  wav_done = CPRC_abuf_wav_done(audio_buffer);
  if (wav_mk < 0) wav_mk = 0;
  if (wav_done < 0) wav_done = 0;

  if(tts_object->player_)
  {
    audio_sound_cue = CPRC_sc_audio_short_disposable(CPRC_abuf_wav_data(audio_buffer) + wav_mk, wav_done - wav_mk);
    CPRC_sc_audio_cue(tts_object->player_, audio_sound_cue);
  }
  else
  {
    ROS_ERROR("Can't play audio! Pointer to player is NULL!");
  }
}

bool CerevoiceTts::init()
{
  ros::NodeHandle private_node_handle("~");

  bool success = true;

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

      // load this voice
      success = CPRCEN_engine_load_voice(engine_, license.c_str(), NULL, path.c_str(), CPRC_VOICE_LOAD);
      if(!success)
      {
        ROS_ERROR("Unable to load voice file %s!", path.c_str());
        return false;
      }
      ROS_INFO("Loaded voice %s.", CPRCEN_engine_get_voice_info(engine_, i * -1 + xml_value.size() - 1, "VOICE_NAME"));
    }
  }
  catch(XmlRpc::XmlRpcException &ex)
  {
    ROS_ERROR("Caught an XmlRpcException! Message: %s, Code: %d", ex.getMessage().c_str(), ex.getCode());
    ROS_ERROR_COND(ex.getCode() == -1, "Have you forgot to specify a namespace in your launch file?");
    return false;
  }

  // Open a synthesis channel
  channel_handle_ = CPRCEN_engine_open_default_channel(engine_);
  if(channel_handle_ == 0)
  {
    ROS_ERROR("Unable to open synthesis channel!");
    return false;
  }

  std::string sample_rate_string = CPRCEN_channel_get_voice_info(engine_, channel_handle_, "SAMPLE_RATE");
  if(sample_rate_string.empty())
  {
    ROS_ERROR("Unable to get sample rate of the channel!");
    return false;
  }

  int sample_rate = atoi(sample_rate_string.c_str());

  // create the audio player
  ROS_INFO("Will now create the player. This may create messages from ALSA.");
  player_ = CPRC_sc_player_new(sample_rate);
  if(player_ == NULL)
  {
    ROS_ERROR("Unable to create player with sample rate %d Hz!", sample_rate);
    return false;
  }

  // set the callback
  success = CPRCEN_engine_set_callback(engine_, channel_handle_, this, channelCallback);
  if(!success)
  {
    ROS_ERROR("Unable to set callback function!");
    return false;
  }

  // start the action server
  action_server_.start();

  return true;
}

void CerevoiceTts::executeCB(const cerevoice_tts_msgs::TtsGoalConstPtr &goal)
{
  cerevoice_tts_msgs::TtsResult result;
  std::string xml = constructXml(goal->text, goal->voice);

  ROS_INFO("Will now use voice %s synthesize the text: %s", goal->voice.c_str(), goal->text.c_str());
  // synthesize the text
  if(!CPRCEN_engine_channel_speak(engine_, channel_handle_, xml.c_str(), xml.length(), 0))  // 0 = do not flush
  {
    ROS_ERROR("The speak method returned an error!");
    action_server_.setAborted();
    return;
  }


  // Finished processing, flush the buffer with empty input
  if(!CPRCEN_engine_channel_speak(engine_, channel_handle_, "", 0, 1))
  {
    ROS_ERROR("The speak method returned an error!");
    action_server_.setAborted();
    return;
  }

  // Wait till completion
  while(CPRC_sc_audio_busy(player_))
    CPRC_sc_sleep_msecs(AUDIO_SLEEP_TIME);

  if(action_server_.isActive())
    action_server_.setSucceeded(result);
}

void CerevoiceTts::preemptCB()
{
  ROS_INFO("Aborting text synthesis.");

  if(engine_)
  {
    if(!CPRCEN_engine_channel_reset(engine_, channel_handle_))  // Reset the channel, cancel processing of input
      ROS_ERROR("Could not reset channel!");
  }
  else
      ROS_ERROR("Engine is NULL!");

  if(player_)
  {
    int ret = CPRC_sc_audio_stop(player_);

    if(ret == 0)
      ROS_ERROR("Player can't be stopped!");

    action_server_.setPreempted();
  }
  else
    ROS_ERROR("Player is NULL!");
}

std::string CerevoiceTts::constructXml(std::string text, std::string voice)
{
  std::string xml = "<?xml version=\"1.0\"?>\n"
      "<speak version=\"1.1\" xmlns=\"http://www.w3.org/2001/10/synthesis\"\n"
      " xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
      " xsi:schemaLocation=\"http://www.w3.org/2001/10/synthesis\n"
      "           http://www.w3.org/TR/speech-synthesis11/synthesis.xsd\">\n";
  xml += "\n<voice name=\"" + voice + "\">\n";
  xml += text;
  xml += "\n</voice>";
  xml += "\n</speak>";
  return xml;
}

} /* namespace cerevoice_tts */
