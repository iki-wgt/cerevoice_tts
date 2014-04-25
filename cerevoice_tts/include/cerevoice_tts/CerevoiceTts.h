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

#ifndef CEREVOICETTS_H_
#define CEREVOICETTS_H_

#include <cerevoice_eng.h>

namespace cerevoice_tts
{

class CerevoiceTts
{
private:
  /**
   * Pointer to the CereVoice text-tp-speech engine.
   */
  CPRCEN_engine *engine_;

  /**
   * Handle to the synthesis channel.
   */
  CPRCEN_channel_handle channel_handle_;

  /**
   * @brief Callback function that is fired for every phrase returned by the synthesizer.
   *
   * The callback method is usefull for low latency speech output.
   *
   * @param audio_buffer Audio buffer with the synthesized phrase.
   * @param user_data User defined data.
   */
  static void channelCallback(CPRC_abuf * audio_buffer, void * user_data);

public:
  CerevoiceTts();
  ~CerevoiceTts();

  /**
   * @brief Initializes the TTS object.
   *
   * Fetches the voice and license files form the parameter server.
   * Loads these voices. Creates a audio channel.
   *
   * @return True if successful, false otherwise.
   */
  bool init();
};

} /* namespace cerevoice_tts */

#endif /* CEREVOICETTS_H_ */