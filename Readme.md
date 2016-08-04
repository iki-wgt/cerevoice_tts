# CereVoice Text-To-Speech repository

This repository contains ROS packages for the CereProc CereVoice TTS engine. It is useless without the CereVoice SDK and a valid license. To get CereVoice please visit www.cereproc.com.

## Installation

To use this package you have to create a package for the CereVoice SDK you got from CereProc. Put all folders and files from the SDK in this package. The ```CMakeLists.txt``` has to look like this:

```CMake
cmake_minimum_required(VERSION 2.8.3)
project(cerevoice_sdk)

find_package(catkin REQUIRED)
find_package(ALSA REQUIRED)

find_library(ENG_LIB
  NAMES cerevoice_eng_shared
  PATHS cerevoice_eng/lib)

find_library(PMOD_LIB
  NAMES cerevoice_pmod_shared
  PATHS cerevoice_pmod/lib)

find_library(CEREHTS_LIB
  NAMES cerehts_shared
  PATHS cerehts/lib)

find_library(CEREVOICE_LIB
  NAMES cerevoice_shared
  PATHS cerevoice/lib)

find_library(AUD_LIB
  NAMES cerevoice_aud_shared
  PATHS cerevoice_aud/lib)

catkin_package(
  INCLUDE_DIRS cerevoice_eng/include cerevoice_aud/include
  LIBRARIES
  ${ENG_LIB}
  ${PMOD_LIB}
  ${CEREHTS_LIB}
  ${CEREVOICE_LIB}
  ${AUD_LIB}

  DEPENDS ALSA
)
```

In the ```package.xml``` depend on ```libasound2-dev```.

## Running

```$ roslaunch cerevoice_tts tts.launch ```

You may have to modify the language file information in the launch file.

Always launch the node on the computer where the speakers are connected!

## Synthesizing text

The text in the goal will be put in XML format, so feel free to use XML or SSML tags in your text.
You do NOT have to provide the `<xml>` and `<speak>` tags. This is done automatically.

If you leave the voice parameter unset, the default voice will be used.

The default voice is the first voice specified in the launch file.

### From the command line

```$ rosrun actionlib axclient.py /TTS ```

There enter the name of the voice and the text.

### From within a ROS node

See the [ROS actionlib tutorial](http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient) for how to use actionlib.

Create the action client with
```actionlib::SimpleActionClient<cerevoice_tts_msgs::TtsAction> action_client("TTS", true);```

Set the voice and the text in the action goal with

    cerevoice_tts_msgs::TtsGoal goal;
    goal.text = "Your text here.";
    goal.voice = "Heather";

## Launch file format

Example:

    <?xml version="1.0"?>
    <launch>
      <param name="cerevoice_tts_node/startup_sentence" type="string" value="Text zu Sprache bereit." />
      <rosparam param="voices" ns="cerevoice_tts_node" subst_value="true">
      - path: $(env HOME)/cerevoice_sdk/voices/cerevoice_alex_3.0.6_22k.voice
        license: $(env HOME)/cerevoice_sdk/voices/license.lic
        abbrev: $(env HOME)/cerevoice_sdk/example_data/abbrev_german.txt
      - path: $(env HOME)/cerevoice_sdk/voices/cerevoice_gudrun_3.0.6_22k.voice
        license: $(env HOME)/cerevoice_sdk/voices/license.lic
        abbrev: $(env HOME)/cerevoice_sdk/example_data/abbrev_german.txt
      - path: $(env HOME)/cerevoice_sdk/voices/cerevoice_heather_3.0.8_22k.voice
        license: $(env HOME)/cerevoice_sdk/voices/cereproc_license.lic
        lexicon: $(env HOME)/cerevoice_sdk/example_data/additional.lex
      </rosparam>
      <node name="cerevoice_tts_node" pkg="cerevoice_tts" type="cerevoice_tts_node" respawn="false" output="screen" />
    </launch>

The path and the license of a voice are required. Lexicon and abbreviations file are optional.
The optional string startup_sentence will be synthesized when the TTS node is ready.

## Scripts

### txt2speech

To test text you can use the script txt2speech. It takes as an argument the path to a text file. This text will be synthesised.

    usage: txt2speech [-h] [--voice Alex] path/file

Examples:

    $ rosrun cerevoice_tts txt2speech text.txt
    $ rosrun cerevoice_tts txt2speech.py --voice=Alex text.txt

## Disclaimer

This software has been developed by the Ravensburg-Weingarten Institute of Artificial Intelligence and not by CereProc. It has been published with kind permission of CereProc.
