# CereVoice Text-To-Speech repository

This repository contains ROS packages for the CereProc CereVoice TTS engine.

## Installation
0.0 [Install ROS Hydro](http://wiki.ros.org/hydro/Installation/Ubuntu) (or maybe Indigo or J-Turtle in the future) if you haven't already.
0.1 [Create a Catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) if you don't already have one.

1.  Unpack CereVoice into a directory (e.g. ~/cerevoice, ~/CereVoice, ~/Cerevoice, ~/cereproc/cerevoice, ~/cerevoice_sdk, /opt/cereproc/cerevoice or /opt/cerevoice (to add more possible install directories edit the [CMakeLists.txt](cerevoice_tts/CMakeLists.txt)))
2.  Copy your language and license files to a directory. (e.g. create a voices directory in your CereVoice directory)
3.  Clone this repository:
```
$ cd ~/catkin_ws/src
$ git clone git@141.69.58.11:ros/cerevoice_tts.git
```

## Running
```$ roslaunch cerevoice_tts tts.launch ```

You may have to modify the langua file information in the launch file.

## Synthesizing text
The text in the goal will be put in XML format, so feel free to use XML or SSML tags in your text.
You do NOT have to provide the <xml> and <speak> tags. This is done automatically.

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
```
cerevoice_tts_msgs::TtsGoal goal;
goal.text = "Your text here.";
goal.voice = "Heather";
```

## Launch file format
Example:
```
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
```

The path and the license of a voice are required. Lexicon and abbreviations file are optional.
The optional string startup_sentence will be synthesized when the TTS node is ready.
