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
