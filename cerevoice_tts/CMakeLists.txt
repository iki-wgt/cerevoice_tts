cmake_minimum_required(VERSION 2.8.3)
project(cerevoice_tts)

if(EXISTS "$ENV{HOME}/cerevoice")
  MESSAGE(STATUS "Setting location of CereVoice to $ENV{HOME}/cerevoice")
  set(cerevoice_DIR $ENV{HOME}/cerevoice)
elseif(EXISTS "$ENV{HOME}/CereVoice")
  MESSAGE(STATUS "Setting location of CereVoice to $ENV{HOME}/CereVoice")
  set(cerevoice_DIR $ENV{HOME}/CereVoice)
elseif(EXISTS "$ENV{HOME}/Cerevoice")
  MESSAGE(STATUS "Setting location of CereVoice to $ENV{HOME}/Cerevoice")
  set(cerevoice_DIR $ENV{HOME}/Cerevoice)
elseif(EXISTS "$ENV{HOME}/cereproc/cerevoice")
  MESSAGE(STATUS "Setting location of CereVoice to $ENV{HOME}/cereproc/cerevoice")
  set(cerevoice_DIR $ENV{HOME}/cereproc/cerevoice)
elseif(EXISTS "$ENV{HOME}/cerevoice_sdk")
  MESSAGE(STATUS "Setting location of CereVoice to $ENV{HOME}/cerevoice_sdk")
  set(cerevoice_DIR $ENV{HOME}/cerevoice_sdk)
elseif(EXISTS "/opt/cereproc/cerevoice")
  MESSAGE(STATUS "Setting location of CereVoice to /opt/cereproc/cerevoice")
  set(cerevoice_DIR /opt/cereproc/cerevoice)
elseif(EXISTS "/opt/cerevoice")
  MESSAGE(STATUS "Setting location of CereVoice to /opt/cerevoice")
  set(cerevoice_DIR /opt/cerevoice)
  # here you can add more locations
else()
  MESSAGE(SEND_ERROR "Can't find location of CereVoice! Modify the CMakeLists.txt to find your installation directory")
endif()

set(cerevoice_LIBS
  ${cerevoice_DIR}/cerevoice_eng/lib/libcerevoice_eng_shared.so
  ${cerevoice_DIR}/cerevoice_pmod/lib/libcerevoice_pmod_shared.so
  ${cerevoice_DIR}/cerehts/lib/libcerehts_shared.so
  ${cerevoice_DIR}/cerevoice/lib/libcerevoice_shared.so
  
  ${cerevoice_DIR}/cerevoice_aud/lib/libcerevoice_aud_shared.so
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  cerevoice_tts_msgs
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES cerevoice_tts
 CATKIN_DEPENDS actionlib cerevoice_tts_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${cerevoice_DIR}/cerevoice_eng/include
  ${cerevoice_DIR}/cerevoice_aud/include
)

## Declare a cpp library
#add_library(cerevoice_tts
#  src/${PROJECT_NAME}/cerevoice_tts.cpp
# )

## Declare a cpp executable
add_executable(cerevoice_tts_node src/cerevoice_tts_node.cpp src/CerevoiceTts.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(cerevoice_tts_node cerevoice_tts_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(cerevoice_tts_node
  ${catkin_LIBRARIES}
  
  ${cerevoice_LIBS}
  asound
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  ## Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test test/test_cerevoice_tts.cpp src/CerevoiceTts.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test 
    ${catkin_LIBRARIES}
    asound
    ${cerevoice_LIBS})
  endif()
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
