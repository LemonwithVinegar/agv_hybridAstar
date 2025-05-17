#! /bin/zsh 

export SEED_HOME=`pwd`
source ${SEED_HOME}/devel/setup.zsh

#convert map
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=${SEED_HOME}/$1 -pbstream_filename=${SEED_HOME}/$1.pbstream -resolution=0.05


