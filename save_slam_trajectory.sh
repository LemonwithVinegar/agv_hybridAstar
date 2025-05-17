#! /bin/zsh

export SEED_HOME=`pwd`
source ${SEED_HOME}/devel/setup.zsh
rosservice call /trajectory_query 0  
