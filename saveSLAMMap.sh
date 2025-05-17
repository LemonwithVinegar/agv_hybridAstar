#! /bin/zsh

export SEED_HOME=`pwd`
source ${SEED_HOME}/devel/setup.zsh

rosservice call /finish_trajectory 0 
rosservice call /write_state "{filename: '${SEED_HOME}/$1.pbstream',
include_unfinished_submaps: true}" 



