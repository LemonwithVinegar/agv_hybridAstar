#! /bin/zsh
#sudo ptpd -M -i eno1 -C

export SEED_HOME=`pwd`
source ${SEED_HOME}/devel/setup.zsh

echo "nvidia" | 
sudo -S chmod 777 /dev/ttyUSB*
source ${SEED_HOME}/devel/setup.sh 
# roslaunch ${SEED_HOME}/src/pure_slamLocation.launch 
roslaunch ${SEED_HOME}/src/example.launch 

