export EE4308WS=`echo "$( cd "$( dirname "$0" )" && pwd )"`
cd `echo $EE4308WS`

source ~/hector/devel/setup.bash # underlay the hector ws for catkin_make # imu reqiured for turtle
catkin_make # overlays this workspace over hector ws so pkgs in this can find pkgs in hector
source devel/setup.bash
