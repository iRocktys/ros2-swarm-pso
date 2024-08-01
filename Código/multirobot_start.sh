#!/bin/bash
ros2 run turtlesim turtlesim_node&

#ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

i=1
x=0
y=0

l=0
c=0
while [ $i -le $1 ]; do
  	echo "Creating Node client $i..."

  	#limit of horizontal
  	if [ $y -lt 5 ]; then #line
      if [ $x -lt 500 ]; then #column
      	y=`expr $y + 1` #y:line
      else
      	x=0
      	y=0
      fi
  	else
      #limit of horizontal
      x=`expr $x + 1` #x:column
      y=0
  	fi
 	 
   #   ros2 topic pub --once /turtle$i/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.5}}"
  	ros2 service call /turtle$i/set_pen turtlesim/srv/SetPen "{'r': 0, 'g': 0, 'b': 0, 'width': 0, 'off': 1}"
  	a=`expr $i + 1`  #different var due bash while restriction
  	if [ $a -le $1 ]; then
      echo 'Spawn $a...'
      ros2 service call spawn turtlesim/srv/Spawn "{'x':$x, 'y':$y, 'theta':0.0}"
  	fi
  	i=`expr $i + 1`
  	#sleep 1
done
