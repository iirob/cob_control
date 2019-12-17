# DO NOT USE UNLESS YOU ARE SURE.
# When you control the platfrom with a joystick, there are limitations on speed.
# With this, nothing.

# Use this to move the robot.
# f 0.1 0.2 0.1
# should move the platform forwards with 10cm/s, to the left with 20cm/s, and CCW rotation with 0.1 radian/s.
# Keep the numbers under 0.5.
f () {
   rostopic pub -r 10 /base/twist_controller/command geometry_msgs/Twist  "{linear:  {x: $1, y: $2, z: 0.0}, angular: {x: 0.0,y: 0.0,z: $3}}" 
} 

# You can use these to move the robot with an isolated component.
x () { 
   f $1 0 0 
} 
y () { 
   f 0 $1 0 
} 
z () { 
   f 0 0 $1 
} 

# Switch to Ackermann mode
ack () { 
   timeout 1 rostopic pub -r 10 /base/twist_controller/command geometry_msgs/Twist  "{linear:  {x: 0.0, y: 0.0, z: 1.1}, angular: {x: 0.0,y: 0.0,z: 0.0}}" 
}                                                                                                                                                   

# Switch to differential mode
dif () {                                                                                                                                            
   timeout 1 rostopic pub -r 10 /base/twist_controller/command geometry_msgs/Twist  "{linear:  {x: 0.0, y: 0.0, z: 1.2}, angular: {x: 0.0,y: 0.0,z: 0.0}}" 
}                                                                                                                                                   

# Set a center of rotation (valid for differential and ackermann)
# Unit is meters.
cen () {                                                                                                                                            
   timeout 1 rostopic pub -r 10 /base/twist_controller/command geometry_msgs/Twist  "{linear:  {x: 0.0, y: 0.0, z: 1.3}, angular: {x: $1,y: $2,z: 0.0}}" 
}

# Switch to omnidirectional mode
omn () {                                                                                                                                            
   timeout 1 rostopic pub -r 10 /base/twist_controller/command geometry_msgs/Twist  "{linear:  {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}" 
}                                                                                                                                                   

