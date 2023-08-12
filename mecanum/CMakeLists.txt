cmake_minimum_required(VERSION 2.8.3)

project(mecanum)

find_package(catkin REQUIRED COMPONENTS
  gazebo
  rospy
  rviz
  urdf
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package()
#
#find_package(roslaunch)
#
#foreach(dir config launch meshes urdf)
#	install(DIRECTORY ${dir}/
#		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
#endforeach(dir)


catkin_install_python(PROGRAMS 
  scripts/mecanum_teleop.py
  scripts/mecanum_ctrl.py
  scripts/send_cmd_vel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)