#execute with docker build -t <image_name> .

FROM ros:indigo-ros-core
MAINTAINER Jakob Hirscheider <hirschei@in.tum.de>

#required for catkin_make
RUN apt-get update && apt-get install build-essential -y

#build a catkin workspace
RUN mkdir /catkin_ws/src -p
WORKDIR /catkin_ws/src/
RUN /bin/bash -c "source /opt/ros/indigo/setup.bash \
    && catkin_init_workspace \
    && catkin_create_pkg beginner_tutorials std_msgs rospy roscpp"
WORKDIR /catkin_ws/
RUN /bin/bash -c "source /opt/ros/indigo/setup.bash \
    && catkin_make"

#add external files
RUN mkdir /catkin_ws/src/beginner_tutorials/scripts
COPY transformData.py /catkin_ws/src/beginner_tutorials/scripts/
RUN chmod +x /catkin_ws/src/beginner_tutorials/scripts/transformData.py
COPY external_controller.sh /

