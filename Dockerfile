FROM osrf/ros:melodic-desktop-full
RUN apt-get update && apt-get install -y apt-utils build-essential psmisc vim-gtk
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN apt-get update && apt-get install -q -y python3
RUN apt-get update && apt-get install -q -y python-catkin-tools
RUN apt-get update && apt-get install -q -y ros-melodic-gazebo-plugins

COPY . /
# RUN cd gazebo_robomaster_gym
RUN apt-get install -y python3-pip
RUN pip3 install -r requirements.txt && pip3 install -e .
EXPOSE 5000
CMD debug_wrapper.sh
CMD tail -f /dev/null
