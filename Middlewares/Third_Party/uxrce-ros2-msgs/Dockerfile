FROM osrf/ros:humble-desktop

COPY ./entrypoint.sh /home
COPY ./Micro-XRCE-DDS-ROS2Msg-Gen.sh /home

ENTRYPOINT ["/bin/bash", "/home/entrypoint.sh"]
