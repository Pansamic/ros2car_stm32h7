#!/bin/bash

if [ ! -d "/project" ]; then
  echo 'Does not mount host directory to container, please use \"docker run -it --rm -v $PWD:/project uxrce-ros2-msg-gen\" instead.'
  exit 1
fi

cd /home

apt update && apt install -y ros-$ROS_DISTRO-ros2-control

git clone https://github.com/eProsima/Micro-XRCE-DDS-Gen
# Loop until git clone succeeds or the user interrupts
while true; do
  # Try to clone the repo
  git clone https://github.com/eProsima/Micro-XRCE-DDS-Gen

  # Check the exit status of the git clone command
  if [ $? -eq 0 ]; then
    # Exit status is 0, which means git clone succeeded
    echo "Git clone successful"
    break
  else
    # Exit status is not 0, which means git clone failed
    echo "Git clone failed, retrying in 5 seconds"
    # Remove the destination directory if it exists
    rm -rf Micro-XRCE-DDS-Gen
    # Wait for 10 seconds before retrying
    sleep 5
  fi
done
cd Micro-XRCE-DDS-Gen
git submodule init
git submodule update
./gradlew assemble

source /opt/ros/humble/setup.bash

cd /home

bash /home/Micro-XRCE-DDS-ROS2Msg-Gen.sh

cp -r /home/output/* /project

