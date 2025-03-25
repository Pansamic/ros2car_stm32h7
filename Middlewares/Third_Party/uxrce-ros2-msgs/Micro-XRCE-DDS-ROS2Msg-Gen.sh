#!/bin/bash
generator=$(find . -name "microxrceddsgen" -exec realpath {} \;)
root_dir=$PWD

if [ ! -f "$generator" ]; then
  echo "Micro-XRCE-DDS-Gen generator not found."
  exit 1
fi

echo "find Micro-XRCE-DDS-Gen generator in $generator."

rm -rf uxrce_ros2_msgs output

mkdir -p output/example
mkdir uxrce_ros2_msgs

# cp -r common_interfaces/* uxrce_ros2_msgs
# cp -r rcl_interfaces/* uxrce_ros2_msgs

cp -r /opt/ros/$ROS_DISTRO/share/builtin_interfaces uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/control_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/common_interfaces uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/visualization_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/shape_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/trajectory_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/unique_identifier_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/stereo_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/sensor_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/lifecycle_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/diagnostic_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/statistics_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/nav_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/geometry_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/rosgraph_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/action_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/std_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/pendulum_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/map_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/pcl_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/tf2_geometry_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/tf2_sensor_msgs uxrce_ros2_msgs
cp -r /opt/ros/$ROS_DISTRO/share/tf2_msgs uxrce_ros2_msgs


cd uxrce_ros2_msgs

# Define an empty array to store the folders
dirlist=$(find . -mindepth 1 -maxdepth 1 -type d)

cd ..

# Loop through the array and do something with each folder
for dir in $dirlist
do
  if [ -d "$root_dir/uxrce_ros2_msgs/$dir/msg" ]; then
    cd $root_dir/uxrce_ros2_msgs/$dir/msg 
    # Do something, the folder is accessible with $f
    for file in *.msg
    do
      if [ -f "$file" ]; then
        ros2 run rosidl_adapter msg2idl.py $file
      fi
    done
  fi
done

# Loop through the array and do something with each folder
for dir in $dirlist
do
  if [ -d "$root_dir/uxrce_ros2_msgs/$dir/msg" ]; then
    cd $root_dir/uxrce_ros2_msgs/$dir/msg 
    mkdir -p $root_dir/output/example/$dir/msg/
    mkdir -p $root_dir/output/$dir/msg/
    # Do something, the folder is accessible with $f
    for file in *.idl
    do
      if [ -f "$file" ]; then
        $generator -I $root_dir/uxrce_ros2_msgs -d $root_dir/output/$dir/msg -example -cs $file
      fi
    done
    mkdir -p $root_dir/output/example/$dir/msg
    mv $root_dir/output/$dir/msg/*Publisher.c $root_dir/output/example/$dir/msg/
    mv $root_dir/output/$dir/msg/*Subscriber.c $root_dir/output/example/$dir/msg/
  fi
done

cd $root_dir

# Loop through all C source files in the project directory
find "$root_dir/output" -type f -name '*.c' | while read -r file; do
    # Skip files in the "example" folder
    if [[ ! "$file" == */example/* ]]; then
        # Extract the interface and message from the file path
        interface=$(echo "$file" | awk -F'/' '{print $(NF-2)}')
        message=$(basename "$file" .c)

        # Update the include statement in the file
        sed -i "s|^#include \"${message}\.h\"|#include \"${interface}/msg/${message}.h\"|" "$file"
        echo "generated $file"
    fi
done
