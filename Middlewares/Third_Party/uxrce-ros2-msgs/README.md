# Micro-XRCE-DDS-ROS2-Msg

ROS2 messages and its generation tool for Micro-XRCE-DDS-Client.

## Usage

1. add this folder to your include list.
2. compile source files except for `example` folder.
3. check `example` folder for usage examples.

## Generate

You can use the sources directly, but if you want latest ones, you need to generate them again. Follow the steps beneath.

### Docker Container

If your machine doesn't have ROS2 environment, you can use docker to generate.

```bash
git clone https://github.com/Pansamic/Micro-XRCE-DDS-ROS2-Msg
docker build -t uxrce-ros2-msg-gen .
docker run -it --rm -v $PWD:/project uxrce-ros2-msg-gen
```

### Local Machine

If your machine has ROS2, you can just execute script.
```bash
git clone https://github.com/Pansamic/Micro-XRCE-DDS-ROS2-Msg
bash Micro-XRCE-DDS-ROS2Msg-Gen.sh
```
