# nvidia-egl-desktop-ros2

## Introduction

This is a Dockerfile to use ROS2 on Ubuntu MATE Desktop container with NVIDIA GPU.  
This Dockerfile is based on [ehfd/docker-nvidia-egl-desktop](https://github.com/ehfd/docker-nvidia-egl-desktop).

![](nvidia-egl-desktop-ros2-screenshot.png)

## Requirements

- NVIDIA graphics driver 450.80.02+ [^1]
- Docker
- nvidia-docker2

## Build docker image

### ROS2 Foxy

```
cd foxy
docker build -t nvidia-egl-desktop-ros2:foxy .
```

### ROS2 Galactic

```
cd galactic
docker build -t nvidia-egl-desktop-ros2:galactic .
```

## Launch docker container

Execute the command described below. And, browse <http://127.0.0.1:6080/>.  
In this docker container, default account is `user`.

If you hope to customize, please read <https://github.com/ehfd/docker-nvidia-egl-desktop/blob/main/README.md>.

### ROS2 Foxy

```
docker run --gpus 0 -it --shm-size=1024m -e SIZEW=1920 -e SIZEH=1080 -e PASSWD=mypasswd -e BASIC_AUTH_PASSWORD=mypasswd -e NOVNC_ENABLE=true -p 6080:8080 nvidia-egl-desktop-ros2:foxy
```

### ROS2 Galactic

```
docker run --gpus 0 -it --shm-size=1024m -e SIZEW=1920 -e SIZEH=1080 -e PASSWD=mypasswd -e BASIC_AUTH_PASSWORD=mypasswd -e NOVNC_ENABLE=true -p 6080:8080 nvidia-egl-desktop-ros2:galactic
```

[^1]: <https://github.com/ehfd/docker-nvidia-egl-desktop/blob/main/README.md>