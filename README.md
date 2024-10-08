## Requirement

To use it, make sure you have [Docker](https://docs.docker.com/get-docker/) installed, then build and run the image :

**Be caution:** Don't install Docker Desktop. Please install Docker Engine!

## Clone this repository

```shell
$ git clone https://github.com/smartrobotsdesignlab/ATI_ROS2.git

cd ati_sensor_ros
```

## Clone SOEM by git submodule

`git submodule update --init --recursive`

## Build the Docker File

```shell
$ docker build --tag ati_sensor:humble --file .docker/Dockerfile .
$ docker run -it --network host --rm ati_sensor:humble
```

## Run the ATI sensor

```shell
$ ros2 run ati_sensor_ros ati_sensor_ros
```
