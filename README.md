## Requirement

To use it, make sure you have [Docker](https://docs.docker.com/get-docker/) installed, then build and run the image :

**Be caution:** Don't install Docker Desktop. Please install Docker Engine!

## Clone SOEM by git submodule

`git submodule update`

## Build the Docker File

```shell
$ docker build --tag ati_sensor:humble --file .docker/Dockerfile .
$ docker run -it --network host ati_sensor:humble
```

## Run the ATI sensor

```shell
$ ros2 run ati_sensor_ros ati_sensor_ros
```