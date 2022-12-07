# EtherCAT Driver ROS2 Docker Containers
Provides a basic preconfigured docker container for development purposes. To use it, make sure you have [Docker](https://docs.docker.com/get-docker/) installed, then build and run the image :

```shell
$ docker build --tag ati_sensor:humble --file .docker/Dockerfile .
$ docker run -it --network host ati_sensor:humble
```