FROM osrf/ros:humble-desktop

COPY . /ati_ws/src/ati_sensor_ros

# Enable UDP transport to allow multiusers to connect to the same ROS2 node
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /ati_ws && \
    colcon build"

RUN touch /root/.bashrc && \
    echo "source /ati_ws/install/setup.bash" >> /root/.bashrc && \
    echo "source /ati_ws/src/ati_sensor_ros/ati_environment.bash" >> /root/.bashrc

