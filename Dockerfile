FROM osrf/ros:foxy-desktop
LABEL Name="base-development" Version=4.0
LABEL maintainer "Olmer Garcia <olmerg@gmail.com>"

RUN echo 'source /opt/ros/foxy/setup.bash' >> /root/.bashrc
RUN /bin/bash
