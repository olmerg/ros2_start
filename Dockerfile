FROM osrf/ros:foxy-desktop
LABEL Name="base-development" Version=4.0
LABEL maintainer "Olmer Garcia <olmerg@gmail.com>"
RUN apt-get -y update \
    && DEBIAN_FRONTEND=noninteractive \
    apt-get install -y --no-install-recommends \
        # apt-transport-https \
        ros-foxy-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean
RUN echo 'source /opt/ros/foxy/setup.bash' >> /root/.bashrc
CMD /bin/bash
