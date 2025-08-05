FROM ros:foxy

RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-foxy-rviz2
RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc
