# syntax=docker/dockerfile:1
FROM ros:noetic
ENV APT_REQS="python3-gst-1.0 python3-rpi.gpio gstreamer1.0-tools gstreamer1.0-python3-plugin-loader gstreamer1.0-plugins-good v4l-utils ros-noetic-cv-camera ros-noetic-tf2 ros-noetic-usb-cam ros-noetic-image-proc ros-noetic-tf2-ros python3-pip openssh-server netcat avahi-utils"
ENV OPT_APT_REQS="less iputils-ping"
RUN apt update && apt install -y $APT_REQS $OPT_APT_REQS
COPY requirements.txt /
RUN pip3 install --upgrade pip setuptools wheel
RUN pip3 install -r requirements.txt
RUN mkdir /var/run/sshd
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd
EXPOSE 2000
RUN adduser cyclops && echo cyclops:cyclops | chpasswd
RUN adduser cyclops video
RUN echo "cyclops ALL=(ALL:ALL) ALL, !/bin/bash" > /etc/sudoers.d/custom_sudoers && \
    chmod 0440 /etc/sudoers.d/custom_sudoers
#
# install ORBSLAM3 ROS package
#

RUN apt-get install -y --no-install-recommends \
        software-properties-common \
        git \
        build-essential \
        cmake \
        libeigen3-dev \
        ros-${ROS_DISTRO}-hector-trajectory-server \
        python3-catkin-tools \
        libopencv-dev \
        libegl1-mesa-dev && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN apt-get update && apt-get install -y --no-install-recommends libepoxy-dev

WORKDIR /root

#RUN git clone https://github.com/stevenlovegrove/Pangolin.git --depth=1 && \
#    cd Pangolin && \
#    mkdir build && cd build && \
#    cmake .. && \
#    make -j 1 && \
#    make install    
#
#USER cyclops
#
#RUN cd arducamble_ws/src && \
#    git clone https://github.com/thien94/orb_slam3_ros.git && \
#    cd .. && \
#    catkin config \
#      --extend /opt/ros/noetic && \
#    catkin build
#    
USER root

CMD ["/usr/sbin/sshd", "-D", "-p", "2000"]

