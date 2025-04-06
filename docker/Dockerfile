# syntax=docker/dockerfile:1
FROM ros:noetic
ENV APT_REQS="python3-gst-1.0 python3-rpi.gpio gstreamer1.0-tools gstreamer1.0-python3-plugin-loader gstreamer1.0-plugins-good v4l-utils ros-noetic-cv-camera ros-noetic-tf2 ros-noetic-usb-cam ros-noetic-image-proc ros-noetic-tf2-ros python3-pip openssh-server netcat avahi-utils clinfo ocl-icd-opencl-dev distcc libepoxy-dev software-properties-common git build-essential cmake libeigen3-dev libopencv-dev libegl1-mesa-dev libglew-dev"
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
RUN mkdir -p /lib/firmware/
RUN mkdir -p /etc/OpenCL/vendors/
ADD https://github.com/JeffyCN/mirrors/raw/refs/heads/libmali/lib/aarch64-linux-gnu/libmali-bifrost-g52-g13p0-gbm.so /lib/firmware
RUN echo /lib/firmware/libmali-bifrost-g52-g13p0-gbm.so > /etc/OpenCL/vendors/mali.icd


#
# install ORBSLAM3 ROS package
#

RUN apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-hector-trajectory-server \
        python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

WORKDIR /root

RUN git clone https://github.com/stevenlovegrove/Pangolin.git --branch v0.6 --depth=1 && \
    cd Pangolin && \
    mkdir build && cd build && \
    export DISTCC_HOSTS=192.168.1.137 && \
    cmake .. -DCMAKE_C_COMPILER=/usr/bin/aarch64-linux-gnu-gcc -DCMAKE_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++ -DCMAKE_C_COMPILER_LAUNCHER=distcc -DCMAKE_CXX_COMPILER_LAUNCHER=distcc && \
    make -j 4 && \
    make install    

WORKDIR /catkin_ws/src/
RUN git clone https://github.com/thien94/orb_slam3_ros.git
# remove -march=native from arguments - prevents distcc from working!
RUN sed -i 's/ -march=native//g' orb_slam3_ros/CMakeLists.txt

SHELL ["/bin/bash", "-c"]
WORKDIR /catkin_ws/
RUN source /opt/ros/$ROS_DISTRO/setup.sh && catkin init

RUN source /opt/ros/$ROS_DISTRO/setup.sh && \
    export DISTCC_HOSTS=192.168.1.137 && \
    catkin config --extend /opt/ros/noetic && \
    catkin build orb_slam3_ros -j 4 --cmake-args -DCMAKE_C_COMPILER=/usr/bin/aarch64-linux-gnu-gcc -DCMAKE_CXX_COMPILER=/usr/bin/aarch64-linux-gnu-g++ -DCMAKE_C_COMPILER_LAUNCHER=distcc -DCMAKE_CXX_COMPILER_LAUNCHER=distcc
    
WORKDIR /catkin_ws/src/
RUN git clone https://github.com/furbrain/sipeed-tofv075-ros.git
RUN source /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build sipeed_tof_cpp

RUN pip install setuptools==70.3.0    

RUN git clone https://github.com/furbrain/ros_mimu_calibrate.git
RUN source /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build mimu_calibrate

RUN git clone https://github.com/orocos/orocos_kinematics_dynamics.git
RUN git clone https://github.com/ros/geometry.git
RUN git clone https://github.com/ros/geometry2.git
RUN git clone https://github.com/ros-perception/image_pipeline.git -b noetic
RUN git clone https://github.com/CCNYRoboticsLab/imu_tools
RUN source /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build orocos_kdl
RUN source /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build depth_image_proc 
RUN source /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build imu_filter_madgwick
RUN git clone https://github.com/ros-perception/image_transport_plugins.git -b noetic-devel
RUN apt update && apt install -y libturbojpeg0-dev
RUN source /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin build compressed_image_transport compressed_depth_image_transport
    
RUN echo "source /home/cyclops/Cyclops/ROS/devel/setup.bash" >> /home/cyclops/.bashrc

CMD ["/usr/sbin/sshd", "-D", "-p", "2000"]

