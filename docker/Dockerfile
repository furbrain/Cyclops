# syntax=docker/dockerfile:1
FROM ros:noetic
ENV APT_REQS="python3-gst-1.0 gstreamer1.0-tools gstreamer1.0-python3-plugin-loader python3-pip"
ENV OPT_APT_REQS="less iputils-ping"
RUN apt update && apt install -y $APT_REQS $OPT_APT_REQS
COPY requirements.txt /
RUN pip3 install --upgrade pip setuptools wheel
RUN pip3 install -r requirements.txt
RUN adduser cyclops && echo cyclops:cyclops | chpasswd
RUN echo "cyclops ALL=(ALL:ALL) ALL, !/bin/bash" > /etc/sudoers.d/custom_sudoers && \
    chmod 0440 /etc/sudoers.d/custom_sudoers
WORKDIR /home/cyclops
USER cyclops

