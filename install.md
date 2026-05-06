start with CM5 with Raspberry Pi OS (bookworm)
install ros2 jazzy as described here: https://github.com/Ar-Ray-code/rpi-bullseye-ros2/tree/jazzy

#install other apt packages:
sudo apt install libcv-bridge-dev python3-cv-bridge python3-bson python3-ujson python3-autobahn python3-twisted python3-tornado python3-systemd distcc python3-flask python3-gunicorn python3-zipstream-ng python3-gevent

#set up distcc
export CMAKE_C_COMPILER=aarch64-linux-gnu-gcc
export CMAKE_CXX_COMPILER=aarch64-linux-gnu-g++
export CMAKE_C_COMPILER_LAUNCHER=distcc
export CMAKE_CXX_COMPILER_LAUNCHER=distcc
export DISTCC_HOSTS={LAPTOP URL}
export DISTCC_SKIP_LOCAL_RETRY=1

#install arducam software 
https://docs.arducam.com/Raspberry-Pi-Camera/Tof-camera/ROS-With-Arducam-ToF-Camera/

edit /boot/firmware/config.txt; edit:
  camera_auto_detect=0
add:
  dtoverlay=arducam-pivariety,cam0
  dtoverlay=arducam-pivariety,cam1 
  dtoverlay=pwm,pin=19,func=2 #may not need this if we do the funky DMA thing from https://github.com/G33KatWork/RP1-Reverse-Engineering/blob/master/pcie/hacks.py

#correct empy version
sudo pip install empy==3.3.4 --break-system-packages

#install mag_cal and rosbags
sudo pip install --break-system-packages circuitpython-mag-cal rosbags pymeshlab

Clone cyclops folder: git clone https://github.com/furbrain/Cyclops.git

#install kalibr
cd Cyclops/kalibr
docker build -t furbrain/kalibr .

#create ros2 workspace
mkdir -p ros2_ws/src
cd ros2_ws/src

#install various packages
git clone git@github.com:RobotWebTools/rosbridge_suite.git -b ros2
git clone git@github.com:furbrain/tf2_web_republisher.git
git clone https://github.com/ros-perception/vision_opencv.git -b rolling

git clone git@github.com:ros-tooling/topic_tools.git
git clone git@github.com:furbrain/ros_mimu_calibrate.git
git clone git@github.com:ros2/geometry2.git -b 0.36.6
git clone https://github.com/RobotWebTools/web_video_server.git
git clone https://github.com/fkie/async_web_server_cpp.git
git clone git@github.com:furbrain/image_common.git -b jazzy
git clone git@github.com:Box-Robotics/ros2_numpy.git -b jazzy
git clone git@github.com:ros-perception/image_pipeline.git -b jazzy # add -fPIC to relevant cxxflags in CMakelists.txt for image_proc and depth_image_proc


git clone https://github.com/christianrauch/camera_ros.git # not sure this is needed...


#Add the following to .colcon/defaults.yaml:
{
    "build": {
        "symlink-install": true,
        "build-base": "/home/pi/ros2_ws/build",
        "install-base": "/home/pi/ros2_ws/install",
        "base-paths": ["/home/pi/ros2_ws/src"]
    },
    "test": {
        "build-base": "/home/pi/ros2_ws/build",
        "install-base": "/home/pi/ros2_ws/install",
        "event-handlers": ["console_direct+"],
        "base-paths": ["/home/pi/ros2_ws/src"],
    }
}


#build it all
cd ..
colcon build --symlink-install



#setup colcon bits and sources
add the following to ~/.bashrc
  source ~/ros2_ws/install/setup.bash
  source /usr/share/colcon_cd/function/colcon_cd.sh
  export _colcon_cd_root=/opt/ros/jazzy/
  
  
# install caddy as http server
sudo apt install -y debian-keyring debian-archive-keyring apt-transport-https
curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/gpg.key' | sudo tee /etc/apt/trusted.gpg.d/caddy-stable.asc
curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/debian.deb.txt' | sudo tee /etc/apt/sources.list.d/caddy-stable.list
sudo apt update
sudo apt install -y caddy
ln -s Cyclops/www www
chmod 770 /home/pi # enable users in pi group to see home dir
sudo usermod -a -G pi caddy # add caddy to pis user group

#edit /etc/caddy/Caddyfile to:
:80 {
        root * /home/pi/www/
        file_server
}

#restart caddy
sudo systemctl reload caddy

#install openMVS
sod it, use docker
cd /home/pi
sudo apt install libceres-dev libcgal-dev libnanoflann-dev libjxl-dev
git clone --branch 2023.12 git@github.com:cnr-isti-vclab/vcglib.git
export VCG_ROOT=/home/pi/vcglib

git clone --recurse-submodules https://github.com/cdcseacave/openMVS.git


