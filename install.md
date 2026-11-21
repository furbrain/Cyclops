start with CM5 with Raspberry Pi OS (bookworm)
install ros2 jazzy as described here: https://github.com/Ar-Ray-code/rpi-bullseye-ros2/tree/jazzy

#install other apt packages:
sudo apt install libcv-bridge-dev python3-cv-bridge python3-bson python3-ujson python3-autobahn python3-twisted python3-tornado python3-systemd distcc python3-flask python3-gunicorn python3-zipstream-ng python3-gevent autoconf-archive python3-networkx samba wsdd smbclient

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

#clone TacticalFrog and link it to /opt/TacticalFrog/current (paths used by services and configs)
cd /home/pi
git clone https://github.com/furbrain/TacticalFrog.git
sudo mkdir -p /opt/TacticalFrog
sudo ln -sfn /home/pi/TacticalFrog/ /opt/TacticalFrog/current

#install kalibr
cd /opt/TacticalFrog/current/kalibr
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
git clone git@github.com:ros/diagnostics.git -b ros2
git clone git@github.com:AgoraRobotics/ros2-system-monitor.git

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
ln -s /opt/TacticalFrog/current/www www
chmod 770 /home/pi # enable users in pi group to see home dir
sudo usermod -a -G pi caddy # add caddy to pis user group
# no need to edit /etc/caddy/Caddyfile: caddy.service (below) uses raspbian/Caddyfile from this repo

#set up samba file access
sudo smbpasswd -a pi # "frog" login maps to pi, see raspbian/smbusers
sudo systemctl disable --now nmbd # NetBIOS not used

#allow the web UI (gunicorn, running as pi) to manage wifi via NetworkManager
# the filename must sort before 49-polkit-pkla-compat.rules, otherwise the vendor
# .pkla default (no system connection changes from inactive sessions) wins
sudo install -m 644 /opt/TacticalFrog/current/raspbian/10-nm-wifi.rules /etc/polkit-1/rules.d/
nmcli general permissions | grep modify.system # should say yes

#install systemd services
# caddy.service and smbd.service replace the packaged units so they use the configs in raspbian/
cd /etc/systemd/system
for u in beep_server caddy gunicorn multicast-loopback ros2_tactical_frog smbd; do
  sudo ln -sfn /opt/TacticalFrog/current/raspbian/$u.service $u.service
done
sudo systemctl daemon-reload
sudo systemctl reenable beep_server caddy gunicorn multicast-loopback ros2_tactical_frog smbd
sudo systemctl restart beep_server caddy gunicorn multicast-loopback ros2_tactical_frog smbd

#reload caddy after editing raspbian/Caddyfile (no sudo needed)
caddy reload --config /opt/TacticalFrog/current/raspbian/Caddyfile

#install openMVS
sod it, use docker
cd /home/pi
sudo apt install libceres-dev libcgal-dev libnanoflann-dev libjxl-dev
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg; ./bootstrap-vcpkg.sh
cd ..
echo "export VCPKG_ROOT=/home/pi/vcpkg" >> .bashrc
git clone --branch 2023.12 git@github.com:cnr-isti-vclab/vcglib.git
export VCG_ROOT=/home/pi/vcglib

git clone --recurse-submodules https://github.com/cdcseacave/openMVS.git


