# Installing TacticalFrog

A device runs three separately built releases, each in `/opt/<name>/<version>` with a `current` link to the
one in use:

| Release | From | Contents |
| --- | --- | --- |
| TacticalFrog | this repository | the web UI, services, modelling scripts and the `tactical_frog` ROS package |
| orb_slam3 | [furbrain/orb_slam3_ros](https://github.com/furbrain/orb_slam3_ros) (submodule `ROS2/orb_slam3_ros`) | the SLAM node, as a colcon prefix |
| OpenMVS | [furbrain/openMVS](https://github.com/furbrain/openMVS) (submodule `openMVS`) | the reconstruction tools, statically linked |

Everything else (ROS 2 jazzy, the camera stack, `~/ros2_ws`, kalibr, the system configuration) is installed
once by `bootstrap.sh`.

## A new device, from the disk image

Each release has a disk image, `tacticalfrog-VERSION-cm5.img.xz`: Raspberry Pi OS Lite (bookworm) with
everything below already done and all three releases installed. Images over GitHub's 2GB limit for release
files are only an artifact of the "Disk image" workflow run, kept for 5 days.

1. Write it to the SD card or eMMC with Raspberry Pi Imager ("Use custom"), or any image writer.
   Imager's OS customisation can set a password and turn on SSH. The username must be `pi`, as TacticalFrog
   needs it. Without customisation, `pi` has no password, so you can't log in, but the web UI still works.
2. Boot it. The first boot grows it to fill the card and reboots, which takes a minute or two.
3. Join the "TacticalFrog" wifi hotspot and go to http://frog.local, then calibrate the cameras and IMU.

The "Disk image" workflow (`.github/workflows/image.yml`, `.github/image/build.sh`) can also build one by hand
from any release, or from a commit.

## A new device, by hand

1. Install stock Debian 12 (bookworm) or Raspberry Pi OS Lite (bookworm) for arm64 on the CM5.
2. Run `bootstrap.sh` as root. It needs internet and takes an hour or more. It doesn't need a checkout of this
   repository, and it's safe to run again, e.g. after a network failure:
   ```bash
   curl -fsSLO https://raw.githubusercontent.com/furbrain/TacticalFrog/main/bootstrap.sh
   sudo ./bootstrap.sh
   ```
   Settings go before it, e.g. `sudo FROG_HOSTNAME=frog2 ./bootstrap.sh`; see the top of the script.
   The log is `/var/log/tacticalfrog-bootstrap.log`.
3. Reboot. Wifi then starts as the open "TacticalFrog" hotspot.
4. Install the three releases, in any order, from their GitHub releases. Use the orb_slam3 and OpenMVS
   versions named in the TacticalFrog release's `RELEASES` file. For each of TacticalFrog, orb_slam3 and OpenMVS:
   ```bash
   sudo tar -xzf NAME-VERSION-aarch64.tar.gz -C /opt/NAME --no-same-owner
   sudo ln -sfn NAME-VERSION /opt/NAME/current
   ```
   Everything starts by itself a few seconds after the third: http://frog.local
5. Calibrate the cameras and IMU from the web page, as they're different on every device.
6. Optional, for the "frog" file sharing login that Windows needs: `sudo smbpasswd -a pi`

`bootstrap.sh` refuses to run on a development device (see below), so that it can't wreck one.

## Updates

Settings › Software update installs the latest TacticalFrog release together with the orb_slam3 and OpenMVS
releases it was built with. The page also shows an alert when an update can be installed. Updating needs an
internet connection, and the battery charging or over 50%.

All three are downloaded, checked and unpacked next to the versions in use before anything is switched, so a
failed update changes nothing. Recording and the web page stop for about a minute while the services restart.

By hand:
```bash
sudo systemctl start tacticalfrog-update
```
```bash
journalctl -u tacticalfrog-update
```

The previous versions are kept. To go back, point the three `current` links at them again (`ls /opt/NAME`
shows what's there), then reload the services, as their unit files come from the TacticalFrog release:
```bash
sudo systemctl daemon-reload
```
```bash
sudo systemctl restart multicast-loopback beep_server caddy smbd gunicorn ros2_tactical_frog
```

## Making a release

Pushing a tag to a repository builds its release and publishes it on GitHub (`.github/workflows/release.yml`).
Any tag publishes a release.

Each TacticalFrog release records, in `RELEASES`, the orb_slam3 and OpenMVS commits it was built with (its
submodules) and their release tags, and devices update to those along with it. So release them in this order:

1. Tag orb_slam3 and/or OpenMVS at the commits you want, and wait for their releases to be published.
2. Move the submodules in this repository to those commits, and commit.
3. Tag this repository. Its release workflow refuses to publish if either submodule commit has no release.

The "Bootstrap test" workflow (`.github/workflows/bootstrap-test.yml`) builds all three from the submodule
commits without publishing them, bootstraps stock Debian in a container and checks that they all work together.
It runs on pushes to `bootstrap-test` and `claude/` branches, or by hand. It can't test the CM5's boot
configuration, cameras, IMU, beeper or hotspot.

## A development device

A development device has a git checkout as `/opt/TacticalFrog/current` and builds orb_slam3 and OpenMVS
itself. These are the notes it was set up from; `bootstrap.sh` does the same for a release device.

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

#orb_slam3 and OpenMVS are updated separately, so each has its own "current" link
# (these can be made before either is built; a development device links to its local builds)
# orb_slam3/current is a colcon install prefix for that one package (has lib/ and share/)
# OpenMVS/current is the folder holding the binaries
sudo mkdir -p /opt/orb_slam3 /opt/OpenMVS
sudo ln -sfn /home/pi/ros2_ws/install/orb_slam3 /opt/orb_slam3/current
sudo ln -sfn /usr/local/bin/OpenMVS /opt/OpenMVS/current

#install kalibr: furbrain/kalibr, which needs neither ROS nor docker (bootstrap.sh's install_kalibr does this,
# at the commit it names)
sudo apt install libsuitesparse-dev python3-igraph python3-pyx
git clone https://github.com/furbrain/kalibr.git
cmake -S kalibr -B kalibr/build -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DCMAKE_INSTALL_PREFIX=/usr/local/lib/kalibr -DKALIBR_PYTHON_INSTALL_DIR=/usr/local/lib/kalibr/python \
  -DKALIBR_BIN_INSTALL_DIR=/usr/local/lib/kalibr/bin
cmake --build kalibr/build -j4 && sudo cmake --install kalibr/build
echo /usr/local/lib/kalibr/python | sudo tee /usr/local/lib/python3.11/dist-packages/kalibr.pth
sudo ln -sfn /usr/local/lib/kalibr/bin/kalibr_* /usr/local/bin/

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
