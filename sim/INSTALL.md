# Installation checklist

This is a minimal, manual checklist for bringing up ArduPilot SITL + gz sim.
Commands are examples; adjust paths to your environment.

## 1) System prerequisites (Ubuntu)
```
sudo apt update
sudo apt install -y git python3 python3-pip python3-venv build-essential
sudo apt install -y gz-harmonic gz-fuel-tools
```

If `gz-harmonic` is not found, add the OSRF repo and retry:
```
sudo curl -sSL https://packages.osrfoundation.org/gazebo.key \
  -o /usr/share/keyrings/gazebo-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install -y gz-harmonic gz-fuel-tools
```

## 2) ArduPilot SITL
```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter
```

## 3) Gazebo plugin for ArduPilot
```
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## 4) Run SITL + Gazebo
See `start_sitl.sh` for example commands.

## 5) Verify MAVLink
```
mavproxy.py --master=udp:127.0.0.1:14550
```
