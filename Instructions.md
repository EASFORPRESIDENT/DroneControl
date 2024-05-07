# Simulation (PX4-Autopilot) guide

```bash
git clone https://github.com/EASFORPRESIDENT/DroneControl.git
```

## Get MAVSDK
```bash
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK
git submodule update --init --recursive
cmake -DCMAKE_BUILD_TYPE=Release -Bbuild/default -H.
cmake --build build/default -j4
sudo cmake --build build/default --target install
```

## Add plugin to Gazebo

Open terminal in PX4-Autopilot folder: 
```bash
gedit Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world
```

Within the world bracket, add:
```bash
<plugin name="GazeboPlugin" filename="/home/.../DroneControl/GazeboPlugin/build/libGazeboPlugin.so"/>
```
Change the "..." to the file path that leads to DroneControl folder

## Make the files

Open terminal in DroneControl folder:
```bash
cd GazeboPlugin
mkdir build && cd build
cmake ..
make

cd ../..
cd MAVSDK
mkdir build && cd build
cmake ..
make
```

## Using scripts

GazeboPlugin will start when gazebo starts:
```bash
make px4_sitl gazebo-classic
```

To start MainTest, go to DroneControl folder:
```bash
cd MAVSDK/build
./MainTest udp://:14540
```