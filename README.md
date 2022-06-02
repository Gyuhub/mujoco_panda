# mujoco_panda
MuJoCo robot simulator using Franka Emika panda model

## Requirements
- ROS2 (foxy)
- Eigen (more than 3)
- RBDL & RBDL URDFReader (ORB version)
- Mujoco (MUlti-JOint dynamics with COntact)
- GLEW
- OpenGL

## Installations
- ROS2

- Eigen (Eigen is automatically installed when installing Ubuntu)
-- You can check either the Eigen is installed or not by entering the command
```
pkg-config --modversion eigen3
```

if the Eigen library is successfully installed, you will see the version of Eigen. Or not, just enter the command below to terminal
```
sudo apt install libeigen3-dev
```
- RBDL
```
git clone https://github.com/rbdl/rbdl-orb
cd rbdl
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -DRBDL_BUILD_ADDON_URDFREADER=ON -DRBDL_USR_ROS_URDF_LIBRARY=OFF ..
make all
make install
sudo ldconfig
```

- Mujoco
Mujoco installation link : https://mujoco.org/download
If you enter the link successfully, all you need to do is just push the button which has a name of "Linux"
```
cd
mkdir Simulations
```
After download the file, you can Extract the file to the location of "~/Simulations"

- GLEW
```
sudo apt install libglew-dev
```

- OpenGL
```
sudo apt install build-essential
sudo apt install freeglut3-dev libglu1-mesa-dev mesa-common-dev
```
After that you can check either the OpenGL is successfully installed or not by entering the command below
```
ls /usr/include/GL
```

Contents
├─include
│   | control_math.h
│   | controller.hpp
│   | model.h
|   └─trajectory.h
├─model
│   ├─meshes
|   |   ├─collision
|   |   └─visual
│   | assets.xml
│   | franka_panda.urdf
│   | franka_panda.xml
│   | franka_panda_rviz.urdf
│   | gripper.xml
|   └─objects.xml
├─rviz
|   └─franka_panda.rviz
├─src
│   | controller.cpp
│   | model.cpp
│   | simulate.cc
|   └─trajectory.cpp
├─srv
|   └─Commands.srv
| CMakeLists.txt
| README.md
└─package.xml
