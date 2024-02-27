# RaiSim

RaiSim is a physics engine for robotics and artificial intelligence research that provides efficient and accurate simulations for robotic systems. We specialize in running rigid-body simulations while having an accessible, easy to use C++ library.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/CN0ah5-OWik/0.jpg)](https://www.youtube.com/watch?v=CN0ah5-OWik)

## News
Closed-loop system simulation is now available! Check out the [minitaur example](https://github.com/raisimTech/raisimLib/tree/master/examples/src/server/minitaur.cpp)

## Dependencies

### - Eigen3
##### Ubuntu
```bash
sudo apt install libeigen3-dev
```

#### Windows
Install desired release [here](http://eigen.tuxfamily.org) and unzip file at C:\Program, files for better discoverability.

### - CMake
Install version > 3.10 [here](https://cmake.org/download/).

### - Visual Studio 2019
Install [here](https://visualstudio.microsoft.com/vs/older-downloads/), make sure to install C++ module by checking the box during installation.

## Installation

Further documentation available on the [RaiSim Tech website](http://raisim.com).

## Features
- Supports free camera movement
- Allows the recording of screenshots and recordings
- Contact and collision masks
- Materials system to simulate different textures
- Height maps to create different sytles of terrain
- Ray Test to create collision checkers 

## Troubleshooting
- Ensure that all versions of dependencies fit the documentation. etc.(Visual Studio 2019, CMake verson > 3.10)
- If run into problem with executing into the raisimUnity.x86_64 file, ensure that your graphics card driver is compatible with current graphics card. (Cannot use the default open-source graphics card driver nouveau)
- Make sure to use raisimUnity natively and not on a docker.
- If using Linux, install minizip, ffmpeg, and vulkan.
- If drivers don't support vulkan, use raisimUnityOpengl instead of raisimUnisty. Found in raisimUnityOpengl directory.
- Make sure to set environment variable to $LOCAL_INSTALL when installing raisim.

## License

You should get a valid license and an activation key from the [RaiSim Tech website](http://raisim.com) to use RaiSim.
Post issues to this github repo for questions. 
Send an email to info.raisim@gmail.com for any special inquiry.

## Supported OS

MAC (including m1), Linux, Windows.







