# grvc-ual
[![Releases](https://img.shields.io/github/release/grvcTeam/grvc-ual.svg)](https://github.com/grvcTeam/grvc-ual/releases) [![DOI](https://img.shields.io/badge/DOI-10.1177%2F1729881420925011-blue)](https://doi.org/10.1177/1729881420925011)

A repository for the GRVC UAV abstraction layer.

## Installation and use

Download the latest stable version from [here](https://github.com/grvcTeam/grvc-ual/releases).

Start configuring which backends should be active:

```
    $ cd ~/catkin_ws/src/grvc-ual
    $ ./configure.py
```

You can find detailed instructions for installation and how to use the UAL in the [Wiki](https://github.com/grvcTeam/grvc-ual/wiki).

## Compatibile autopilots

### [PX4](https://github.com/PX4/Firmware)

 * Via [ual_backend_mavros](https://github.com/grvcTeam/grvc-ual/wiki/Backend-MAVROS) and [ual_backend_mavlink](https://github.com/grvcTeam/grvc-ual/wiki/Backend-MAVLink)
 * In simulation (SITL): version [v1.10.*](https://github.com/PX4/Firmware/tree/v1.10.1)
 * Flying: versions from [v1.7.3](https://github.com/PX4/Firmware/tree/v1.7.3) to [v1.10.1](https://github.com/PX4/Firmware/tree/v1.10.1)
 * [Instructions to setup the SITL](https://github.com/grvcTeam/grvc-ual/wiki/Setup-instructions:-PX4-SITL)

### [Ardupilot](http://ardupilot.org/) (beta)

 * Via [ual_backend_mavros](https://github.com/grvcTeam/grvc-ual/wiki/Backend-MAVROS)
 * Last tested version [v4.0.0](https://firmware.ardupilot.org/Copter/stable-4.0.0)

### DJI A3/N3

 * Via [ual_backend_dji_ros](https://github.com/grvcTeam/grvc-ual/wiki/Backend-DJI-ROS)
 * ROS dji_sdk version [TBD]()
 * DJI Onboard SDK version [TBD]()

### Crazyflie

 * Via [ual_backend_crazyflie](https://github.com/grvcTeam/grvc-ual/wiki/Backend-Crazyflie)

### Unreal Engine - Airsim

 * Via [ual_backend_ue](https://github.com/grvcTeam/grvc-ual/wiki/Backend-UE)

## Citation
If you find UAL useful in your research, please consider citing:

```
@article{real_ijars20, 
    author = {Fran Real and Arturo Torres-Gonz\'{a}lez and Pablo Ram\'{o}n Soria and Jes\'{u}s Capit\'{a}n and Anibal Ollero}, 
    title = {Unmanned aerial vehicle abstraction layer: An abstraction layer to operate unmanned aerial vehicles}, 
    journal = {International Journal of Advanced Robotic Systems}, 
    year = {2020}, 
    volume = {17}, 
    number = {4}, 
    pages = {1-13}, 
    doi = {10.1177/1729881420925011},
    url = {https://doi.org/10.1177/1729881420925011}
}
```
