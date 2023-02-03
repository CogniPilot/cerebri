# Cerebri

Cerebri is a minimalistic flight controller that is designed to simplify verification & validation and ease development.

* Verification and Validation
  * Minimimum viable product/ minimum lines of code for each vehicle, avoid branching
  * Use C++ 17 standard, following industry guidelines
  * Enforce code test coverage and signing
  * Generated estimator/control code with tracking error bound proofs
* Development
  * Algorithm deployment from python (minimal knowledge of C/C++ Zephyr necessary)
  * Optional Docker containers for development
  * Documented debugging process, Software/Hardware in the loop support
  * Designed for interface with ROS2
  * Ensure documentation quality with mathematical derivations and references
  * Minimize developer maintenance by designing for out of tree custom vehicles and
    with several officially supported examples

---------------------------
## Official Support

* Development OS
  * Ubuntu Linux 22.04
* Simulators
  * Gazebo Garden
* Vehicles
  * HGDRONEK66 (simulation in development, hardware planned)
  * MR-BUGGY3 (planned)

---------------------------
## Native Installation

---------------------------
## Docker Installation

The necessary dependencies are docker and hardware-acceleration for docker if you have an NVidia graphics card. Also, we will install Visual Studio Code as the development environment.

### On host

#### Install Visual Studio Code (or preferred IDE)
https://code.visualstudio.com/docs/setup/linux

#### Install Docker
https://docs.docker.com/engine/install/ubuntu/
Dont' forget: https://docs.docker.com/engine/install/linux-postinstall/

### Hardware Acceleration

#### NVidia Container Toolkit
To run gazebo within the container you will need an nvidia graphics card and
should install the Nvidia Container Toolkit.

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

### Host-based Gazebo
Note, you can run gazebo natively to get better performance and only need to set the gazebo partition before
launching.

```bash
export GZ_PARTITION=cognipilot
gz sim quad.sdf
```

---------------------------
## Build

### Clone the Repository

```bash
git clone https://github.com/CogniPilot/cerebri
```
or with push access
```bash
git clone git@github.com:CogniPilot/cerebri
```

### Docker Command Line Development

```bash
cd cerebri/docker
docker compose up
```

#### Hardware Rendering

Create a docker override, this file will be ignored by .git so you
can use it as a location to put any customizations.

docker/docker-compose.override.yml

```yaml
services:

  cerebri:

    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
```

### VSCode Development

You can configure VSCode to create a devcontainer for docker by creating a .devcontainer.json file in the root of the repository.

#### .devcontainer.json  (Software Rendering)
```json
{
  "name": "cerebri",
  "dockerComposeFile": "docker/docker-compose.yml",
  "service": "cerebri",
  "workspaceFolder": "/workdir/cerebri",
  "shutdownAction": "stopCompose"
}
```

#### .devcontainer.json  (NVidia GPU)
```json
{
  "name": "cerebri",
  "dockerComposeFile": "docker/docker-compose-nvidia.yml",
  "service": "cerebri",
  "workspaceFolder": "/workdir/cerebri",
  "shutdownAction": "stopCompose"
}
```

After creating .devcontainer.json in the root of the cerebri repository,
start visual studio code in cerebri directory and select yes, when asked if you would like to reopen folder to develop in container.

### Build Zephyr

Now we invoke the west command to build the Zephyr project:
```bash
cd /workdir/cerebri
west init -l .
west update
west build
```

---------------------------
## Simulation

For the simulation, we need to start both gazebo and the zephyr cerebri binary.

### Start Gazebo

Note that the GZ_PARTITION is already set on the docker image, but will be necessary if you
run the simulator on your host machine.

```bash
export GZ_PARTITION=cognipilot
gz sim quad.sdf
```

### Start Cerebri

```bash
./build/zephyr/zephyr.elf
```

Hit play on the gazebo gui and the simulator should connect.

### Joystick Control

Start ros2 joystick and ros_gz bridge.

```bash
./scripts/joy.sh
```
