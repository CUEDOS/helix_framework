[![CI On Pull](https://github.com/CUEDOS/helixio/actions/workflows/test.yml/badge.svg?branch=main&event=pull_request)](https://github.com/CUEDOS/helixio/actions/workflows/test.yml)

<img src="https://github.com/CUEDOS/cascade-demo/blob/main/img/HelixioLogoFinal.svg" alt="" />

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

The goal of this project is to demonstrate a practical implementation of UAV traffic management concepts. The hardware being used for this demonstration is the [PX4 Vision Autonomy Development Kit](https://docs.px4.io/v1.12/en/complete_vehicles/px4_vision_kit.html).

### Built With

* [PX4](https://px4.io/)
* [MAVSDK-Python](https://github.com/mavlink/MAVSDK-Python)
* [pymap3d](https://github.com/geospace-code/pymap3d)
* [Mosquitto](https://mosquitto.org/)
* [paho-mqtt](https://www.eclipse.org/paho/index.php?page=clients/python/index.php)

<!-- GETTING STARTED -->
## Getting Started

Follow the instructions below to install the dependencies required for this project.

### Quick Start

Run the following commands one by one in terminal from your home directory to install cascade-demo and all the required dependencies.

```sh
sudo apt-get update
```

```sh
sudo apt-get install git
```

```sh
git clone https://github.com/CUEDOS/cascade-demo
```

```sh
sudo ./cascade-demo/setup.sh
```

The last script will take while to run and will install all required python packages, the Mosquitto MQTT broker, the PX4 Firmware and the Gazebo simulation environment. Once the script finishes it will print 'Installation Complete'. At this point **restart your computer**.

Once your computer has restarted run the following commands in terminal one by one.

```sh
cd PX4-Autopilot
```

```sh
make px4_sitl gazebo
```

After a few minutes a gazebo window will open with a single drone on a flat ashphalt world. Close this window and the terminal window.

Finally right click on the file called 'cascade-demo.desktop' on your desktop and select 'Allow Launching'. The icon will change to the CASCADE logo and you will now be able to open the gui using this app icon.

  
### Installation
See the wiki for installation advice for Ubuntu, Gazebo, PX4, VS Code and more
