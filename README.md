<table style="height: 96px; width: 100%; border-collapse: collapse;" border="0">
<tbody>
<tr style="height: 96px;">
<td style="width: 20%; height: 96px;"><img src="https://github.com/CUEDOS/cascade-demo/blob/return_dev/img/HelixioLogoFinalSmall.svg" width="60" alt="" /></td>
<td style="width: 80%; text-align: center; height: 96px;">
<img src="https://github.com/CUEDOS/cascade-demo/blob/return_dev/img/HelixioLogoFinal.svg" alt="" />
</td>
<td style="width: 20%; height: 96px;"><img src="https://cascadeuav.files.wordpress.com/2018/03/cropped-white.png" alt="" width="60" /></td>
</tr>
</tbody>
</table>

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

### Prerequisites

This project uses python 3, make sure you have pip3 installed on your system and run the following commands in a terminal.
* MAVSDK
  ```sh
  pip3 install mavsdk
  ```
* pymap3d
  ```sh
  pip3 install pymap3d
  ```
* paho-mqtt
  ```sh
  pip3 install paho-mqtt
  ```
### Installation
