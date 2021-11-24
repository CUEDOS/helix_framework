#!/usr/bin/env bash

if [[ "$(id -u)" != "0" ]]; then
   echo "THIS SCRIPT MUST BE RUN AS ROOT (sudo ./setup.sh)"
   sleep 3
   exit 1
  else
   echo "THIS SCRIPT WILL BE EXECUTED AS ROOT   "
   sleep 3
   clear
fi

function echo_block() {
    echo "----------------------------"
    echo $1
    echo "----------------------------"
}

function check_installed_pip() {
   ${PYTHON} -m pip > /dev/null
   if [ $? -ne 0 ]; then
        echo_block "Installing Pip for ${PYTHON}"
        sudo apt-get update
        sudo apt-get install pip3
   fi
}

function check_installed_python() {
    if [ -n "${VIRTUAL_ENV}" ]; then
        echo "Please deactivate your virtual environment before running setup.sh."
        echo "You can do this by running 'deactivate'."
        exit 2
    fi

    for v in 9 8 7
    do
        PYTHON="python3.${v}"
        which $PYTHON
        if [ $? -eq 0 ]; then
            echo "using ${PYTHON}"
            check_installed_pip
            return
        fi
    done

    echo "No usable python found. Please make sure to have python3.7 or newer installed"
    exit 1
}

check_installed_python

check_installed_pip

echo_block "Installing Mavsdk"
${PYTHON} -m pip install mavsdk --upgrade

echo_block "Installing pymap3d"
${PYTHON} -m pip install pymap3d --upgrade

echo_block "Installing paho-mqtt"
${PYTHON} -m pip install paho-mqtt --upgrade

echo_block "Installing numpy"
${PYTHON} -m pip install numpy --upgrade

echo_block "Installing mosquitto MQTT broker"
sudo apt-get update
sudo apt-get install mosquitto

cd $USER_HOME
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx


set -eou pipefail
IFS=$'\n\t'
setterm --reset

clear

USER_HOME=$(getent passwd $SUDO_USER | cut -d: -f6)

# Delete old installations
rm -f $USER_HOME/cascade-demo/cascade-demo.desktop
rm -f $USER_HOME/.local/share/applications/cascade-demo.desktop
rm -f $USER_HOME/Desktop/cascade-demo.desktop
rm -f /usr/share/applications/cascade-demo.desktop


# Create .desktop file
echo "
[Desktop Entry]
Version=1.0
Name=Cascade Demo
Icon=`echo $USER_HOME`/cascade-demo/img/cascade-logo.png
Exec=`echo $USER_HOME`/cascade-demo/cascade-demo/sitl_gui.py
Path=`echo $USER_HOME`/cascade-demo/cascade-demo/
Terminal=false
Type=Application
" > `echo $USER_HOME`/cascade-demo/cascade-demo.desktop

# give permissions for .desktop file
chown -R $USER: `echo $USER_HOME`/cascade-demo/cascade-demo.desktop

chown -R $USER: `echo $USER_HOME`/PX4-Autopilot

# give read and write access to everyone
chmod 755 `echo $USER_HOME`/cascade-demo/cascade-demo.desktop

ln -s `echo $USER_HOME`/cascade-demo/cascade-demo.desktop $USER_HOME/.local/share/applications/cascade-demo.desktop
ln -s `echo $USER_HOME`/cascade-demo/cascade-demo.desktop $USER_HOME/Desktop/cascade-demo.desktop
ln -s `echo $USER_HOME`/cascade-demo/cascade-demo.desktop /usr/share/applications/cascade-demo.desktop


clear


echo ''
echo ''
echo '#------------------------------------------------------------------#'
echo '# THANKS FOR INSTALLING CASCADE-DEMO #'
echo '#------------------------------------------------------------------#'
echo ''
echo ''

sleep 3

echo_block "Installation Finished, please restart your computer"
