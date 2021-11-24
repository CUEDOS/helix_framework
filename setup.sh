#!/usr/bin/env bash

setterm -background red

if [[ "$(id -u)" != "0" ]]; then
   echo "THIS SCRIPT MUST BE RUN AS ROOT (sudo ./setup.sh)"
   sleep 3
   clear      
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
        curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
        ${PYTHON} get-pip.py
        rm get-pip.py
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

#PYTHON="python3"

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

cd cascade-demo

git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh


set -eou pipefail
IFS=$'\n\t'
setterm --reset

clear


# Delete old installations
rm -f $HOME/cascade-demo/cascade-demo.desktop
rm -f $HOME/.local/share/applications/cascade-demo.desktop
rm -f $HOME/Desktop/cascade-demo.desktop
rm -f /usr/share/applications/cascade-demo.desktop


# Create .desktop file
echo "
[Desktop Entry]
Version=1.0
Name=Cascade Demo
Icon=`echo $HOME`/cascade-demo/img/cascade-logo.png
Exec=`echo $HOME`/cascade-demo/cascade-demo/sitl_gui.py
Path=`echo $HOME`/cascade-demo/cascade-demo/
Terminal=false
Type=Application
" > `echo $HOME`/cascade-demo/cascade-demo.desktop

# give permissions for .desktop file
chown $USER:$USER -R `echo $HOME`/cascade-demo/cascade-demo.desktop

# give read and write access to everyone
chmod 755 `echo $HOME`/cascade-demo/cascade-demo.desktop

ln -s `echo $HOME`/cascade-demo/cascade-demo.desktop $HOME/.local/share/applications/cascade-demo.desktop
ln -s `echo $HOME`/cascade-demo/cascade-demo.desktop $HOME/Desktop/cascade-demo.desktop
ln -s `echo $HOME`/cascade-demo/cascade-demo.desktop /usr/share/applications/cascade-demo.desktop

update-menus

clear


echo ''
echo ''
echo '#------------------------------------------------------------------#'
echo '# THANKS FOR INSTALLING CASCADE-DEMO #'
echo '#------------------------------------------------------------------#'
echo ''
echo ''

sleep 3

echo_block "Installation Finished"