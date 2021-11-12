#!/bin/sh
# Replace the path below with the path to the mavsdk bin directory on your system
#cd ~/anaconda3/lib/python3.7/site-packages/mavsdk/bin
# Check if arguments are supplied
if [ -z "$3" ]
  then
    echo "Not enough arguments, using default values"
    # Default values
    no_of_drones=1
    port=50041
    udp_port=14540
    else
    no_of_drones=$1
    port=$2
    udp_port=$3
    id=101
fi

# Launch the mavsdk servers and launch flocking scripts, incrementing the port and UDP port by 1 each time
for (( i=1; i <= $no_of_drones; ++i ))
do
    gnome-terminal --tab --title="Mavsdk Server" -- bash -c "cd ~/anaconda3/lib/python3.7/site-packages/mavsdk/bin && ./mavsdk_server -p $port udp://:$udp_port; exec bash"
    gnome-terminal --tab --title="" -- bash -c "cd ~/cascade-demo && python3 simple_flocking.py $id $no_of_drones $port; exec bash"
    ((id++))
    ((port++))
    ((udp_port++))
done

