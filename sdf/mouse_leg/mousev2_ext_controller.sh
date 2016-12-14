#!/bin/bash

start() {
    echo "Loading external controller"
    if [ "$(docker ps -a | grep -o nrp_prototype_container)" = "" ]; then #check if container exists
        if [ "$(docker images -q nrp_prototype)" = "" ]; then   #check if image exists
            docker build -t nrp_prototype docker/      #build image from docker subdirectory
        fi
        dns_server=$(nm-tool | grep -m 1 DNS | grep -o '[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*$') #get the dns server listed first by nm-tool
        docker run -tid --name nrp_prototype_container --dns=$dns_server nrp_prototype #create container from image
        sleep 1
    fi
    docker start nrp_prototype_container #start container
    docker exec nrp_prototype_container sed -i "s/export ROS_MASTER_URI.*/export ROS_MASTER_URI=http:\/\/$(hostname -I | cut -d " " -f 1):11311\//g" /external_controller.sh   #set ROS_MASTER_URI to host machine
    docker exec -d nrp_prototype_container bash /external_controller.sh #run container in detached mode
}

stop() {
    rosnode kill /ext_controller    #stop the ROS node
    docker stop nrp_prototype_container    #stop the container (sends SIGTERM and SIGKILL to container process)
}

mode=$1

case $mode in
'start')
    start
    exit 0
    ;;
'stop')
    stop
    exit 0
    ;;
*)
    echo "Unknown mode $mode."
    exit 1
    ;;
esac
