You need to install docker to run this container. To install docker, execute

    curl -fsSL https://get.docker.com/ | sh

If you want to observe the installation process of the container, execute 

    docker build -t nrp_prototype $HBP/Models/mouse_v2_model/docker

If you receive errors like "Could not resolve <repository name>" during installation, 
add DOCKER_OPTS="--dns <your dns server>" to /etc/default/docker

For further information read https://bbpteam.epfl.ch/project/spaces/display/HSP10/Running+containers+with+docker
