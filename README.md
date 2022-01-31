# CMPE498 Design Project I - Senior 1
>Authors : Abdelrahman Soliman, Mohamad Bahri, Daniel Izham

>Semester : Fall 2021

>Workflow : [Simple Git workflow is simple](https://www.atlassian.com/git/articles/simple-git-workflow-is-simple)

## Quick Start

1. Install *Sphinx 1.8*

   ```bash
   $ echo "deb http://plf.parrot.com/sphinx/binary `lsb_release -cs`/" | sudo tee /etc/apt/sources.list.d/sphinx.list > /dev/null
   $ sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 508B1AE5
   $ sudo apt update
   $ sudo apt install parrot-sphinx
   ```
   
1. Install *Docker*
   
   ```bash
   $ sudo apt-get remove docker docker-engine docker.io containerd runc
   $ curl -fsSL https://get.docker.com -o get-docker.sh
   $ sudo sh get-docker.sh
   ```

1. Install the Boost C++ Libraries development files

   ```bash
   $ sudo apt install -y libboost-all-dev
   ```

1. Unfortunately, the Boost ver. 65 that runs on Ubuntu 18.04 is incompatible with ver. 58 required by *Sphinx*. So let's trick it

   ```bash
   $ cd /usr/lib/x86_64-linux-gnu/
   $ sudo ln -s libboost_system.so.1.65.1 libboost_system.so.1.58.0
   ```
1. Clone this repository
1. `cd` into the cloned repository
1. Initiate *firmwared*

   ```bash
   $ sudo systemctl start firmwared
   ```

1. Make Sphinx know where to find the models and plugins

   ```bash
   $ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/models/ 
   $ export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(pwd)/plugins/moving_target/build/
   ```
1. Correct the `/home/daniel/dp-sim-code` part of the path to the Anafi firmware in the `worlds/custom.world` file. 
   Change it to where this repo on your system is located. The line that needs to be changed is as below
   
   ```xml
   <drone
       ...
       firmware="/home/daniel/dp-sim-code/firmwares/anafi-pc.ext2.zip"
       ...>
   </drone>
   ```
   
1. Edit `worlds/custom.world` and change the basename of the firmware path
to match your system
1. Start up Sphinx

   ```bash
   $ sphinx ./worlds/custom.world
   ```

1. Spin up a container for the development environment

   ```bash
   $ docker run --rm -it -d --network host daniel/sim-dev-env
   ```
   
1. Run the code from one of the jupyter notebooks in the `demos/` folder
