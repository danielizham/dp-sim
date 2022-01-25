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

1. Clone this repository
1. `cd` into the cloned repository
1. Initiate *firmwared*

   ```bash
   $ sudo systemctl start firmwared
   ```

1. Make Sphinx know where to find the models and plugins

   ```bash
   $ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:./models/ 
   $ export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./plugins/moving_target/build/
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
