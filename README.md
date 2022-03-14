# CMPE498 Design Project I - Senior 1
>Authors : Abdelrahman Soliman, Mohamad Bahri, Daniel Izham

>Semester : Fall 2021

>Workflow : [Simple Git workflow is simple](https://www.atlassian.com/git/articles/simple-git-workflow-is-simple)

## Quick Start

1. Ensure your computer's graphics mode is set to Discrete to avoid memory leak while running Sphinx.

1. Install *Sphinx 1.8*

   ```bash
   echo "deb http://plf.parrot.com/sphinx/binary `lsb_release -cs`/" | sudo tee /etc/apt/sources.list.d/sphinx.list > /dev/null && \
   sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 508B1AE5                                                       && \   
   sudo apt update                                                                                                              && \ 
   sudo apt install parrot-sphinx
   ```
   
1. Install *Docker*
   
   ```bash
   sudo apt-get remove docker docker-engine docker.io containerd runc
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   ```
1. Make a Python virtual environment using your method of choice. My recommendation is to use a combination of Pyenv and Poetry.
   
   To do so, first install the required Ubuntu packages
   ```bash
   sudo apt-get update && sudo apt-get install -y make build-essential libssl-dev zlib1g-dev \
   libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm \
   libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev \
   libffi-dev liblzma-dev
   ```
   
   Then, set up Pyenv
   ```bash
   curl https://pyenv.run | bash && \
   echo >> ~/.bashrc && \
   echo "# to use pyenv and pyenv-virtualenv" >> ~/.bashrc && \
   echo 'export PATH="$HOME/.pyenv/bin:$PATH"' >> ~/.bashrc && \
   echo 'eval "$(pyenv init --path)"' >> ~/.bashrc && \
   echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc && \
   echo 'source $(pyenv root)/completions/pyenv.bash' >> ~/.bashrc
   ```
   
   and Poetry
   ```bash
   sudo apt-get install -y python3-venv && \
   curl -sSL https://install.python-poetry.org | python3 - && \
   echo >> ~/.bashrc && \
   echo "# to use poetry" >> ~/.bashrc && \
   echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc && \
   poetry self update && \
   poetry completions bash | sudo tee /etc/bash_completion.d/poetry.bash-completion
   ```
   
   Restart your terminal for the changes to take effect.
   
   If you face any issue while following the instructions above, please refer to their respective official documentations
   for [Pyenv](https://github.com/pyenv/pyenv) and [Poetry](https://python-poetry.org/docs/)
   
1. Install the right version of Python (Note: this will take some time)

   ```bash
   pyenv install 3.8.12
   ```
1. Install the zbar source and header files required by the Pyzbar package

   ```bash
   sudo apt-get install zbar-tools
   ```
1. Install the Boost C++ Libraries development files

   ```bash
   sudo apt install -y libboost-all-dev
   ```

1. Unfortunately, the Boost ver. 65 that runs on Ubuntu 18.04 is incompatible with ver. 58 required by *Sphinx*. So let's trick it

   ```bash
   cd /usr/lib/x86_64-linux-gnu/ && \
   sudo ln -s libboost_system.so.1.65.1 libboost_system.so.1.58.0
   ```
1. Initiate *firmwared*

   ```bash
   sudo systemctl start firmwared
   ```
1. Clone this repository
1. `cd` into the cloned repository
1. Set a virtual environment for this directory (change \<project-name\> to whatever you want to identify this environment with)
  
   ```bash
   pyenv virtualenv 3.8.12 <project-name> && pyenv local <project-name>
   ```
1. Install the Python dependencies based on the pyproject.toml (Note: this will take some time)

   ```bash
   poetry install
   ```
1. Make Sphinx know where to find the models and plugins

   ```bash
   export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${PWD}/models
   export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${PWD}/worlds
   export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${PWD}/plugins/moving_target/build
   ```
1. Edit `worlds/custom.world` file and change the basename (`/home/daniel/dp-sim`) of the Anafi firmware path
to match your directory structure. The line that needs to be changed is as follows
   
   ```xml
   <drone
       ...
       firmware="/home/daniel/dp-sim/firmwares/anafi-pc.ext2.zip"
       ...>
   </drone>
   ```
   
   Instead of doing it manually, the correction can be done in one simple line:

   ```bash
   sed -i "s|/home/daniel/dp-sim|${PWD}|g" ./worlds/custom.world
   ```
   
1. Start up Sphinx

   ```bash
   sphinx custom.world
   ```

1. Spin up a container for the development environment

   ```bash
   docker run --rm -it -d --network host daniel/sim-dev-env
   ```
   
1. Run the code from one of the jupyter notebooks in the `demos/` folder
