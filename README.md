# CMPE498 Design Project I - Senior 1
>Authors : Abdelrahman Soliman, Mohamad Bahri, Daniel Izham

>Semester : Fall 2021

>Workflow : [Simple Git workflow is simple](https://www.atlassian.com/git/articles/simple-git-workflow-is-simple)

## Quick Start

1. Make sure Sphinx and Olympe are working on your system
1. Clone this repository
1. `cd` into the cloned repository
1. Make a virtual environment using your method of choice
1. Install dependencies from the `requirements.txt`
1. Initiate `firmwared` and start up Sphinx by running the following commands:

   ```bash
   $ GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:./models/ LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./plugins/moving_target/build/ sphinx ./worlds/empty.world
   ```

1. Run the code from one of the jupyter notebooks in the `demos/` folder
