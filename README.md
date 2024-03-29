# CMPE498 Design Project I - Senior 1
>Authors : Abdelrahman Soliman, Mohamad Bahri, Daniel Izham

>Semester : Fall 2021

>Workflow : [Simple Git workflow is simple](https://www.atlassian.com/git/articles/simple-git-workflow-is-simple)

## Quick Start

1. Make sure Sphinx and Olympe are working on your system
1. Clone this repository
1. `cd` into the cloned repository
1. Make a virtual environment using your method of choice

   It is up to you whether to make a virtual environment or use your global python setup.
   If you prefer the latter, it is easier because you can just source the Olympe shell script 
   and you are done, but you will pollute your system with the dependencies.
   
   On the other hand, if you do the former, then you won't have the said problem but you have
   to follow these [instructions](https://forum.developer.parrot.com/t/use-olympe-from-a-ros-program/10009/4)
   to be able to import the Olympe libraries in a virtual environment.
   
1. Install dependencies from the `requirements.txt` OR use *docker*
1. Initiate *firmwared*

   ```bash
   $ sudo systemctl start firmwared
   ```

1. Make Sphinx know where to find the models and plugins

   ```bash
   $ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:./models/ 
   $ export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./plugins/moving_target/build/
   ```
1. Start up Sphinx

   ```bash
   $ sphinx ./worlds/empty.world
   ```

1. Run the code from one of the jupyter notebooks in the `demos/` folder
