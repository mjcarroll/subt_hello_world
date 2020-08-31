# Docker workflow for SubT

There are two docker images: one for development and one for entry (submitting to the competition for evaluation/scoring). Each image is based off of a "common image" that has all of the solution source code dependencies set up.

## Requirements

The `common` image (used as the base image for both the `dev` and `entry` images) uses CUDA 10.1, which only supports a certain set of NVIDIA drivers.
Take a look at the valid driver list [here](https://github.com/NVIDIA/nvidia-docker/wiki/CUDA) to ensure that you have a valid NVIDIA driver installed on your machine.
If your NVIDIA driver version does not support CUDA 10.1, you can either update your driver version to one that is compatible with CUDA 10.1, or try to modify the [common Dockerfile](./hello_world_common/Dockerfile) to use a different version of CUDA.

## Usage

1. Build the common image:
```
./build_common_image.bash
```

2. Build the development image:
```
./build_dev_image.bash
```

3. Start a container from the development image, loading the solution source code into the container as a volume (if you don't have the solution source code on your machine yet, make sure to `git clone` the repository first):
```
# if you have the solution source code stored in "~/subt_solution", then "<PATH/TO/SUBT/SOLUTION/SOURCE/CODE>" may end up being "~/subt_solution/packages"
./run_dev_container.bash <PATH/TO/SUBT/SOLUTION/SOURCE/CODE>
```

4. Once in the development container, you can build your solution workspace:
```
source ~/setup_solution_ws.bash
```

5. After building the solution workspace, you can run the following command in different terminals to start other shells in that container that have the workspaces built and sourced:
```
./join.bash
```

6. Once development is done, you can build the "entry image" to create a solution submission for one robot (keep in mind that you may need to modify the launch file used for the entry point in [hello_world_entry/run_solution.bash](./hello_world_entry/run_solution.bash) first):
```
./build_entry_image.bash
```

7. You can then start a container from the entry image for competition scoring (if you want to submit the entry image for actual competition evaluation, follow the [cloudsim submission](https://github.com/osrf/subt/wiki/Cloudsim%20Submission) steps):
```
./run_entry_container.bash
```

### Running SubT Simulations
The `dev` and `entry` docker containers are not meant for running SubT simulation environments (these containers do not have the simulation models downloaded). The [osrf/subt-virtual-testbed](https://hub.docker.com/r/osrf/subt-virtual-testbed) image is meant for this. You can use this image to run a container that will start a simulation environment:
```
cd simulation_runner/
./run.bash osrf/subt-virtual-testbed:latest cave_circuit.ign worldName:=cave_qual robotName1:=X1 robotConfig1:=X2_SENSOR_CONFIG_6
```

Now, in another terminal, start a container from the dev image to run the solution (if you are used a UAV instead of the UGV listed in the simulation command above, be sure to replace `robot.launch` with `uav_robot.launch` in the commands below):
```
# start the dev container
./run_dev_container.bash <PATH/TO/SUBT/SOLUTION/SOURCE/CODE>

# build the solution workspace (run this in the docker container shell)
source ~/setup_solution_ws.bash

# start the solution (run this in the docker container shell)
roslaunch subt_solution_launch robot.launch name:=X1
```

*If you'd like to do other things in the dev container (like start RViz or inspect ROS topics and nodes), don't forget that you can get access to another shell in the dev container by running `./join.bash` in a new terminal.*