FROM subt_hello_world_common

# import the solution source code that will be used for submission
WORKDIR ${SOLUTION_WS_PATH}/src
COPY packages/ .

# build the solution
WORKDIR ${SOLUTION_WS_PATH}
RUN [ "/bin/bash" , "-c" , \
  "source ${PATH_TO_DEPS_WS_SETUP_BASH} \
  && sudo apt-get -qq update \
  && rosdep install --from-paths src --ignore-src -r -y --skip-keys=ignition-common3 --skip-keys=ignition-msgs4 \
  && colcon build \
  && sudo rm -rf src build log \
  && sudo rm -rf /var/lib/apt/lists/* \
  && sudo apt-get -qq clean" ]

# run the solution launch file when the container is started
WORKDIR /home/$USERNAME
COPY ./docker/hello_world_entry/run_solution.bash .
ENTRYPOINT [ "/bin/bash" , "-c" , \
  "./run_solution.bash" ]