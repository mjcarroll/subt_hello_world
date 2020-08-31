#!/usr/bin/env bash

# we want the context of the build to be where the source code is so that we
# can copy over the solution packages into the entry image
docker build -t subt_entry -f ./hello_world_entry/Dockerfile ../