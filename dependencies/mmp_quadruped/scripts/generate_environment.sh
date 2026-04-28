#!/usr/bin/env bash

planning_env="ramp"

# name of the environment file (without extension)
file_name="down_ramp_10"

# friction coefficient for the environment
friction_coefficient=1.0

python3 make_environment_yaml.py -e ${planning_env} -f ${file_name}

python3 convert_yaml_to_world.py -e ${planning_env} -f ${file_name} -c ${friction_coefficient}