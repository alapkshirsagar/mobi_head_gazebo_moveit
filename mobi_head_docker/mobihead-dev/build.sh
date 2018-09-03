#!/bin/bash

docker build -t mobihead-dev --build-arg CUSTOM_UID=$UID  .
