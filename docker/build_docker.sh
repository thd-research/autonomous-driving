#!/usr/bin/env bash

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
EXEC_PATH=$PWD

cd $ROOT_DIR

echo "[!] If you use nvidia gpu, please rebuild with -n or --nvidia argument"
docker build -t autonomous-driving-img -f $ROOT_DIR/docker/Dockerfile $ROOT_DIR \
                                       --network=host \
                                       --build-arg from=nvidia/cuda:11.4.3-cudnn8-runtime-ubuntu20.04

cd $EXEC_PATH
