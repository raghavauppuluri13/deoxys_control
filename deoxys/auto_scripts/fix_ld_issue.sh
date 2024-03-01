#!/bin/bash

export LD_LIBRARY_PATH=$(readlink -f .)/protobuf/install/lib:$LD_LIBRARY_PATH
export LIBRARY_PATH=$(readlink -f .)/protobuf/install/lib:$LIBRARY_PATH
