#!/bin/bash

export LD_LIBRARY_PATH=$(readlink -f .)/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$(readlink -f .):$LD_LIBRARY_PATH
