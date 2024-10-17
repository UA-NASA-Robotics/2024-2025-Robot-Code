#!/bin/bash

# Run RS Package with turtlesim for testing.
if [ "$1" == "--rs-package" && "$2" == "--with-turtlesim" ]; then
    ./RS_Package/run.sh --with-turtlesim
fi

# Run RS Package without turtlesim.
if [ "$1" == "--rs-package" ]; then
    ./RS_Package/run.sh
fi