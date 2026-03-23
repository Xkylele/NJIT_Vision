#!/bin/bash

cd ~/Desktop/NJIT_Vision/

while true
do
    echo "Restarting program..."

    ./build/sentry

    echo "Program exited, restarting..."

    sleep 2
done