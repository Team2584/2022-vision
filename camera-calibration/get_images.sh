#!/bin/bash

NUM_RE='^[0-9]+$'

if ! [[ $@ =~ $NUM_RE ]]
then
    echo "Pass a single integer number of images to capture as an argument."; exit 2
fi

for (( i=1; i<=$1; i++))
do
    echo "Capturing image $i in..."
    echo "3"
    sleep 1
    echo "2"
    sleep 1
    echo "1"
    fswebcam -r 1920x1080 -D 1 --no-banner -q $(printf "%02d.jpg" "$i") && printf "Image captured\n\n"
    sleep 1
done
