#!/usr/bin/env bash

ffmpeg -re -framerate 10 -video_size 1280x720 -i /dev/video10 -c:v libx264 -f tee -preset ultrafast -tune zerolatency -b 1500k -map 0:v "output1.ts|[f=mpegts]udp://230.0.0.1:8001?ttl=18"
