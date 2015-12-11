#!/bin/bash

rel_video_path=$1
abs_video_path=$HOME$rel_video_path

vlc --fullscreen --quiet $abs_video_path &

duration=$(mediainfo --Inform="General;%Duration%" $abs_video_path)
seconds=$((duration / 1000))
rounded=$(printf %.0f $seconds)

sleep $rounded

kill $(pidof vlc)

exit 0
