#!/bin/bash

rel_sound_path=$1
abs_sound_path=$HOME$rel_sound_path

aplay -q $abs_sound_path &

exit 0
