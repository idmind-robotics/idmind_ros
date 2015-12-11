#!/bin/bash

rel_img_path=$1
abs_img_path=$HOME$rel_img_path

eog --fullscreen $abs_img_path &

exit 0
