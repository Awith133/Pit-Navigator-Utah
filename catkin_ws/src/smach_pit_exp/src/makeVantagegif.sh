#!/usr/bin/env bash
echo $1
echo $2
cd $1
convert -delay 200 -loop 0 *.png $2.gif
