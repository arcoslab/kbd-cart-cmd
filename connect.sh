#!/bin/sh
base="kbd-cart"
if [ $3 ]; then
    base=$3
fi

if [ "$2" != "left" -a "$2" != "right" ]; then
  echo "syntax: connect.sh [robot] [left|right] [portbasename]"
  exit
fi

yarp connect /$base/out /$1/$2/ofeeder/object
yarp connect /$1/$2/vectorField/pose /$base/in
