kbd-cart-cmd
============

kbd-cart-cmd: Keyboad Cartesian Commander. It takes key strokes and sends new end-effector commands to the vfclik (vector field closed loop inverse kinematics) system. It uses yarp to send this commands.

CAUTION: This program is specific to the US keyboard layout!

compile it with

> make

or

mkdir build
cd build
cmake ../
make

make a debian package with

make package

run it and connect it to a running arm system with

kbd-cart-cmd

and connect to the system with:

connect.sh [robot] [left|right]
