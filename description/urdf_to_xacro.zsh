#!/usr/bin/zsh

xacro rover.xacro > rover.urdf

check_urdf rover.urdf

urdf_to_graphviz rover.urdf rover

echo "DONE!"
