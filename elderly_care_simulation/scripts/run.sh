#!/bin/bash

dependencies=( "python" "python-tk" "idle" "python-pmw" "python-imaging" )
not_installed=()

for dependency in ${dependencies[@]}
do
    PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $dependency|grep "install ok installed")
    if [ "" == "$PKG_OK" ]; then
      echo "Missing dependency: $dependency"
      not_installed+=($dependency)
    fi
done

if [ ${#not_installed[@]} -ne 0 ]; then
    while true; do
        read -p "You are missing some dependencies. Would you like to install them? (y/n) " yn
        case $yn in
            [Yy]* ) sudo apt-get --force-yes --yes install python python-tk idle python-pmw python-imaging; break;;
            [Nn]* ) exit;;
            * ) echo "Please answer either 'y' or 'n'";;
        esac
    done
fi

roslaunch `rospack find elderly_care_simulation`/launch/default.launch
