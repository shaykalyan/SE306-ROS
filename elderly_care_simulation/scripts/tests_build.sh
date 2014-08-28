#!/bin/bash

package_directory=`rospack find elderly_care_simulation`
if [ $? -ne 0 ]; then
    exit $?
fi

mkdir -p $package_directory/bin/test
make test 