#!/bin/bash

package_directory=`rospack find elderly_care_simulation`
if [ $? -ne 0 ]; then
    exit $?
fi

printf '================================================================================\n'
echo 'Running gtests.'
echo '================================================================================'


cd $package_directory/bin/test

for gtest_file in *
do
    printf '\n\n\n'
    echo $gtest_file
    echo '================='
    ./$gtest_file
done


printf '\n\n\n\n================================================================================\n'
echo 'Running rostests.'
echo '================================================================================'

cd $package_directory/test

for rostest_file in *.test
do
    printf '\n\n\n'
    echo $rostest_file
    echo '================='
    rostest $rostest_file
done