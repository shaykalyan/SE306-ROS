#!/bin/bash

make test 

echo '================================================================='
echo 'Build complete, beginning tests.'
echo '================================================================='


cd `rospack find elderly_care_simulation`/bin/test

for f in *
do
    ./$f
done
