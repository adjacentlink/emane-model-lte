#!/bin/bash -

# usage ./emane-spectrum-analyzer.sh 10.88.1.1

if [ "$#" != '1' ]
then
    echo 'usage: emane-spectrum-analyzer.sh <hostname>'
    exit 1
fi

emane-spectrum-analyzer \
    --hz-min 2500000000 \
    --hz-max 2700000000 \
    --subid-name 12,LTE \
    $1:8883 \
    -110
