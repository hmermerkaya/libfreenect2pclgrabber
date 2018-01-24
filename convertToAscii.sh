#!/bin/bash
filename='fileList'
filelines=`cat $filename`
echo Start
for line in $filelines ; do
    echo $line
done
