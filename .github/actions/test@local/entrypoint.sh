#!/bin/bash


file=$1
version=$2

set -e # fail on error

ls -l $file

dpkg -c $file
apt -y install $file

echo verifying $file with version $version is installed

apt-cache policy apt-utils | grep Installed: | grep $version

# if we made it this far...then it is true
echo ::set-output name=passed::true

