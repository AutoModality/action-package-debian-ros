#!/bin/bash


file=$1
version=$2

set -e # fail on error

ls -l $file

dpkg -c $file
apt -y install $file

echo verifying $file with version $version is installed

# debian-package-test-action is the name of the action, per the debian/control and debian/changelog.
apt-cache policy debian-package-test-action | grep Installed: | grep $version

# if we made it this far...then it is true
echo ::set-output name=passed::true

