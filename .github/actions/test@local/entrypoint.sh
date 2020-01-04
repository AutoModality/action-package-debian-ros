#!/bin/bash


file=$1

set -e # fail on error

ls - l $file

apt -y install $file

# see control file for package name
ls -l /usr/share/doc/debian-package-test-action/changelog.gz

# if we made it this far...then it is true
echo ::set-output name=passed::true

