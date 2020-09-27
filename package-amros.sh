#!/bin/bash

#  Downloads debian packages necessary for the build

#see action.yml for inputs
NONE="None" #because blank doesn't work for bash


version=${1:-$NONE} # the version of the generated package
build_number=${2:-$NONE}
pull_request_number=${3:-$NONE}
branch=${4:-$NONE}

# authenticate as sudo to allow installation without prompt
# requires password = username
whoami | sudo -S whoami
amros install
package.sh "$version" "$build_number" "$pull_request_number" "$branch"