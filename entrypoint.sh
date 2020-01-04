#!/bin/bash

# Makes and Creates a debian package
# This script works in a local docker container or with it's Github action.yml
# ./build directory is cleaned every time
# the version generated is only for development builds currently

#see action.yml for inputs
NONE="None"
DEBIAN_DIR=debian
version=${1:-$NONE} # the version of the generated package
build_number=${2:-$NONE} 
pull_request_number=${3:-$NONE} 
branch=${4:-$NONE} 



# extract the package name from the control file
package_name_from_control(){
    #get the project name from the control file
    package_line=$(cat debian/control | grep Package)
    echo "$package_line" | awk -F': ' '{print $2}'
}

# uses or makes version based on caascading set of rules based on what is provided
version_guaranteed(){
    
    if [[ -n $version ]]; then
        # use version as provided
        v = $version
    elif [[ -n $pull_request_number ]]; then
        # pr as the major is good for consistency and identity 
        # possibilities:
        # {pr#}
        # {pr#}.{branch}
        # {pr#}.{build#}
        # {pr#}.{branch}.{build#}
        v = $pull_request_number
        if [[ -n $branch ]]; then
            branch_escaped=$($branch| awk -F'/' '{print $1-$2}')
            v = "$v.$branch"
        fi
        if [[ -n $branch ]]; then
            branch_escaped=$($branch| awk -F'/' '{print $1-$2}')
            v = "$v.$branch"
        fi
        if [[ -n $build_number ]]; then
            v = "$v.$build_number"
        fi
    elif [[ -n $build_number ]]; then
        # build number is fine as the major and is good for identify (once github provides it)
        # branch provides consistency across builds
        v = $build_number
        if [[ -n $branch ]]; then
            branch_escaped=$($branch| awk -F'/' '{print $1-$2}')
            v = "$v.$branch"
        fi
    else
        # nothing provided.  use date
        v=$(date +%Y%d%m%H%M%S )
    fi

    echo $v
}
# ========= MAIN

set -e # fail on error

# the git root is always mapped to the docker's /root 
cd /github/workspace

source /opt/ros/kinetic/setup.bash



# the directory where the artifact should be copied so other actions can access
# see also https://medium.com/@fonseka.live/sharing-data-in-github-actions-a9841a9a6f42
staging_dir="/github/home"

if [[ ! -d "$DEBIAN_DIR" ]]; then
    echo "$DEBIAN_DIR must exist with rules (executable), "
    exit 1
fi

rm -rf build
rm -f $DEBIAN_DIR/changelog

# clean the debian and build directories and will validate necessary files
fakeroot debian/rules clean #ensures no residue

#TODO: get release notes from github and add them to the changelog
control_version_line="$(package_name_from_control) ($(version_guaranteed)) unstable; urgency=medium"
echo $control_version_line
exit 1
echo $control_version_line > $DEBIAN_DIR/changelog


fakeroot debian/rules binary #performs the package

artifact_filename=$(ls .. | grep $package_name) #the package is generated in base directory
artifact_path="$staging_dir/$artifact_filename"
mv "../$artifact_filename" $staging_dir

#show the details of the file FYI and to validate existence
ls -lh $artifact_path

echo ::set-output name=artifact-path::$artifact_path  #reference available to other actions

