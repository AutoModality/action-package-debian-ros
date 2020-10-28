#!/bin/bash

# Creates a debian package using catkin build
# This script works in a local docker container or with it's Github action.yml
# Script expects an environment prepared by either package-ros.sh or package-amros.sh container
# the version generated is only for development builds currently

#see action.yml for inputs
NONE="None" #because blank doesn't work for bash

DEBIAN_DIR=debian
version=${1:-$NONE} # the version of the generated package
build_number=${2:-$NONE}
pull_request_number=${3:-$NONE}
branch=${4:-$NONE}


# replaces / with - so feature/BB-182 = feature-BB-182 for version compatibility
append_branch_version(){
    if [[ $branch != $NONE ]]; then
        echo ".$(echo "$branch" | tr /_ -)"
    fi
}
#provides a timestamp as unique number to ensure version will be unique
append_timestamp(){
    echo ".$(date +%Y%m%d%H%M%S )"
}

# uses or makes version based on caascading set of rules based on what is provided
# if
version_guaranteed(){
    
    if [[ "$version" != "$NONE" ]]; then
        # use version as provided
        v="$version"
    elif [[ $pull_request_number != $NONE ]]; then
        # pr as the major is good for consistency and identity
        # possibilities:
        # {pr#}
        # {pr#}.{branch}
        # {pr#}.{build#}
        # {pr#}.{branch}.{build#}
        v="$pull_request_number$(append_branch_version)"
        if [[ "$build_number" != "$NONE" ]]; then
            v="$v.$build_number"
        fi
        v="$v$(append_timestamp)"
    elif [[ "$build_number" != "$NONE" ]]; then
        # build number is fine as the major and is good for identify (once github provides it)
        # branch provides consistency across builds
        # timestamp provides uniquenenss allowing repeat deployment
        # {build#}.{timestamp}
        # {build#}.{branch}.{timestamp}
        v="$build_number$(append_branch_version)$(append_timestamp)"
    else
        # nothing provided.  use date
        # {timestamp}
        # {timestamp}.{branch}
        v="0$(append_timestamp)$(append_branch_version)"
    fi
    
    echo $v
}

log(){
    message=$1
    echo
    echo "============================================="
    date --iso-8601=s
    echo $message
    echo "============================================="
    echo
}

# ========= MAIN

set -e # fail on error

# the directory where the artifact should be copied so other actions can access
# see also https://medium.com/@fonseka.live/sharing-data-in-github-actions-a9841a9a6f42
staging_dir="/github/home"
workspace="/github/workspace"

# the git root is always mapped to the docker's /root
mkdir -p $workspace
mkdir -p $staging_dir
cd $workspace


if [[ ! -d "$DEBIAN_DIR" ]]; then
    echo "$DEBIAN_DIR does not exist ... will attempt to generate "
fi

echo amros | sudo -S echo authenticated as root

log "installing dependencies from control file"


version=$(version_guaranteed)
sudo amros dev build deb --clean --version "$version" #performs the package

gen_dir="."
artifact_filename=$(ls $gen_dir | grep .deb | tail -1) #the package is generated in base directory

# build debian artifacts may be generated and misreport success
if [[ $artifact_filename == *"build-deps"* ]]; then
  echo "Failed to generate package. Only $artifact_filename was found."
  exit -2
fi

artifact_gen_path="$gen_dir/$artifact_filename"

log "staging package for sharing"

# share with other actions in github
artifact_share_path="$staging_dir/$artifact_filename"

if [[ -f "$artifact_gen_path" ]];then   
    cp "$artifact_gen_path" "$artifact_share_path"
else
    echo "Failed to generate debian binary"
    exit -1
fi

#show the details of the file FYI and to validate existence
echo package file info -----------------------
ls -lh "$artifact_share_path"

echo package info  -----------------------
dpkg --info "$artifact_share_path"

echo package contents -----------------------
dpkg --contents "$artifact_share_path"


echo ::set-output name=artifact-path::"$artifact_share_path"  #reference available to other actions
echo ::set-output name=version::$version  #reference available to other actions
