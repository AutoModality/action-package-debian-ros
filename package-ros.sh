#!/bin/bash

#  Downloads debian packages necessary for the build

#see action.yml for inputs
NONE="None" #because blank doesn't work for bash

DEBIAN_DIR=debian
version=${1:-$NONE} # the version of the generated package
build_number=${2:-$NONE}
pull_request_number=${3:-$NONE}
branch=${4:-$NONE}
cloudsmith_read_dev_entitlement=${5:-${CLOUDSMITH_READ_DEV_ENTITLEMENT:-$NONE}}
cloudsmith_read_release_entitlement=${6:-${CLOUDSMITH_READ_RELEASE_ENTITLEMENT:-$NONE}}

# cloudsmith package repository needs to be included for dependency downloads
# it is private and requires access key provided as a parameter
authorize_dev_package_repo(){

    if [[ $cloudsmith_read_dev_entitlement != $NONE ]]; then
        echo "Entitlement provided to access Cloudsmith Dev Repository.  You should see OK messages."

        curl -u "token:$cloudsmith_read_dev_entitlement" -1sLf \
        'https://dl.cloudsmith.io/basic/automodality/dev/cfg/setup/bash.deb.sh' \
        | sudo bash
    else
        echo "No access to Cloudsmith Dev Repository.  Entitlement not provided."
    fi
  # new repository comes new package directory
  apt-get -y update
}

# release repository is available to all 
authorize_release_package_repo(){

    if [[ $cloudsmith_read_release_entitlement != $NONE ]]; then
        echo "Entitlement provided to access Cloudsmith Release Repository.  You should see OK messages."
        curl -u "token:$cloudsmith_read_release_entitlement" -1sLf \
        'https://dl.cloudsmith.io/basic/automodality/release/cfg/setup/bash.deb.sh' \
        | sudo bash
    else
        echo "No access to Cloudsmith Release Repository.  Entitlement not provided."
    fi
  # new repository comes new package directory
  apt-get -y update
}

# ========= MAIN

set -e # fail on error



if [[ ! -d "$DEBIAN_DIR" ]]; then
    echo "$DEBIAN_DIR must exist with rules (executable), "
    exit 1
fi

authorize_dev_package_repo
authorize_release_package_repo


#gets dependencies and packages them for 
mk-build-deps --install --tool='apt-get -o Debug::pkgProblemResolver=yes --no-install-recommends --yes' debian/control

/package.sh "$version" "$build_number" "$pull_request_number" "$branch"