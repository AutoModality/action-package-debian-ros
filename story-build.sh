#!/bin/bash

echo "Building with release candidates and releases"

/package-ros.sh None None None None $CLOUDSMITH_READ_DEV_ENTITLEMENT $CLOUDSMITH_READ_RELEASE_ENTITLEMENT
