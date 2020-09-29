#!/bin/bash

echo "Building with only releases"

/package-ros.sh None None None None None $CLOUDSMITH_READ_RELEASE_ENTITLEMENT
