#!/bin/bash

echo "Building with release candidates and releases"

/entrypoint.sh None None None None $CLOUDSMITH_READ_DEV_ENTITLEMENT $CLOUDSMITH_READ_RELEASE_ENTITLEMENT
