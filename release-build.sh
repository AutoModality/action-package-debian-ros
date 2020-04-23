#!/bin/bash

echo "Building with only releases"

/entrypoint.sh None None None None None $CLOUDSMITH_READ_RELEASE_ENTITLEMENT
