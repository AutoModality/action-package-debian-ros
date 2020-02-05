# Debian Packaging on a ROS Container
ROS project packaging in Debian format ready for installation.

---

[
![Action Badge](https://github.com/AutoModality/action-package-debian-ros/workflows/Validate%20Packaging/badge.svg)](https://github.com/AutoModality/action-package-debian-ros/actions)

---

## Features
* Generates a debian package available for subsequent actions to access (presumably install or deploy)
* Generates changelog based on the version provided
* Version can be provided directly
* Version can be generated automatically using timestamp
* Version can be built from a combination of pull-request-number, build-number, branch
* Can download dependencies from the AM private package repository

## Usage

```
name: Package Example
on: push
jobs:
  package:
    runs-on: ubuntu-16.04
    name: Example of creating a debian package
    steps:
      - uses: actions/checkout@v1
      - name: Package
        id: package
        uses: AutoModality/action-package-debian-ros@v1
        with:
          version: 1.3.0
          release-repo-entitlement: ${{ secrets.CLOUDSMITH_READ_RELEASE_ENTITLEMENT }}
      - name: The generated package
        run: echo "The artifact is ${{ steps.package.outputs.artifact-path }}"
```

See [action.yml](action.yml) for more options. 

## Releases

Uses Semantic Release to publish [releases](https://github.com/AutoModality/action-package-debian-ros/releases).


## Tests & Specs

Tests are run by the [Actions](https://github.com/AutoModality/action-package-debian-ros/actions) and demonstrate the features provided. 

## Development

You will need to build packages in your local development environment.

**Build the Docker Image** so you can run a container locally.

```
docker build -t amros-build .
```

**Provide access to package repository** by exporting Cloudsmith entitlements to environment variables.

* [Release repository entitlements](https://cloudsmith.io/elevate/?next=/~automodality/repos/release/entitlements/)
* [Dev repository entitlements](https://cloudsmith.io/elevate/?next=/~automodality/repos/dev/entitlements/)

```
export CLOUDSMITH_READ_DEV_ENTITLEMENT={your entitlement}
export CLOUDSMITH_READ_RELEASE_ENTITLEMENT={your entitlement}
```

or using the [cloudsmith cli](https://github.com/cloudsmith-io/cloudsmith-cli)

```
cloudsmith entitlements list automodality/dev --show-tokens
cloudsmith entitlements list automodality/release --show-tokens
```
**Run the Docker Container** in the root of the project you wish to build (e.g. ~/am/github/visbox).


```
docker run --entrypoint=/bin/bash -v `pwd`/:/github/workspace -w /github/workspace --env CLOUDSMITH_READ_DEV_ENTITLEMENT=$CLOUDSMITH_READ_DEV_ENTITLEMENT --env CLOUDSMITH_READ_RELEASE_ENTITLEMENT=$CLOUDSMITH_READ_RELEASE_ENTITLEMENT -it amros-build
```

**Run the Build Script** in the container.

```
/entrypoint.sh None None None None $CLOUDSMITH_READ_DEV_ENTITLEMENT $CLOUDSMITH_READ_RELEASE_ENTITLEMENT
```


## Parameters

From [action.yml](action.yml).

```
  version:  
    description: 'The version of the package to be generated. Other inputs ignored if provided.'
    required: false
    default: 'None'
  branch: 
    description: 'Optional: The branch name used in the minor version. Paths will be escaped.'
    required: false
    default: 'None'
  build-number: 
    description: 'Optional: Corresponds to the build number that invoked this. Patch version. '
    required: false
    default: 'None'
  pull-request-number: 
    description: 'Optional: Corresponds to the pull request that invoked this. Major version.'
    required: false
    default: 'None'
  dev-repo-entitlement: 
    description: 'Optional. If provided, will have access to the Cloudsmith dev repository for dependency download.'
    required: false
    default: 'None'
  release-repo-entitlement: 
    description: 'Optional. If provided, will have access to the Cloudsmith release repository for dependency download.'
    required: false
    default: 'None'
outputs:
  artifact-path:
    description: 'The file path where the artifact can be found'
```