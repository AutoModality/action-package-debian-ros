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

Run locally in a docker container:

```
docker run -v `pwd`/:/github/workspace -w /github/workspace -it ros:kinetic-perception-xenial
```

Then ...

```
# generates version using timestamp
#/github/home/debian-package-test-action_20200105014549_amd64.deb
./entrypoint.sh  

# generates using given version
# /github/home/debian-package-test-action_3.2.8_amd64.deb
./entrypoint.sh 3.2.8 
```

## Contribute 

1. Create the issue explaining the feature or bug
1. Create a branch with the issue id in the branch name
1. Make code changes
1. Add an Action Job to verify your changes (when appropriate)
1. Commit significant notes starting with `fix:` or `feat:`
    1. BREAKING CHANGES: in such cases, but discouraged 
    1. Will be included in release notes
1. Make a pull request
