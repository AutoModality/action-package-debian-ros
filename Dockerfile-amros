FROM docker.cloudsmith.io/automodality/release/amros-melodic:latest

# Copies your code file from your action repository to the filesystem path `/` of the container
COPY package-amros.sh /package-amros.sh
COPY package.sh /package.sh
COPY release-build.sh /release-build.sh
COPY story-build.sh /story-build.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/package-amros.sh"]