FROM ros:melodic-perception

ENV DEBIAN_FRONTEND noninteractive

ARG CACHE_BUSTER="v3.2.5"
# Bring up to date https://automodality.atlassian.net/wiki/spaces/AUTOMOD/pages/491579/Ubuntu+Setup#UbuntuSetup-BringUpToDate
RUN apt-get -y update 

RUN apt-get -y install \
        apt-utils \
        apt-transport-https \
        python-catkin-tools \
        catkin \
        xsdcxx \ 
        devscripts \
        equivs=2.1.0 \ 
        debhelper \ 
        javahelper
        
# Copies your code file from your action repository to the filesystem path `/` of the container
COPY entrypoint.sh /entrypoint.sh
COPY release-build.sh /release-build.sh
COPY story-build.sh /story-build.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]