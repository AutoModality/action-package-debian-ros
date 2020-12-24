FROM ros:melodic

ENV DEBIAN_FRONTEND noninteractive

ARG CACHE_BUSTER="v5.1.2"
# Bring up to date https://automodality.atlassian.net/wiki/spaces/AUTOMOD/pages/491579/Ubuntu+Setup#UbuntuSetup-BringUpToDate
RUN apt-get -y update 

RUN apt-get -y install \
        apt-utils \
        apt-transport-https \
        python-catkin-tools \
        xsdcxx \ 
        devscripts \
        equivs \ 
        debhelper \ 
        javahelper
        
# Copies your code file from your action repository to the filesystem path `/` of the container
COPY package-ros.sh /package-ros.sh
COPY package.sh /package.sh
COPY release-build.sh /release-build.sh
COPY story-build.sh /story-build.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/package-ros.sh"]