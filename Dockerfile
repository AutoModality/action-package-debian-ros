FROM ros:melodic-perception

ENV DEBIAN_FRONTEND noninteractive

ARG CACHE_BUSTER="v3.2.5"
# Bring up to date https://automodality.atlassian.net/wiki/spaces/AUTOMOD/pages/491579/Ubuntu+Setup#UbuntuSetup-BringUpToDate
RUN apt-get -y update 

RUN apt-get -y install \
        apt-utils=1.6.12ubuntu0.1 \
        apt-transport-https=1.6.12ubuntu0.1 \
        python-catkin-tools=0.6.1-1 \
        xsdcxx=4.0.0-7build1 \ 
        devscripts=2.17.12ubuntu1.1 \
        equivs=2.1.0 \ 
        debhelper=12.1.1ubuntu1~ubuntu18.04.1 \ 
        javahelper=0.72.1~18.04.1
        
# Copies your code file from your action repository to the filesystem path `/` of the container
COPY entrypoint.sh /entrypoint.sh
COPY release-build.sh /release-build.sh
COPY story-build.sh /story-build.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]