FROM ros:kinetic

# Bring up to date https://automodality.atlassian.net/wiki/spaces/AUTOMOD/pages/491579/Ubuntu+Setup#UbuntuSetup-BringUpToDate
RUN apt-get -y update
RUN apt-get -y dist-upgrade

RUN apt-get -y install apt-utils apt-transport-https python-catkin-tools xsdcxx devscripts equivs javahelper 
RUN usermod -aG dialout root 




# Copies your code file from your action repository to the filesystem path `/` of the container
COPY entrypoint.sh /entrypoint.sh

# Code file to execute when the docker container starts up (`entrypoint.sh`)
ENTRYPOINT ["/entrypoint.sh"]