# Stage 1
FROM ubuntu:18.04
# Pre-Requirements
RUN apt remove nodejs npm
RUN apt update
RUN apt install -q -y curl apt-utils
# General Requirements
RUN curl -sL https://deb.nodesource.com/setup_13.x | bash -
RUN apt update
RUN apt install -q -y libjansson-dev nodejs npm libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
# Stage 2
FROM gazebo:libgazebo7
# Gazebo Requirements
RUN apt install -q -y gazebo7 libgazebo7-dev
# Gzweb clone
RUN RUN hg clone https://bitbucket.org/osrf/gzweb ~/gzweb
# Gzweb deployment
RUN cd ~/gzweb \
    && hg up default \
    && ./deploy.sh -m
EXPOSE 8080
EXPOSE 7681
RUN mkdir /root/gzweb/http/client/assets
RUN cd /root/gzweb && ./deploy.sh -m -t
CMD ["gzserver"]
CMD ["npm" "start"]
# Info
LABEL author="Aly Shmahell"
LABEL email="aly.shmahell@student.univaq.it"