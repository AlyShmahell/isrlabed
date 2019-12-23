
FROM ubuntu:18.04

LABEL author="Aly Shmahell"
LABEL email="aly.shmahell@student.univaq.it"
RUN apt update
RUN apt install -q -y curl
RUN curl -sL https://deb.nodesource.com/setup_13.x | -E bash -
RUN apt update
RUN apt install -q -y libjansson-dev nodejs npm nodejs-legacy libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential


FROM gazebo:libgazebo7


RUN apt install -q -y gazebo7 libgazebo7-dev
RUN RUN hg clone https://bitbucket.org/osrf/gzweb ~/gzweb
RUN cd ~/gzweb \
    && hg up default \
    && ./deploy.sh -m
EXPOSE 8080
EXPOSE 7681
RUN mkdir /root/gzweb/http/client/assets
RUN cd /root/gzweb && ./deploy.sh -m -t
CMD ["gzserver"]
CMD ["npm" "start"]