FROM gazebo:libgazebo7

LABEL author="Aly Shmahell"
LABEL email="aly.shmahell@student.univaq.it"

RUN curl -sL https://deb.nodesource.com/setup_13.x | sudo -E bash -
RUN sudo apt update
RUN sudo apt install gazebo7 libgazebo7-dev
RUN sudo apt install libjansson-dev nodejs npm nodejs-legacy libboost-dev imagemagick libtinyxml-dev mercurial cmake build-essential
RUN RUN hg clone https://bitbucket.org/osrf/gzweb ~/gzweb
RUN cd ~/gzweb \
    && hg up default \
    && ./deploy.sh -m
EXPOSE 8080
EXPOSE 7681
CMD mkdir /root/gzweb/http/client/assets
CMD cd /root/gzweb && ./deploy.sh -m -t
CMD ["gzserver"]
CMD ["npm" "start"]