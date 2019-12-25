FROM gazebo:libgazebo7
# General Requirements
RUN apt-get update && apt-get upgrade -q -y && apt install -q -y \ 
        libjansson-dev \
        nodejs \
        npm \
        nodejs-legacy \
        libboost-all-dev \
        libgts-dev \
        imagemagick \
        libtinyxml-dev \
        mercurial \
        cmake \
        pkg-config \
        psmisc \
        build-essential && rm -rf /var/lib/apt/lists/*
# Gazebo Requirements
RUN apt install -q -y gazebo7 libgazebo7-dev && rm -rf /var/lib/apt/lists/*
# Gzweb clone
RUN hg clone https://bitbucket.org/osrf/gzweb ~/gzweb
# Gzweb deployment
RUN cd ~/gzweb \
    && hg up default \
    && ./deploy.sh -m
EXPOSE 8080
EXPOSE 7681
RUN mkdir /root/gzweb/http/client/assets
CMD cd /root/gzweb && ./deploy.sh -m -t
CMD ["gzserver"]
CMD ["npm" "start"]
# Info
LABEL author="Aly Shmahell"
LABEL email="aly.shmahell@student.univaq.it"