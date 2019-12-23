FROM gazebo:libgazebo7
# General Requirements
RUN apt update
RUN apt install -q -y \ 
        libjansson-dev \
        nodejs \
        npm \
        libboost-dev \
        imagemagick \
        libtinyxml-dev \
        mercurial \
        cmake \
        build-essential
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