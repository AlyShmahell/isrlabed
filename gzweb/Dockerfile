FROM gazebo:libgazebo7
RUN apt-get update && apt-get upgrade -q -y && apt install -q -y \
		curl && rm -rf /var/lib/apt/lists/*
RUN curl -sL https://deb.nodesource.com/setup_10.x | bash -
RUN apt-get update && apt-get upgrade -q -y && apt install -q -y \ 
        libjansson-dev \
        libboost-all-dev \
        libgts-dev \
        imagemagick \
        libtinyxml-dev \
        mercurial \
        cmake \
        pkg-config \
        psmisc \
        build-essential \
		nodejs && rm -rf /var/lib/apt/lists/*
RUN apt install -q -y libgazebo7-dev && rm -rf /var/lib/apt/lists/*
# Gzweb clone
RUN hg clone https://bitbucket.org/osrf/gzweb ~/gzweb
# Gzweb deployment
RUN cd ~/gzweb \
    && hg up gzweb_1.4.0
RUN mkdir -p ~/gzweb/http/client/assets
RUN cd ~/gzweb \
    && npm run deploy --- -m
EXPOSE 8080
EXPOSE 7681
CMD ["gzserver" "--verbose"]
CMD npm start --prefix ~/gzweb
# Info
LABEL author="Aly Shmahell"
LABEL email="aly.shmahell@student.univaq.it"