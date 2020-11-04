# Copyrights © 2020 Aly Shmahell

#!/bin/bash

apt update
apt install -y libglib2.0-dev\
               texinfo\
               make\
               automake\
               autoconf\
               perl\
               m4\
               gcc\
               libc-dev\
               pkg-config\
               wget\
               python3

python=/usr/bin/python
python3=/usr/bin/python3
if [ -L ${python} ] ; then
   if [ -e ${python} ] ; then
      echo "python is installed correctly"
   else
      ln -s ${python3} ${python}
   fi
else
   ln -s ${python3} ${python}
fi

cd /opt

wget http://staff.itee.uq.edu.au/pjr/HomePages/PedroFiles/pedro-1.11.tgz
wget http://staff.itee.uq.edu.au/pjr/HomePages/QPFiles/qp10.5.tar.gz
wget http://staff.itee.uq.edu.au/pjr/HomePages/QulogFiles/qulog0.7.tgz

tar -zxvf ./pedro-1.11.tgz  --transform='s/pedro-1.11/pedro/'
tar -zxvf ./qp10.5.tar.gz   --transform='s/qp10.5/qp/'
tar -zxvf ./qulog0.7.tgz    --transform='s/qulog0.7/qulog/'

rm -f pedro-1.11.tgz
rm -f qp10.5.tar.gz
rm -f qulog0.7.tgz

ln -s /opt/qp/bin/qc /usr/local/bin/qc

(cd pedro && autoreconf -f -i && ./configure && make && make install && cd src/python_api && python setup.py install)
(cd qp    && ./configure && make )
(cd qulog && ./configure && make)

ln -s /opt/qulog/bin/qulog  /usr/local/bin/qulog
ln -s /opt/qulog/bin/teleor /usr/local/bin/teleor

apt remove  -y --purge wget\
                       texinfo\
                       make\
                       automake\
                       autoconf\
                       gcc\
                       libc-dev\
                       pkg-config\
                       m4\
                       perl
apt clean   -y