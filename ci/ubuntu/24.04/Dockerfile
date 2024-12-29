FROM ubuntu:24.04

RUN mkdir -p /opt/built

# prevent failures due to interactive apt transactions
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get -y update
RUN apt-get -y upgrade --no-install-recommends

# Copy and install packages from the latest opentestpoint build
WORKDIR /opt
RUN mkdir -p dependencies
COPY --from=gcr.io/automated-builds-303920/ubuntu24.04.emane /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/ubuntu24.04.opentestpoint /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/ubuntu24.04.openstatistic /opt/built/* dependencies/
RUN dpkg -i dependencies/*; apt-get -y -f install

# Build tools and dependencies
RUN apt-get -y install git gcc g++ autoconf automake libtool debhelper dh-python pkg-config python3-setuptools \
                       protobuf-compiler libprotobuf-dev python3-protobuf
RUN apt-get -y install libxml2-dev libpcap-dev libpcre3-dev uuid-dev

RUN git clone https://github.com/adjacentlink/emane-model-lte -b develop

WORKDIR emane-model-lte
RUN ./autogen.sh && ./configure && make deb
RUN cp $(find .debbuild -name "*\.deb") /opt/built
RUN dpkg -i /opt/built/*; apt-get -y install -f

RUN echo 'complete'

