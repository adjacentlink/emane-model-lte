FROM fedora:37

RUN mkdir -p /opt/built

RUN dnf -y update

# Copy and install packages from the latest upsteam dependencies
WORKDIR /opt
RUN mkdir -p dependencies
COPY --from=gcr.io/automated-builds-303920/fedora37.emane /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/fedora37.opentestpoint /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/fedora37.openstatistic /opt/built/* dependencies/
RUN dnf -y install dependencies/*\.rpm

# Build tools and depends
RUN dnf -y install git gcc-c++ make autoconf automake libtool rpm-build
RUN dnf -y install libxml2-devel libpcap-devel pcre-devel libuuid-devel

RUN git clone https://github.com/adjacentlink/emane-model-lte -b develop

WORKDIR emane-model-lte
RUN ./autogen.sh && ./configure && make rpm
RUN cp $(find .rpmbuild/RPMS -name "*\.rpm") /opt/built
RUN dnf -y install /opt/built/*

RUN dnf clean all

RUN echo 'complete'

