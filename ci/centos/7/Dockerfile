FROM centos:centos7

RUN mkdir -p /opt/built

RUN yum -y install epel-release
RUN yum -y update

# Copy and install packages from the latest opentestpoint build
WORKDIR /opt
RUN mkdir -p dependencies
COPY --from=gcr.io/automated-builds-303920/centos7.emane /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/centos7.opentestpoint /opt/built/* dependencies/
COPY --from=gcr.io/automated-builds-303920/centos7.openstatistic /opt/built/* dependencies/
RUN yum -y install dependencies/*\.rpm

# Build tools
RUN yum -y install git gcc-c++ make autoconf automake libtool rpm-build
RUN yum -y install libxml2-devel libpcap-devel pcre-devel libuuid-devel

# Install devtoolset-9 to support c++17 compatibility
RUN yum -y install centos-release-scl
RUN yum -y install devtoolset-9

RUN git clone https://github.com/adjacentlink/emane-model-lte -b develop
WORKDIR emane-model-lte

# Using devtoolset-9
RUN scl enable devtoolset-9 "./autogen.sh && ./configure && make rpm"
RUN cp $(find .rpmbuild/RPMS -name "*\.rpm") /opt/built
RUN yum -y install /opt/built/*

RUN yum clean all

RUN echo 'complete'

