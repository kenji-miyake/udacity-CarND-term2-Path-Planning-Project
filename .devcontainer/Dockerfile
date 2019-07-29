#-------------------------------------------------------------------------------------------------------------
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License. See https://go.microsoft.com/fwlink/?linkid=2090316 for license information.
#-------------------------------------------------------------------------------------------------------------

FROM debian:10

# Avoid warnings by switching to noninteractive
ENV DEBIAN_FRONTEND=noninteractive

# Configure apt and install packages
RUN apt-get update \
  && apt-get -y install --no-install-recommends apt-utils 2>&1 \
  #
  # Verify vim, jq, git, process tools, lsb-release (useful for CLI installs) installed
  && apt-get -y install vim jq git procps lsb-release \
  #
  # Install C++ tools
  && apt-get -y install build-essential cmake cppcheck valgrind libtinfo5 \
  #
  # Clean up
  && apt-get autoremove -y \
  && apt-get clean -y \
  && rm -rf /var/lib/apt/lists/*

# Switch back to dialog for any ad-hoc use of apt-get
ENV DEBIAN_FRONTEND=dialog

# Install uWebSockets
RUN apt-get update \
  && apt-get -y install libuv1-dev libssl-dev zlib1g-dev \
  && git clone https://github.com/uWebSockets/uWebSockets \
  && cd uWebSockets \
  && git checkout e94b6e1 \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make \
  && make install \
  && cd ../../ \
  && ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so \
  && rm -r uWebSockets
