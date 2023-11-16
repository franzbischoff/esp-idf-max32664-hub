FROM espressif/idf:release-v4.4 AS build-44
FROM espressif/idf:release-v5.0 AS build-50
FROM espressif/idf:release-v5.1 AS build-51

ARG DEBIAN_FRONTEND=nointeractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# added some libs for myself
RUN apt-get update \
  && apt-get install -y -q --no-install-recommends \
  cmake \
  gpg \
  curl \
  git \
  doxygen \
  graphviz \
  universal-ctags \
  libjansson4 \
  libyaml-0-2 \
  astyle \
  mlocate \
  nano \
  hwdata \
  libglib2.0-0 \
  libnuma1 \
  libpixman-1-0 \
  linux-tools-virtual \
  && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/local/bin/usbip usbip "$(find /usr/lib/linux-tools/*/usbip | tail -n1)" 20

# QEMU
ENV QEMU_VER=8.1.2
ENV QEMU_REL=20231017
ENV QEMU_SHA256=3f5a60b92ac10950c424d5494e91ae43e65dd2ba48b590050712adc9f40f6625
ENV QEMU_FILE=esp-qemu-xtensa-softmmu-develop_${QEMU_VER}_${QEMU_REL}-x86_64-linux-gnu.tar.bz2
ENV QEMU_URL=https://github.com/espressif/qemu/releases/download/esp-develop-${QEMU_VER}-${QEMU_REL}/${QEMU_FILE}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN curl -sSL -o ${QEMU_FILE} ${QEMU_URL} \
  && echo "${QEMU_SHA256} *${QEMU_FILE}" | sha256sum --check --strict - \
  && tar -xf $QEMU_FILE -C /opt \
  && rm ${QEMU_FILE}

RUN curl https://get.trunk.io -fsSL | bash -s -- -y

RUN curl https://raw.githubusercontent.com/petervanderdoes/gitflow-avh/develop/contrib/gitflow-installer.sh -fsSL | bash -s -- install stable

ENV PATH=/opt/qemu/bin:${PATH}

RUN curl -sSL -o /usr/bin/hadolint https://github.com/hadolint/hadolint/releases/download/v2.12.0/hadolint-Linux-x86_64 \
  && chmod 755 /usr/bin/hadolint

RUN curl -sSL -o vale.tar.gz https://github.com/errata-ai/vale/releases/download/v2.28.0/vale_2.28.0_Linux_64-bit.tar.gz \
  && tar -xf vale.tar.gz -C /usr/bin \
  && rm vale.tar.gz

RUN curl -sSL -o packages-microsoft-prod.deb "https://packages.microsoft.com/config/ubuntu/$(lsb_release -rs)/packages-microsoft-prod.deb" \
  && dpkg -i packages-microsoft-prod.deb \
  && rm packages-microsoft-prod.deb

RUN curl -fsSL https://files.pvs-studio.com/etc/pubkey.txt  | apt-key add - \
  && curl -sSL -o /etc/apt/sources.list.d/viva64.list https://files.pvs-studio.com/etc/viva64.list \
  && apt-get update \
  && apt-get install -y -q --no-install-recommends \
  pvs-studio \
  strace \
  && rm -rf /var/lib/apt/lists/*

# academic work
RUN pvs-studio-analyzer credentials PVS-Studio Free FREE-FREE-FREE-FREE

# installed dotnet for Astyle plugin. You'll get some warning about missing dotnet, but you can ignore it
# RUN wget --no-verbose https://dot.net/v1/dotnet-install.sh \
#   && chmod 755 dotnet-install.sh \
#   && ./dotnet-install.sh -c STS \
#   && rm dotnet-install.sh

# RUN pip install --no-cache-dir -r /opt/esp/idf/docs/requirements.txt

RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> ~/.bashrc \
  && updatedb

## idf_tools.py install xtensa-clang

# trunk-ignore(semgrep/dockerfile.security.missing-user.missing-user)
ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

# trunk-ignore(semgrep/dockerfile.security.missing-user.missing-user)
CMD ["/bin/bash"]
