FROM espressif/idf:release-v4.4 AS build-44
FROM espressif/idf:release-v5.0 AS build-50

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
ENV QEMU_REL=esp-develop-20220919
ENV QEMU_SHA256=f6565d3f0d1e463a63a7f81aec94cce62df662bd42fc7606de4b4418ed55f870
ENV QEMU_DIST=qemu-${QEMU_REL}.tar.bz2
ENV QEMU_URL=https://github.com/espressif/qemu/releases/download/${QEMU_REL}/${QEMU_DIST}

ENV LC_ALL=C.UTF-8
ENV LANG=C.UTF-8

RUN curl -sSL -o ${QEMU_DIST} ${QEMU_URL} \
  && echo "${QEMU_SHA256} *${QEMU_DIST}" | sha256sum --check --strict - \
  && tar -xf $QEMU_DIST -C /opt \
  && rm ${QEMU_DIST}

RUN curl https://get.trunk.io -fsSL | bash -s -- -y

RUN curl https://raw.githubusercontent.com/petervanderdoes/gitflow-avh/develop/contrib/gitflow-installer.sh -fsSL | bash -s -- install stable

ENV PATH=/opt/qemu/bin:${PATH}

RUN curl -sSL -o /usr/bin/hadolint https://github.com/hadolint/hadolint/releases/download/v2.12.0/hadolint-Linux-x86_64 \
  && chmod 755 /usr/bin/hadolint

RUN curl -sSL -o packages-microsoft-prod.deb "https://packages.microsoft.com/config/ubuntu/$(lsb_release -rs)/packages-microsoft-prod.deb" \
  && dpkg -i packages-microsoft-prod.deb \
  && rm packages-microsoft-prod.deb

# installed dotnet for Astyle plugin. You'll get some warning about missing dotnet, but you can ignore it
# RUN wget --no-verbose https://dot.net/v1/dotnet-install.sh \
#   && chmod 755 dotnet-install.sh \
#   && ./dotnet-install.sh -c STS \
#   && rm dotnet-install.sh

# RUN pip install --no-cache-dir -r /opt/esp/idf/docs/requirements.txt

RUN echo "source /opt/esp/idf/export.sh > /dev/null 2>&1" >> ~/.bashrc \
  && updatedb

# trunk-ignore(semgrep/dockerfile.security.missing-user.missing-user)
ENTRYPOINT [ "/opt/esp/entrypoint.sh" ]

# trunk-ignore(semgrep/dockerfile.security.missing-user.missing-user)
CMD ["/bin/bash"]
