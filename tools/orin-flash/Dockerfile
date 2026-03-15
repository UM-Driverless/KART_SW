FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Flash dependencies (from NVIDIA docs)
RUN apt-get update && apt-get install -y --no-install-recommends \
    abootimg binfmt-support binutils cpio cpp \
    device-tree-compiler dosfstools lbzip2 libxml2-utils \
    nfs-kernel-server python3-yaml sshpass udev \
    qemu-user-static wget ca-certificates sudo \
    && rm -rf /var/lib/apt/lists/*

# Download and extract L4T R36.5 BSP + root filesystem
WORKDIR /jetson-flash
RUN wget -q --show-progress \
    https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v5.0/release/Jetson_Linux_r36.5.0_aarch64.tbz2 \
    && tar xf Jetson_Linux_r36.5.0_aarch64.tbz2 \
    && rm Jetson_Linux_r36.5.0_aarch64.tbz2

RUN wget -q --show-progress \
    https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v5.0/release/Tegra_Linux_Sample-Root-Filesystem_r36.5.0_aarch64.tbz2 \
    && tar xpf Tegra_Linux_Sample-Root-Filesystem_r36.5.0_aarch64.tbz2 -C Linux_for_Tegra/rootfs/ \
    && rm Tegra_Linux_Sample-Root-Filesystem_r36.5.0_aarch64.tbz2

# Prepare the flash environment
WORKDIR /jetson-flash/Linux_for_Tegra
RUN ./tools/l4t_flash_prerequisites.sh && ./apply_binaries.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
