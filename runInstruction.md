# Run Instructions

## Prerequisites

- Docker and `nvidia-docker2` (if using CUDA acceleration)
- Source code for COVINS and wrapper available on host machine
- Sufficient build resources (multi-core CPU recommended)

## Building the Docker Images

Step 1: Build the base image with dependencies.

```bash
docker build -f sparx_docker/Dockerfile.base -t covins_base_abs_layer --build-arg NR_JOBS_BASE=14 .
```

Step 2: Build the development image.

```bash
docker build -f sparx_docker/Dockerfile.dev -t covins_dev_abs_layer --build-arg NR_JOBS=$(nproc) .
```