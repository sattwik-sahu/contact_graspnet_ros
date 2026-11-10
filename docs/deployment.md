# Deployment

## Host (Pixi)

```bash
pixi install -e jazzy
pixi run -e jazzy build
pixi run -e jazzy zenohd   # terminal 1
pixi run -e jazzy launch   # terminal 2
```

The activation script sets:

- `CONTACT_GRASPNET_ROOT`
- `GRASPNET_CONFIG_DIR`
- `PYTHONPATH` for the vendored port
- `RMW_IMPLEMENTATION=rmw_zenoh_cpp` on Jazzy

Call the service:

```bash
pixi run -e jazzy python scripts/call_grasp_service.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --segmap-id 1
```

## Docker

```bash
git submodule update --init --recursive
docker compose build
docker compose up
```

The root `docker-compose.yaml` includes
`docker/compose.contact_graspnet.yaml` and adds a Zenoh router.

The image contains the Pixi environment, colcon install space, wrapper source
and vendored port source. It does **not** contain checkpoint weights; they are
mounted read-only.

Once the container is running, call `/graspnet/generate_grasps` from another
ROS 2 container or from the host with the same RMW/router configuration.

### Mounts

| Host | Container | Purpose |
|---|---|---|
| `./config` | `/opt/contact_graspnet_ros/config` | ROS + model config |
| checkpoint dir | `/opt/contact_graspnet_ros/checkpoints` | `model.pt` |

Override with `GRASPNET_CONFIG_DIR` and `GRASPNET_CHECKPOINT_DIR`.

### GPU

Compose requests one NVIDIA GPU. Verify the host has the NVIDIA Container
Toolkit installed. For CPU-only, remove the `deploy` block and set
`device: cpu` in `config/graspnet.yaml`.

### Networking

The services use `network_mode: host` for reliable DDS/Zenoh discovery on
Linux. If the parent Compose project already runs a Zenoh router, remove the
`zenoh-router` service. If the parent uses Fast DDS, set
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

### Include-able file

```yaml
include:
  - path: docker/compose.contact_graspnet.yaml
```

The included file defines the `contact_graspnet` service. A parent project can
add `depends_on`, network overrides, or its own Zenoh router.

## Zenoh (Jazzy)

```bash
pixi run -e jazzy zenohd
```

or use the `zenoh-router` service in `docker-compose.yaml`.

## GitHub Pages documentation

The documentation site is built from `docs/` with the
[Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme.

- Site: https://sattwik-sahu.github.io/contact_graspnet_ros/
- Source: `docs/`
- Deployment workflow: `.github/workflows/docs.yml`
- GitHub Pages source must be set to **GitHub Actions** in the repository
  settings.

The workflow rebuilds and deploys the site when `docs/**` changes on `main`.
It can also be run manually with `workflow_dispatch`.

Local preview:

```bash
pip install -r docs/requirements.txt
mkdocs serve
```

## Docker CI

`.github/workflows/docker.yml` builds the image with BuildKit on pushes that
touch the Dockerfile, Pixi environment, source, config or submodule. It does
not push an image; it only verifies that the image builds.

## Humble

Jazzy is first-class. The Pixi Humble environment is a secondary build target.
The current RoboStack Humble channel does not solve `rmw-zenoh-cpp` with Python
3.10, so the Pixi Humble environment uses Python 3.12 and Fast DDS. Validate
the native Humble Python 3.10 runtime separately before relying on it.

For the Humble environment, use the normal launch file with
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`. Do not start
`graspnet_zenoh.launch.py` unless a Humble Zenoh package is available; use
`start_router:=false` if you do.
