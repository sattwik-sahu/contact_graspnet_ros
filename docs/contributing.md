---
title: Contributing
description: Development workflow, submodule policy, tests and documentation.
permalink: /contributing/
layout: default
parent: Project
nav_order: 1
---
## Ground rules

- The `third_party/contact_graspnet_pytorch` submodule is **read-only**. Do not
  edit files inside it. All fixes and orchestration belong in
  `src/contact_graspnet_ros`.
- Keep the `core/` package free of ROS imports. ROS belongs in `node.py`,
  `conversions.py` and `config.py`.
- Configuration changes belong in `config/`, not hard-coded in Python.
- Update the docs when behavior, interfaces or defaults change.

## Branching and commits

Use short, descriptive branches:

```text
feat/<topic>
fix/<topic>
docs/<topic>
```

Prefer conventional commit subjects, for example:

```text
feat: add point-cloud service
fix: handle empty depth frames
docs: document Zenoh setup
```

## Development environment

```bash
pixi install -e jazzy
pixi run -e jazzy build
pixi run -e jazzy test
pixi run -e jazzy lint
pixi run -e jazzy typecheck
```

The Jazzy environment is the primary target. Humble is a secondary build target.
Do not introduce Jazzy-only APIs without a fallback or a documented limitation.

## Python style

- Type-annotate public APIs.
- Use `ruff` for linting and formatting.
- Run `mypy` on `core/` before opening a pull request.
- Prefer small, testable functions over large ROS callbacks.
- Do not import `rclpy`, `cv_bridge` or ROS messages in `core/`.

## Tests

Add tests for:

- geometry and coordinate transforms,
- parameter validation,
- model/checkpoint loading,
- service request/response behavior when feasible.

GPU tests may be skipped when CUDA is unavailable; mark them clearly.

## Documentation

The documentation is a Jekyll site in `docs/` using the Just the Docs theme.

```bash
cd docs
bundle install
bundle exec jekyll serve
```

Before committing docs changes:

```bash
cd docs
bundle exec jekyll build
```

Add new pages under `docs/` with Just the Docs front matter
(`title`, `parent`, `nav_order`). Internal links should use:

```liquid
[Page]({{ '/page/' | relative_url }})
```

## Pull request checklist

- [ ] `pixi run -e jazzy build` passes.
- [ ] `pixi run -e jazzy test` passes.
- [ ] `pixi run -e jazzy lint` and `typecheck` pass.
- [ ] No changes inside `third_party/`.
- [ ] New parameters are documented in `docs/configuration.md`.
- [ ] New interfaces are documented in `docs/ros-interfaces.md`.
- [ ] Docs build passes.
- [ ] Docker build passes when the image is affected.

## Reporting issues

Include:

- ROS distro and Pixi environment,
- GPU/driver and `torch.cuda.is_available()`,
- exact command and full error,
- whether a mask is supplied and the relevant `segmap_id`,
- node logs and the service response where relevant.
