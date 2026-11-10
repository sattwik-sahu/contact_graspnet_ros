---
layout: page
title: License
description: Wrapper and upstream licensing.
permalink: /license/
---

## This wrapper

The ROS 2 wrapper code in this repository is licensed under the **Apache-2.0**
license.

## Upstream model

The vendored `third_party/contact_graspnet_pytorch` submodule and its model
weights are covered by the upstream NVIDIA license. See:

- `third_party/contact_graspnet_pytorch/License.pdf`
- `third_party/NOTICE`

Model weights are not baked into the Docker image; they are mounted read-only at
runtime. Review the upstream license before redistributing weights or a
container image that contains them.

## Contact-GraspNet paper

If you use this work, cite the original paper:

```bibtex
@inproceedings{sundermeyer2021contact,
  title={Contact-GraspNet: Efficient 6-DoF Grasp Generation in Cluttered Scenes},
  author={Sundermeyer, Martin and Mousavian, Arsalan and Triebel, Rudolph and Fox, Dieter},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2021}
}
```
