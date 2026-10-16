# Third-party notices

This repository includes components derived from other open-source projects.
Their licenses apply to those components independently of the repository's
license.

## Velodyne simulator

The following directories contain code and assets derived from the Velodyne
simulator packages maintained by Dataspeed Inc. and later ROS 2 contributors:

- `src/velodyne_description`
- `src/velodyne_gazebo_plugins`

These components are distributed under the BSD 3-Clause License. The original
copyright notices in the source files must be retained. A copy of the upstream
license is provided in `LICENSES/BSD-3-Clause-Velodyne.txt`.

The imported version was compared with upstream commit
`ff155fa33abef0b0c160489e5bdfe14f812ff49e` from
`https://github.com/rahgirrafi/velodyne_simulator_ros2_gz`. Local changes are
present in the VLP-16 Xacro and Gazebo plugin source files.

## Gazebo model database assets

The following assets were obtained from the official Gazebo model database at
`https://github.com/osrf/gazebo_models`, using repository revision
`8163eb4b5e7e21985c6591d1c0bfb56468c0093f`:

- `src/environment_description/models/dumpster`
  - Original model: Dumpster
  - Original authors: Cole Biesemeyer and Nate Koenig
  - Upstream source: `https://github.com/osrf/gazebo_models/tree/master/dumpster`
  - Local changes include Gazebo Fortress-compatible SDF and material settings,
    model scale, static-model configuration, and resource paths.
- `src/environment_description/models/sandandmud/materials/textures/rough_concrete_diffuse.jpg`
  - Original model: Collapsed industrial building
  - Original author: Nate Koenig
  - Upstream source:
    `https://github.com/osrf/gazebo_models/tree/master/collapsed_industrial`
  - The texture file is redistributed without modification.

These assets are distributed under the Creative Commons Attribution 3.0
Unported License (CC BY 3.0). A copy of the upstream license notice is provided
in `LICENSES/CC-BY-3.0-Gazebo-Models.txt`.

## Project-authored terrain meshes

The `sandandmud_x*.dae` terrain meshes under
`src/environment_description/models/sandandmud/meshes` were created by the
authors of this repository and are covered by the repository's MIT License.

## Modified articulated wheel-loader meshes

The wheel-loader meshes under `src/articulated_description/meshes` are substantially modified derivatives of assets from [GANXX/articulated_vehicle](https://github.com/GANXX/articulated_vehicle), commit `64ede1ddbe678014e6b0e5f301b8c0a0978b05e1`.

The upstream `articulated_model/robot_sim_demo/package.xml` declares the package license as "BSD", but does not specify a BSD variant or provide a complete license text. The applicability of this declaration to the individual mesh assets has not been independently confirmed. Accordingly, this repository does not identify these assets as specifically BSD-2-Clause or BSD-3-Clause.

The meshes have been extensively modified, including geometry cleanup, rescaling, component redesign, and joint-position alignment. These modifications do not supersede any rights associated with the original assets.

The upstream license terms and redistribution permissions for these assets remain subject to clarification with the original author.

## Asset requiring further provenance review

Before publishing, confirm and document the source and redistribution license
for the remaining externally sourced environment texture under
`src/environment_description/models/warehouse_80x70`.
