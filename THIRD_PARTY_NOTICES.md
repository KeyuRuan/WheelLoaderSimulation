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

## Gazebo Dumpster model

`src/environment_description/models/dumpster` identifies Cole Biesemeyer and
Nate Koenig as its original authors in `model.config`. Keep that attribution
with the model. Its exact redistribution license must be confirmed against the
original Gazebo model source before a public release.

## Environment textures and custom meshes

Before publishing, confirm and document the source and redistribution license
for every texture and mesh under `src/environment_description/models` and for
the custom wheel-loader meshes under `src/articulated_description/meshes`.
File names or embedded authoring-tool metadata alone are not proof of a license.
