# Third-party notices

The robot descriptions and STL meshes in this package were migrated from the
following locally installed ROS 2 description packages:

| Resource in this package | Migration source | Source manifest declaration |
|---|---|---|
| `urdf/m6left_isomorphism.urdf`, `urdf/m6right_isomorphism.urdf`, `meshes/leader/**` | `m6_isomorphism_description` | `BSD` |
| `urdf/skye_m6.urdf`, `meshes/follower/**` | `marvin_m6_description` | `BSD`; author `jjhu` |

The URDF files state that they were generated with the SolidWorks to URDF
Exporter. The migrated files were changed only to use the
`skye_factr_description` package name and the new package-relative mesh
locations. Unreferenced mesh files were not migrated.

The package scaffolding and migration changes are licensed under Apache-2.0.
