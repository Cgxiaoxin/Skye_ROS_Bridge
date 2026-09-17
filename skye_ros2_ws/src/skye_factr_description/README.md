# skye_factr_description

ROS 2 description resources used by `skye_factr_teleop`.

Installed models:

- `urdf/m6left_isomorphism.urdf`: seven-axis left leader arm;
- `urdf/m6right_isomorphism.urdf`: seven-axis right leader arm;
- `urdf/skye_m6.urdf`: dual seven-axis follower model plus gripper links.

All mesh references use `package://skye_factr_description/...`. Runtime code
should resolve the URDF itself through the ament package index and pass the
resulting absolute filename to Pinocchio; it must not depend on a source or
`marvin_ws` path.

See `THIRD_PARTY_NOTICES.md` for the migration sources.
