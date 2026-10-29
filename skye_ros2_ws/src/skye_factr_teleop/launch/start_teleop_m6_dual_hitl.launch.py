# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Dual M6 leaders remapped for HITL DAgger (teleop branch → control_arbiter).

Do not use together with ``start_teleop_m6_dual.launch.py`` on the same Domain:
that launch writes ``/gento/*_joint_control`` directly and would bypass the
arbiter. Start ``skye_hitl_dagger`` ``hitl_dagger.launch.py`` after this file
so arbiter owns ``/gento/*_joint_control(_abs)``.

Pass the same ``robot_profile:=thor|orin`` as on the development-machine
driver and ``hitl_dagger.launch.py`` (selects ``config/`` vs ``config/orin/``).
"""

from skye_factr_teleop.launch_support import leader_launch_description


def generate_launch_description():
    return leader_launch_description(("left", "right"), hitl=True)
