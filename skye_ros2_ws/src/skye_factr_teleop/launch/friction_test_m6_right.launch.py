# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

from skye_factr_teleop.launch_support import friction_test_launch_description


def generate_launch_description():
    return friction_test_launch_description("right")
