from __future__ import annotations

import argparse
import sys
from typing import Any

ACC_RATIO_SERVICE_CANDIDATES = (
    "/control/set_acc_ratio",
    "/control/set_accel_ratio",
    "/control/set_acceleration_ratio",
)


def set_int_request_value(request: Any, value: int) -> Any:
    if hasattr(request, "data"):
        request.data = int(value)
        return request
    slots = [slot for slot in getattr(request, "__slots__", ()) if not slot.startswith("_")]
    if len(slots) != 1:
        raise AttributeError("Cannot determine integer request field for marvin_msgs/srv/Int")
    setattr(request, slots[0], int(value))
    return request


def find_first_available_service(
    available_services: list[tuple[str, list[str]]],
    candidates: list[str] | tuple[str, ...],
) -> str | None:
    available_names = {name for name, _types in available_services}
    for candidate in candidates:
        if candidate in available_names:
            return candidate
    return None


def is_service_available(available_services: list[tuple[str, list[str]]], service_name: str) -> bool:
    return any(name == service_name for name, _types in available_services)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Prepare Skye low-level controller for leader-arm teleoperation."
    )
    parser.add_argument("--vel-ratio", type=int, default=10, help="Velocity ratio for first tests.")
    parser.add_argument("--acc-ratio", type=int, default=10, help="Acceleration ratio for first tests.")
    parser.add_argument(
        "--acc-service",
        default="",
        help=(
            "Acceleration ratio service. If omitted, the script tries common names "
            "and warns when no service exists."
        ),
    )
    parser.add_argument("--mode", type=int, default=3, help="Robot control mode, 3 is impedance by convention.")
    parser.add_argument("--timeout", type=float, default=10.0, help="Seconds to wait for each service.")
    parser.add_argument("--skip-clear-fault", action="store_true")
    parser.add_argument("--skip-ready", action="store_true")
    parser.add_argument("--skip-vel-ratio", action="store_true")
    parser.add_argument("--skip-acc-ratio", action="store_true")
    parser.add_argument(
        "--strict-optional-services",
        action="store_true",
        help="Fail when optional clear/ready/ratio services are missing.",
    )
    args = parser.parse_args(argv)

    import rclpy
    from std_srvs.srv import Trigger

    try:
        from marvin_msgs.srv import Int
    except ImportError as exc:
        raise RuntimeError(
            "Cannot import marvin_msgs.srv.Int. Source /opt/kernelmind/apex/install/setup.bash first."
        ) from exc

    rclpy.init()
    node = rclpy.create_node("skye_prepare_impedance")
    try:
        available_services = node.get_service_names_and_types()
        if not args.skip_clear_fault:
            _call_optional_trigger(
                node,
                Trigger,
                "/control/clear_fault",
                args.timeout,
                available_services,
                args.strict_optional_services,
            )
        if not args.skip_ready:
            _call_optional_trigger(
                node,
                Trigger,
                "/control/set_ready",
                args.timeout,
                available_services,
                args.strict_optional_services,
            )
        if not args.skip_vel_ratio:
            _call_optional_int(
                node,
                Int,
                "/control/set_vel_ratio",
                args.vel_ratio,
                args.timeout,
                available_services,
                args.strict_optional_services,
            )
        if not args.skip_acc_ratio:
            acc_service = args.acc_service.strip() or find_first_available_service(
                available_services,
                ACC_RATIO_SERVICE_CANDIDATES,
            )
            if acc_service:
                _call_optional_int(
                    node,
                    Int,
                    acc_service,
                    args.acc_ratio,
                    args.timeout,
                    available_services,
                    args.strict_optional_services,
                )
            else:
                node.get_logger().warn(
                    "No acceleration ratio service found. Set acceleration to 10% in the UI "
                    "or pass --acc-service once the service name is known."
                )
        _call_int(node, Int, "/control/set_mode", args.mode, args.timeout)
        node.get_logger().info(
            "Skye prepare sequence finished: "
            f"vel_ratio={args.vel_ratio}, acc_ratio={args.acc_ratio}, mode={args.mode}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _call_optional_trigger(
    node: Any,
    srv_type: Any,
    service_name: str,
    timeout: float,
    available_services: list[tuple[str, list[str]]],
    strict: bool,
) -> Any | None:
    if not is_service_available(available_services, service_name):
        message = f"Optional service {service_name} is not available; skipping"
        if strict:
            raise RuntimeError(message)
        node.get_logger().warn(message)
        return None
    return _call_trigger(node, srv_type, service_name, timeout)


def _call_optional_int(
    node: Any,
    srv_type: Any,
    service_name: str,
    value: int,
    timeout: float,
    available_services: list[tuple[str, list[str]]],
    strict: bool,
) -> Any | None:
    if not is_service_available(available_services, service_name):
        message = f"Optional service {service_name} is not available; skipping"
        if strict:
            raise RuntimeError(message)
        node.get_logger().warn(message)
        return None
    return _call_int(node, srv_type, service_name, value, timeout)


def _call_trigger(node: Any, srv_type: Any, service_name: str, timeout: float) -> Any:
    client = node.create_client(srv_type, service_name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"Service {service_name} is not available after {timeout:.1f}s")
    future = client.call_async(srv_type.Request())
    response = _spin_until_done(node, future, service_name, timeout)
    success = getattr(response, "success", True)
    message = getattr(response, "message", "")
    node.get_logger().info(f"{service_name}: success={success} message={message!r}")
    if success is False:
        raise RuntimeError(f"Service {service_name} returned success=false: {message}")
    return response


def _call_int(node: Any, srv_type: Any, service_name: str, value: int, timeout: float) -> Any:
    client = node.create_client(srv_type, service_name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(f"Service {service_name} is not available after {timeout:.1f}s")
    request = set_int_request_value(srv_type.Request(), value)
    future = client.call_async(request)
    response = _spin_until_done(node, future, service_name, timeout)
    success = getattr(response, "success", True)
    message = getattr(response, "message", "")
    node.get_logger().info(
        f"{service_name}({value}): success={success} message={message!r}"
    )
    if success is False:
        raise RuntimeError(f"Service {service_name} returned success=false: {message}")
    return response


def _spin_until_done(node: Any, future: Any, service_name: str, timeout: float) -> Any:
    import rclpy

    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f"Timed out waiting for {service_name} response")
    exc = future.exception()
    if exc is not None:
        raise RuntimeError(f"Service {service_name} failed: {exc}") from exc
    return future.result()


if __name__ == "__main__":
    sys.exit(main())
