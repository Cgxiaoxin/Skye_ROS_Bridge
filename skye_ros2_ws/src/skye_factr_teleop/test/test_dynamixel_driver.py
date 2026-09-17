# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""SDK-interaction contract tests for the production Dynamixel adapter.

These tests intentionally replace only the four ``dynamixel_sdk`` objects.  No
serial port or Dynamixel hardware is accessed.
"""

from collections import deque
import math

import pytest

from skye_factr_teleop import dynamixel_driver as driver_module


COMM_SUCCESS = 0
IDS = (1, 2)
SERVO_TYPES = ("XM430-W350", "XC330-T288")


def test_control_table_constants_match_protocol_2_layout():
    assert driver_module.PROTOCOL_VERSION == 2.0
    assert driver_module.ADDR_OPERATING_MODE == 11
    assert driver_module.ADDR_TORQUE_ENABLE == 64
    assert driver_module.ADDR_GOAL_CURRENT == 102
    assert driver_module.LEN_GOAL_CURRENT == 2
    assert driver_module.ADDR_PRESENT_VELOCITY == 128
    assert driver_module.LEN_PRESENT_VELOCITY == 4
    assert driver_module.ADDR_PRESENT_POSITION == 132
    assert driver_module.LEN_PRESENT_POSITION == 4
    assert driver_module.LEN_SYNC_STATE == 8
    assert driver_module.CURRENT_CONTROL_MODE == 0


@pytest.mark.parametrize(
    "raw, expected",
    [
        (0x00000000, 0),
        (0x7FFFFFFF, 2_147_483_647),
        (0x80000000, -2_147_483_648),
        (0xFFFFFFFF, -1),
    ],
)
def test_uint32_to_int32(raw, expected):
    assert driver_module.uint32_to_int32(raw) == expected


@pytest.mark.parametrize(
    "value, expected",
    [
        (0, [0x00, 0x00]),
        (1, [0x01, 0x00]),
        (32767, [0xFF, 0x7F]),
        (-1, [0xFF, 0xFF]),
        (-32768, [0x00, 0x80]),
    ],
)
def test_encode_goal_current_is_signed_16_bit_little_endian(value, expected):
    assert driver_module.encode_goal_current(value) == expected


@pytest.mark.parametrize("bad", [-32769, 32768])
def test_encode_goal_current_rejects_values_outside_int16(bad):
    with pytest.raises(ValueError, match="16-bit"):
        driver_module.encode_goal_current(bad)


def test_position_and_velocity_units_convert_to_si():
    assert driver_module.position_ticks_to_radians(0) == 0.0
    assert driver_module.position_ticks_to_radians(2048) == pytest.approx(math.pi)
    assert driver_module.position_ticks_to_radians(-2048) == pytest.approx(-math.pi)

    one_velocity_unit = 0.229 * 2.0 * math.pi / 60.0
    assert driver_module.velocity_units_to_rad_s(1) == pytest.approx(
        one_velocity_unit
    )
    assert driver_module.velocity_units_to_rad_s(-10) == pytest.approx(
        -10.0 * one_velocity_unit
    )


@pytest.mark.parametrize(
    "alias, canonical",
    [
        ("XM430-W350", "XM430-W350"),
        ("XM430-W350-T", "XM430-W350"),
        ("XM430_W350", "XM430-W350"),
        ("XM430_W350_T", "XM430-W350"),
        ("XC330-T288", "XC330-T288"),
        ("XC330-T288-T", "XC330-T288"),
        ("XC330_T288", "XC330-T288"),
        ("XC330_T288_T", "XC330-T288"),
    ],
)
def test_servo_type_aliases_normalize(alias, canonical):
    assert driver_module.normalize_servo_type(alias) == canonical


def test_model_current_units_limits_and_torque_conversion():
    xm = driver_module.get_servo_spec("XM430_W350_T")
    xc = driver_module.get_servo_spec("XC330-T288")

    assert xm.current_unit_ma == pytest.approx(2.69)
    assert xm.current_limit_raw == 1193
    assert xm.current_limit_ma == pytest.approx(1193 * 2.69)
    assert xm.torque_nm_per_amp == pytest.approx(1.783)

    assert xc.current_unit_ma == pytest.approx(1.0)
    assert xc.current_limit_raw == 910
    assert xc.current_limit_ma == pytest.approx(910.0)
    assert xc.torque_nm_per_amp == pytest.approx(1.136)

    assert driver_module.torque_nm_to_current_ma(1.783, "XM430-W350") == pytest.approx(
        1000.0
    )
    assert driver_module.torque_nm_to_current_ma(-1.136, "XC330_T288") == pytest.approx(
        -1000.0
    )
    assert driver_module.current_ma_to_raw(269.0, "XM430-W350") == 100
    assert driver_module.current_ma_to_raw(-100.0, "XC330-T288") == -100


def test_conversion_input_validation_and_model_current_limit():
    with pytest.raises(ValueError, match="unsupported servo type"):
        driver_module.normalize_servo_type("unknown")
    with pytest.raises(ValueError, match="finite"):
        driver_module.torque_nm_to_current_ma(float("nan"), "XM430-W350")
    with pytest.raises(ValueError, match="current limit"):
        driver_module.current_ma_to_raw(1000.0, "XC330-T288")


class FakePortHandler:
    def __init__(self, sdk, port):
        self.sdk = sdk
        self.port = port
        sdk.events.append(("PortHandler", port))

    def openPort(self):
        self.sdk.events.append(("openPort",))
        return self.sdk.open_ok

    def setBaudRate(self, baudrate):
        self.sdk.events.append(("setBaudRate", baudrate))
        return self.sdk.baudrate_ok

    def closePort(self):
        self.sdk.events.append(("closePort",))


class FakePacketHandler:
    def __init__(self, sdk, protocol_version):
        self.sdk = sdk
        sdk.events.append(("PacketHandler", protocol_version))

    def ping(self, port, dxl_id):
        self.sdk.events.append(("ping", dxl_id))
        return self.sdk.ping_results.get(dxl_id, (1234, COMM_SUCCESS, 0))

    def write1ByteTxRx(self, port, dxl_id, address, value):
        self.sdk.events.append(("write1", dxl_id, address, value))
        return self.sdk.write_results.get(
            (dxl_id, address, value), (COMM_SUCCESS, 0)
        )

    def read1ByteTxRx(self, port, dxl_id, address):
        self.sdk.events.append(("read1", dxl_id, address))
        return self.sdk.read_results.get(
            (dxl_id, address), (self.sdk.operating_mode, COMM_SUCCESS, 0)
        )

    def getTxRxResult(self, result):
        return f"communication error {result}"

    def getRxPacketError(self, error):
        return f"device error {error}"


class FakeGroupSyncRead:
    def __init__(self, sdk, port_handler, packet_handler, address, length):
        self.sdk = sdk
        sdk.events.append(("GroupSyncRead", address, length))

    def addParam(self, dxl_id):
        self.sdk.events.append(("read.addParam", dxl_id))
        return self.sdk.read_add_results.get(dxl_id, True)

    def txRxPacket(self):
        self.sdk.events.append(("read.txRxPacket",))
        if self.sdk.read_tx_results:
            return self.sdk.read_tx_results.popleft()
        return COMM_SUCCESS

    def isAvailable(self, dxl_id, address, length):
        self.sdk.events.append(("read.isAvailable", dxl_id, address, length))
        return self.sdk.available_results.get((dxl_id, address, length), True)

    def getData(self, dxl_id, address, length):
        self.sdk.events.append(("read.getData", dxl_id, address, length))
        return self.sdk.read_data.get((dxl_id, address, length), 0)


class FakeGroupSyncWrite:
    def __init__(self, sdk, port_handler, packet_handler, address, length):
        self.sdk = sdk
        sdk.events.append(("GroupSyncWrite", address, length))

    def addParam(self, dxl_id, data):
        self.sdk.events.append(("write.addParam", dxl_id, list(data)))
        return self.sdk.write_add_results.get(dxl_id, True)

    def txPacket(self):
        self.sdk.events.append(("write.txPacket",))
        return self.sdk.write_tx_result

    def clearParam(self):
        self.sdk.events.append(("write.clearParam",))


class FakeSdk:
    """Configurable fake factory for the four SDK object types."""

    def __init__(self):
        self.events = []
        self.open_ok = True
        self.baudrate_ok = True
        self.operating_mode = 0
        self.ping_results = {}
        self.write_results = {}
        self.read_results = {}
        self.read_add_results = {}
        self.read_tx_results = deque()
        self.available_results = {}
        self.read_data = {}
        self.write_add_results = {}
        self.write_tx_result = COMM_SUCCESS

    def port_handler(self, port):
        return FakePortHandler(self, port)

    def packet_handler(self, protocol_version):
        return FakePacketHandler(self, protocol_version)

    def group_sync_read(self, port_handler, packet_handler, address, length):
        return FakeGroupSyncRead(
            self, port_handler, packet_handler, address, length
        )

    def group_sync_write(self, port_handler, packet_handler, address, length):
        return FakeGroupSyncWrite(
            self, port_handler, packet_handler, address, length
        )


@pytest.fixture
def fake_sdk(monkeypatch):
    sdk = FakeSdk()
    monkeypatch.setattr(driver_module, "COMM_SUCCESS", COMM_SUCCESS, raising=False)
    monkeypatch.setattr(driver_module, "PortHandler", sdk.port_handler, raising=False)
    monkeypatch.setattr(
        driver_module, "PacketHandler", sdk.packet_handler, raising=False
    )
    monkeypatch.setattr(
        driver_module, "GroupSyncRead", sdk.group_sync_read, raising=False
    )
    monkeypatch.setattr(
        driver_module, "GroupSyncWrite", sdk.group_sync_write, raising=False
    )
    return sdk


@pytest.fixture
def driver_factory(fake_sdk, tmp_path):
    # A regular temporary file satisfies the production port-existence check
    # while remaining completely disconnected from real serial hardware.
    port = tmp_path / "ttyUSB-test"
    port.touch()

    def make_driver():
        return driver_module.DynamixelDriver(
            ids=IDS,
            servo_types=SERVO_TYPES,
            port=str(port),
            baudrate=4_000_000,
        )

    return make_driver


def test_constructs_protocol_2_sync_read_and_sync_write(driver_factory, fake_sdk):
    driver_factory()

    assert fake_sdk.events == [
        ("PortHandler", fake_sdk.events[0][1]),
        ("PacketHandler", 2.0),
        ("GroupSyncRead", 128, 8),
        ("GroupSyncWrite", 102, 2),
    ]


def test_open_performs_safe_initialization_in_order(driver_factory, fake_sdk):
    driver = driver_factory()
    fake_sdk.events.clear()

    driver.open()

    assert fake_sdk.events == [
        ("openPort",),
        ("setBaudRate", 4_000_000),
        ("ping", 1),
        ("ping", 2),
        ("read.addParam", 1),
        ("read.addParam", 2),
        ("write1", 1, 64, 0),
        ("write1", 2, 64, 0),
        ("write1", 1, 11, 0),
        ("write1", 2, 11, 0),
        ("read1", 1, 11),
        ("read1", 2, 11),
        ("write1", 1, 64, 1),
        ("write1", 2, 64, 1),
    ]
    assert driver.torque_enabled is True


def test_open_is_idempotent(driver_factory, fake_sdk):
    driver = driver_factory()
    driver.open()
    fake_sdk.events.clear()

    driver.open()

    assert fake_sdk.events == []


def test_by_id_resolution_and_ftdi_latency_read(monkeypatch, tmp_path):
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    device = tmp_path / "ttyUSB7"
    device.touch()
    link = by_id / "usb-FTDI-test"
    link.symlink_to(device)
    sys_tty = tmp_path / "sys-class-tty"
    latency = sys_tty / "ttyUSB7" / "device" / "latency_timer"
    latency.parent.mkdir(parents=True)
    latency.write_text("1\n", encoding="ascii")
    monkeypatch.setattr(driver_module, "SERIAL_BY_ID_DIRECTORY", by_id)
    monkeypatch.setattr(driver_module, "SYS_TTY_DIRECTORY", sys_tty)

    resolved = driver_module.resolve_serial_port("usb-FTDI-test")

    assert resolved == link
    assert driver_module.read_ftdi_latency_timer(resolved) == 1


def test_open_rejects_nonoptimal_ftdi_latency_before_bus_access(
    driver_factory, fake_sdk, monkeypatch
):
    driver = driver_factory()
    monkeypatch.setattr(driver_module, "read_ftdi_latency_timer", lambda port: 16)
    fake_sdk.events.clear()

    with pytest.raises(driver_module.DynamixelPortError, match="expected 1"):
        driver.open()

    assert fake_sdk.events == []


def test_operating_mode_change_requires_torque_disabled(driver_factory, fake_sdk):
    driver = driver_factory()
    driver.open()
    fake_sdk.events.clear()

    with pytest.raises(driver_module.DynamixelError, match="torque.*disabled"):
        driver.set_operating_mode(0)
    assert fake_sdk.events == []

    driver.set_torque_enabled(False)
    fake_sdk.events.clear()
    driver.set_operating_mode(0)
    assert fake_sdk.events == [
        ("write1", 1, 11, 0),
        ("write1", 2, 11, 0),
    ]


def test_enabling_torque_first_verifies_current_mode(driver_factory, fake_sdk):
    driver = driver_factory()
    driver.open()
    driver.set_torque_enabled(False)
    fake_sdk.operating_mode = 3
    fake_sdk.events.clear()

    with pytest.raises(driver_module.DynamixelSdkError, match="device ID 1"):
        driver.set_torque_enabled(True)

    assert fake_sdk.events == [("read1", 1, 11)]
    assert driver.torque_enabled is False


def test_read_state_uses_one_sync_transaction_and_reads_both_registers(
    driver_factory, fake_sdk
):
    driver = driver_factory()
    driver.open()
    fake_sdk.events.clear()

    positions, velocities = driver.read_state()

    assert len(positions) == len(IDS)
    assert len(velocities) == len(IDS)
    assert fake_sdk.events == [
        ("read.txRxPacket",),
        ("read.isAvailable", 1, 128, 4),
        ("read.getData", 1, 128, 4),
        ("read.isAvailable", 1, 132, 4),
        ("read.getData", 1, 132, 4),
        ("read.isAvailable", 2, 128, 4),
        ("read.getData", 2, 128, 4),
        ("read.isAvailable", 2, 132, 4),
        ("read.getData", 2, 132, 4),
    ]


def test_read_state_retries_a_failed_sync_transaction(driver_factory, fake_sdk):
    driver = driver_factory()
    driver.open()
    fake_sdk.events.clear()
    fake_sdk.read_tx_results.extend((-1001, COMM_SUCCESS))

    driver.read_state()

    assert fake_sdk.events[:2] == [
        ("read.txRxPacket",),
        ("read.txRxPacket",),
    ]


def test_read_state_stops_after_bounded_retries(driver_factory, fake_sdk):
    driver = driver_factory()
    driver.open()
    fake_sdk.events.clear()
    fake_sdk.read_tx_results.extend((-1001, -1001, -1001, COMM_SUCCESS))

    with pytest.raises(driver_module.DynamixelSdkError, match="communication error"):
        driver.read_state()

    assert fake_sdk.events == [("read.txRxPacket",)] * 3


def test_read_state_decodes_signed_registers_and_converts_units(
    driver_factory, fake_sdk
):
    driver = driver_factory()
    driver.open()
    fake_sdk.read_data.update(
        {
            (1, 128, 4): 0xFFFFFFFF,
            (1, 132, 4): 2048,
            (2, 128, 4): 10,
            (2, 132, 4): 0xFFFFF800,
        }
    )

    positions, velocities = driver.read_state()

    assert positions.tolist() == pytest.approx([math.pi, -math.pi])
    velocity_scale = 0.229 * 2.0 * math.pi / 60.0
    assert velocities.tolist() == pytest.approx([-velocity_scale, 10 * velocity_scale])


def test_write_currents_sync_writes_every_id_and_always_clears_params(
    driver_factory, fake_sdk
):
    driver = driver_factory()
    driver.open()
    fake_sdk.events.clear()

    driver.write_currents([0.0, 0.0])

    assert fake_sdk.events == [
        ("write.addParam", 1, [0, 0]),
        ("write.addParam", 2, [0, 0]),
        ("write.txPacket",),
        ("write.clearParam",),
    ]

    fake_sdk.events.clear()
    fake_sdk.write_tx_result = -1001
    with pytest.raises(Exception, match="write_currents|sync write"):
        driver.write_currents([0.0, 0.0])
    assert fake_sdk.events[-1] == ("write.clearParam",)


@pytest.mark.parametrize(
    "currents, error",
    [
        ([0.0], "2 values"),
        ([float("nan"), 0.0], "finite"),
        ([4000.0, 0.0], "current limit"),
        ([0.0, 1000.0], "current limit"),
    ],
)
def test_write_currents_validates_before_bus_access(
    driver_factory, fake_sdk, currents, error
):
    driver = driver_factory()
    driver.open()
    fake_sdk.events.clear()

    with pytest.raises(ValueError, match=error):
        driver.write_currents(currents)

    assert fake_sdk.events == []


def test_write_currents_clears_params_when_add_param_fails(
    driver_factory, fake_sdk
):
    driver = driver_factory()
    driver.open()
    fake_sdk.write_add_results[2] = False
    fake_sdk.events.clear()

    with pytest.raises(driver_module.DynamixelSdkError, match="device ID 2"):
        driver.write_currents([0.0, 0.0])

    assert fake_sdk.events[-1] == ("write.clearParam",)


def test_close_zeroes_current_disables_torque_then_closes_port(
    driver_factory, fake_sdk
):
    driver = driver_factory()
    driver.open()
    fake_sdk.events.clear()

    driver.close()

    assert fake_sdk.events == [
        ("write.addParam", 1, [0, 0]),
        ("write.addParam", 2, [0, 0]),
        ("write.txPacket",),
        ("write.clearParam",),
        ("write1", 1, 64, 0),
        ("write1", 2, 64, 0),
        ("closePort",),
    ]
    assert driver.torque_enabled is False


def test_close_still_disables_torque_and_closes_after_zero_write_failure(
    driver_factory, fake_sdk
):
    driver = driver_factory()
    driver.open()
    fake_sdk.write_tx_result = -1001
    fake_sdk.events.clear()

    with pytest.raises(driver_module.DynamixelSdkError, match="sync write"):
        driver.close()

    assert ("write1", 1, 64, 0) in fake_sdk.events
    assert ("write1", 2, 64, 0) in fake_sdk.events
    assert fake_sdk.events[-1] == ("closePort",)
    assert driver.is_open is False


def test_sdk_error_reports_operation_and_device_id(driver_factory, fake_sdk):
    fake_sdk.ping_results[2] = (0, -1001, 0)
    driver = driver_factory()

    with pytest.raises(Exception) as caught:
        driver.open()

    message = str(caught.value).lower()
    assert "ping" in message
    assert "2" in message
    assert "communication error -1001" in message
    assert fake_sdk.events[-1] == ("closePort",)
