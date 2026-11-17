import pytest

from blueboat_ppo_controller.safety import SafetyGate


def test_default_gate_rejects_telemetry_timeout_above_one_second():
    with pytest.raises(ValueError, match='Invalid safety timeout'):
        SafetyGate(telemetry_timeout=1.01)


def test_explicit_deployment_gate_accepts_bounded_low_rate_telemetry_timeout():
    gate = SafetyGate(
        telemetry_timeout=1.25,
        max_telemetry_timeout=1.5,
    )

    assert gate.telemetry_timeout == 1.25


def test_deployment_gate_rejects_telemetry_timeout_above_its_cap():
    with pytest.raises(ValueError, match='Invalid safety timeout'):
        SafetyGate(
            telemetry_timeout=1.51,
            max_telemetry_timeout=1.5,
        )


def test_extended_telemetry_cap_does_not_relax_command_timeout():
    with pytest.raises(ValueError, match='Invalid safety timeout'):
        SafetyGate(
            telemetry_timeout=1.25,
            command_timeout=1.01,
            max_telemetry_timeout=1.5,
        )
