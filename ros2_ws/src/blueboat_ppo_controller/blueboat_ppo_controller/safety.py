import math


def autopilot_status_reason(system_status):
    """Return a diagnostic for a connected FC that is not standby/active."""
    if system_status in (3, 4):
        return None
    names = {
        0: 'uninitialized',
        1: 'booting',
        2: 'calibrating',
        5: 'critical',
        6: 'emergency',
        7: 'poweroff',
        8: 'flight_termination',
    }
    return 'autopilot_' + names.get(system_status, 'system_status_' + str(system_status))


class SafetyGate:
    def __init__(self, mode='shadow', telemetry_timeout=.2, command_timeout=.5, watchdog_timeout=.1):
        if mode not in ('shadow', 'actuator'):
            raise ValueError('Mode must be shadow or actuator')
        if not all(math.isfinite(v) and 0 < v <= 1 for v in (telemetry_timeout, command_timeout, watchdog_timeout)):
            raise ValueError('Invalid safety timeout')
        self.mode = mode
        self.telemetry_timeout = telemetry_timeout
        self.command_timeout = command_timeout
        self.watchdog_timeout = watchdog_timeout
        self.last_loop = None
        self.active = False
        self.fault = None
        self.closed = False


    def check(self, now, receipts, connected, armed, vehicle_mode, enabled, rc_enabled,
              parameters_valid, authority_valid=None, system_status=None):
        if self.closed:
            return 'shutdown'
        if self.fault:
            return self.fault
        for name in ('odometry', 'imu'):
            age = now - receipts.get(name, -math.inf)
            if not 0 <= age <= self.telemetry_timeout:
                return name + '_timeout'
        if not 0 <= now - receipts.get('command', -math.inf) <= self.command_timeout:
            return 'command_timeout'
        if not connected or not 0 <= now - receipts.get('state', -math.inf) <= 2.:
            return 'autopilot_disconnected'
        if system_status is not None:
            reason = autopilot_status_reason(system_status)
            if reason:
                return reason
        if self.mode == 'actuator':
            if not armed or vehicle_mode != 'MANUAL':
                return 'manual_takeover_or_disarmed'
            if not enabled or not 0 <= now - receipts.get('enable', -math.inf) <= .5:
                return 'enable_released'
            if authority_valid is None:
                if not rc_enabled or not 0 <= now - receipts.get('rc', -math.inf) <= .5:
                    return 'manual_takeover'
            elif not authority_valid:
                return 'gcs_authority_lost'
            if not parameters_valid or not 0 <= now - receipts.get('parameters', -math.inf) <= 15.:
                return 'parameters_unverified'
        return None


    def trip(self, reason):
        self.fault = reason
        self.active = False


    def overdue(self, now):
        return self.last_loop is not None and now - self.last_loop > self.watchdog_timeout


    def shutdown(self):
        self.closed = True
        self.active = False
