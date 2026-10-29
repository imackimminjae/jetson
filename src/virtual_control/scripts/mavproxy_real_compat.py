#!/usr/bin/env python3
"""Local MAVProxy compatibility fixes; never initiates vehicle commands."""
import re
import runpy
import shutil
import sys


def actuator_success(text):
    return bool(re.match(
        r'^Got MAVLink msg: COMMAND_ACK \{command\s*:\s*187,\s*result\s*:\s*0,',
        str(text))) or str(text) == 'Got COMMAND_ACK: DO_SET_ACTUATOR: ACCEPTED'


def install_compat():
    from pymavlink import mavutil
    from MAVProxy.modules.lib import textconsole
    from MAVProxy.modules import mavproxy_link

    original_write = textconsole.SimpleConsole.writeln

    def write(self, text, *args, **kwargs):
        if actuator_success(text):
            if not getattr(self, '_real_actuator_ack_seen', False):
                self._real_actuator_ack_seen = True
                original_write(self, '[ACK 187] Actuator accepted. Repeated successes hidden; '
                               'ARM response is command 400.', *args, **kwargs)
            return
        return original_write(self, text, *args, **kwargs)

    textconsole.SimpleConsole.writeln = write
    original_mode = mavutil.mavfile.set_mode

    def set_mode(self, mode, custom_mode=0, custom_sub_mode=0):
        # MAVProxy passes PX4 mode_mapping tuples; pymavlink expects three scalars.
        if (self.field('HEARTBEAT', 'autopilot', None) == mavutil.mavlink.MAV_AUTOPILOT_PX4
                and isinstance(mode, (tuple, list)) and len(mode) == 3):
            return self.set_mode_px4(*mode)
        return original_mode(self, mode, custom_mode, custom_sub_mode)

    mavutil.mavfile.set_mode = set_mode
    original_sent = mavproxy_link.LinkModule.master_send_callback

    def sent(self, message, master):
        result = original_sent(self, message, master)
        if message.get_type() == 'COMMAND_LONG' and message.command == 400:
            self.mpstate.console.writeln(
                '[ARM TX] target=%d/%d arm=%g param2=%g; wait for ACK 400 and ARMED heartbeat.'
                % (message.target_system, message.target_component,
                   message.param1, message.param2))
        return result

    mavproxy_link.LinkModule.master_send_callback = sent


def main():
    executable = shutil.which('mavproxy.py')
    if not executable:
        raise SystemExit('mavproxy.py not found in PATH')
    install_compat()
    sys.argv[0] = executable
    runpy.run_path(executable, run_name='__main__')


if __name__ == '__main__':
    main()
