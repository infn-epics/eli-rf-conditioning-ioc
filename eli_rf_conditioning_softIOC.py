# Import the basic framework components.

# import logging
# import threading
# import time
from softioc import softioc, builder
#import numpy as np
import json
import argparse
import cothread
from epics import caget, caput, cainfo, camonitor
import time

# Read config file
parser = argparse.ArgumentParser()

parser.add_argument("-c", "--conf", required = True ,default = "config.json", help = "the configuration name")
parser.add_argument("-p", "--pvout", required=False, default="pvlist.txt", help="Output PVlist")

args = parser.parse_args()
print(f"* opening {args.conf}")
with open(args.conf, 'r') as f:
    config = json.load(f)
    
# Load prefixes from config file
prefix_rf_conditioning=config.get('prefix_rf_conditioning')
prefix_LLRF=config.get('prefix_LLRF')
print("\nList of epics prefixes:")
print(prefix_rf_conditioning)
print(prefix_LLRF)

# Set the record prefix
builder.SetDeviceName(prefix_rf_conditioning)

#Create IOC PVs and global variables (lowercase characters respect to the relavie PV)
for item in config.get('pv_to_build', []):
        func = getattr(builder, item['type'])
        if item['type']=="boolIn" or item['type']=="boolOut":
            globals()[item['pvname'].lower()]=func(item['pvname'], initial_value=int(item['init_value']))
        elif item['type']=="WaveformOut":
            globals()[item['pvname'].lower()]=func(item['pvname'], initial_value=list(item['init_value']))
        elif item['type'] in ["stringIn", "stringOut"]:
            globals()[item['pvname'].lower()]=func(item['pvname'], initial_value=str(item['init_value']))
        else:
            globals()[item['pvname'].lower()]=func(item['pvname'], initial_value=float(item['init_value']))
print("List of the global definitions:")
print(dir(),"\n")


# Initialize variables

# Boilerplate get the IOC started
builder.LoadDatabase()
softioc.iocInit()

# Start processes required to be run after iocInit

def main(prefix_rf_conditioning: str, prefix_LLRF: str):
    loop_period = 0.1
    vacuum_over_tsh = False

    while True:

        if caget(prefix_LLRF + ":app:rf_ctrl") == False and conditioning_status.get() == True:
            raise_count.set(0)
        else:
            pass

        if power_raise_status.get() == 0 and raise_count.get() != 0:
            raise_count.set(0)

        if conditioning_status.get() == True:

            for i in range(len(vacuum_pumps.get())):
                if caget(prefix_pumps.get()[i] + vacuum_pumps.get()[i] + suffix_pumps.get()[i]) > vacuum_tsh.get()[i]:
                    vac_id.set(i + 1)
                    break
                else:
                    vac_id.set(0)

            if vac_id.get() != 0 and vacuum_over_tsh == False:
                power_raise_status.set(0)
                llrf_fbk_level = caget(prefix_LLRF + ":vm:dsp:sp_amp:power")
                caput(prefix_LLRF + ":app:rf_ctrl", "0")
                conditioning_setpoint.set(llrf_fbk_level * 1e-6 * intlk_hysteresis.get())
                last_intlk_source.set(vacuum_pumps.get()[i])
                last_intlk_datetime.set(str(time.time()))
                vacuum_over_tsh = True

            elif vac_id.get() == 0 and vacuum_over_tsh == True:
                power_raise_status.set(1)
                caput(prefix_LLRF + ":vm:dsp:sp_amp:power", str(conditioning_setpoint.get() * 1e6))
                caput(prefix_LLRF + ":app:rf_ctrl", "1")
                caput(prefix_LLRF + ":vm:dsp:pi_amp:loop_closed", "1")
                vacuum_over_tsh = False

            elif vac_id.get() == 0 and vacuum_over_tsh == False:
                pass

            if power_raise_status.get() == True and caget(prefix_LLRF + ":vm:dsp:sp_amp:power") <= (conditioning_target.get() * 1e6):
                if raise_count.get() % (power_raise_wait.get() * 600) == 0 and raise_count.get() != 0:
                    llrf_fbk_level = caget(prefix_LLRF + ":vm:dsp:sp_amp:power")
                    caput(prefix_LLRF + ":vm:dsp:sp_amp:power", str(llrf_fbk_level + power_raise_step.get() * 1e3))
                    raise_count.set(0)
                raise_count.set(raise_count.get() + 1)

        # ===============================
        # Pulse-to-pulse waveform monitor
        # ===============================

            wf_pv_name = prefix_LLRF + ":" + wf_source_suffix.get()
            wf_time_pv = prefix_LLRF + ":app:time_vector"

            if wf_pulse_intlk_enable.get() == 0:
                wf_interlock.set(0)
                cothread.Sleep(loop_period)
                continue

            wf_curr = caget(wf_pv_name)
            time_vec = caget(wf_time_pv)

            if wf_prev_valid.get() == 0:
                wf_pulse_prev.set(list(wf_curr))
                wf_prev_valid.set(1)
                wf_interlock.set(0)
                cothread.Sleep(loop_period)
                continue

            t_start = wf_offset_curs_us.get()
            t_stop = t_start + wf_duration_curs_us.get()
            i_start = None
            i_stop = None

            for i, t in enumerate(time_vec):
                if i_start is None and t >= t_start:
                    i_start = i
                if t >= t_stop:
                    i_stop = i
                    break

            if i_start is None or i_stop is None or i_stop <= i_start:
                wf_interlock.set(0)
                cothread.Sleep(loop_period)
                continue

            wf_prev = wf_pulse_prev.get()
            tol = wf_mask_percent.get() / 100.0
            fault = False

            for i in range(i_start, i_stop):
                ref = wf_prev[i]
                low = ref * (1 - tol)
                high = ref * (1 + tol)
                if wf_curr[i] < low or wf_curr[i] > high:
                    fault = True
                    break

            if not fault:
                wf_pulse_prev.set(list(wf_curr))

            wf_interlock.set(1 if fault else 0)

            # ===============================
            # Combine vacuum and waveform interlocks
            # ===============================

            interlock_active = vac_id.get() != 0 or wf_interlock.get() == 1

            if interlock_active and vacuum_over_tsh == False:
                power_raise_status.set(0)
                llrf_fbk_level = caget(prefix_LLRF + ":vm:dsp:sp_amp:power")
                caput(prefix_LLRF + ":app:rf_ctrl", "0")
                conditioning_setpoint.set(llrf_fbk_level * 1e-6 * intlk_hysteresis.get())

                if vac_id.get() != 0:
                    last_intlk_source.set(vacuum_pumps.get()[vac_id.get() - 1])
                elif wf_interlock.get() == 1:
                    last_intlk_source.set("WF_PULSE_TO_PULSE")

                last_intlk_datetime.set(str(time.time()))
                vacuum_over_tsh = True
    
            # Pause for loop period
            cothread.Sleep(loop_period)

## dump pvs
softioc.dbl()

import os
with open(args.pvout, "w") as f:
        old_stdout = os.dup(1)
        os.dup2(f.fileno(), 1)
        softioc.dbl()
        os.dup2(old_stdout, 1)
        os.close(old_stdout)
# Launch threads
cothread.Spawn(main(prefix_rf_conditioning, prefix_LLRF))

# Leave the IOC running with an interactive shell.
softioc.interactive_ioc(globals())
