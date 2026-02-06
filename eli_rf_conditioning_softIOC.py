# Import the basic framework components.
from softioc import softioc, builder
import json
import argparse
import cothread
from cothread.catools import caget, caput
import time
import os

# -----------------------------
# Read config file
# -----------------------------
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--conf", required=True, default="config.json", help="the configuration name")
parser.add_argument("-p", "--pvout", required=False, default="pvlist.txt", help="Output PVlist")
args = parser.parse_args()

print(f"* opening {args.conf}")
with open(args.conf, 'r') as f:
    config = json.load(f)

# -----------------------------
# Load prefixes
# -----------------------------
prefix_rf_conditioning = config.get('prefix_rf_conditioning')
prefix_LLRF = config.get('prefix_LLRF')

print("\nList of epics prefixes:")
print(prefix_rf_conditioning)
print(prefix_LLRF)

# -----------------------------
# Create IOC PVs
# -----------------------------
builder.SetDeviceName(prefix_rf_conditioning)

for item in config.get('pv_to_build', []):
    func = getattr(builder, item['type'])
    if item['type'] in ["boolIn", "boolOut"]:
        globals()[item['pvname'].lower()] = func(item['pvname'], initial_value=int(item['init_value']))
    elif item['type'] == "WaveformOut":
        init_val = item['init_value']
        if isinstance(init_val, str):
            init_val = eval(init_val)
        globals()[item['pvname'].lower()] = func(item['pvname'], initial_value=list(init_val))
    elif item['type'] in ["stringIn", "stringOut"]:
        globals()[item['pvname'].lower()] = func(item['pvname'], initial_value=str(item['init_value']))
    else:
        globals()[item['pvname'].lower()] = func(item['pvname'], initial_value=float(item['init_value']))

# -----------------------------
# Start IOC
# -----------------------------
builder.LoadDatabase()
softioc.iocInit()

# -----------------------------
# Main loop
# -----------------------------
def main(prefix_rf_conditioning: str, prefix_LLRF: str):

    loop_period = 0.1
    vacuum_over_tsh = False

    while True:

        # Reset raise counter if RF is off
        if caget(prefix_LLRF + ":app:rf_ctrl") == False and conditioning_status.get():
            raise_count.set(0)

        if power_raise_status.get() == 0 and raise_count.get() != 0:
            raise_count.set(0)

        # =====================================
        # Conditioning logic (vacuum + power raise)
        # =====================================
        if conditioning_status.get():

            for i in range(len(vacuum_pumps.get())):
                pv_val = caget(prefix_pumps.get()[i] +
                               vacuum_pumps.get()[i] +
                               suffix_pumps.get()[i])
                if pv_val > vacuum_tsh.get()[i]:
                    vac_id.set(i + 1)
                    break
                else:
                    vac_id.set(0)

            if vac_id.get() != 0 and not vacuum_over_tsh:
                power_raise_status.set(0)
                llrf_fbk_level = caget(prefix_LLRF + ":vm:dsp:sp_amp:power")
                caput(prefix_LLRF + ":app:rf_ctrl", "0")
                conditioning_setpoint.set(llrf_fbk_level * 1e-6 * intlk_hysteresis.get())
                last_intlk_source.set(vacuum_pumps.get()[vac_id.get() - 1])
                last_intlk_datetime.set(str(time.time()))
                vacuum_over_tsh = True

            elif vac_id.get() == 0 and vacuum_over_tsh:
                power_raise_status.set(1)
                caput(prefix_LLRF + ":vm:dsp:sp_amp:power",
                      str(conditioning_setpoint.get() * 1e6))
                caput(prefix_LLRF + ":app:rf_ctrl", "1")
                caput(prefix_LLRF + ":vm:dsp:pi_amp:loop_closed", "1")
                vacuum_over_tsh = False

            if power_raise_status.get() and \
               caget(prefix_LLRF + ":vm:dsp:sp_amp:power") <= conditioning_target.get() * 1e6:

                if raise_count.get() % (power_raise_wait.get() * 600) == 0 and raise_count.get() != 0:
                    llrf_fbk_level = caget(prefix_LLRF + ":vm:dsp:sp_amp:power")
                    caput(prefix_LLRF + ":vm:dsp:sp_amp:power",
                          str(llrf_fbk_level + power_raise_step.get() * 1e3))
                    raise_count.set(0)

                raise_count.set(raise_count.get() + 1)

        # =====================================
        # Pulse-to-pulse waveform interlock
        # =====================================
        if conditioning_status.get() and wf_pulse_intlk_enable.get():

            wf_curr = list(caget(wf_source_suffix.get()))
            time_vec = list(caget(prefix_LLRF + ":app:time_vector"))
            tol = wf_mask_percent.get() / 100.0

            if len(wf_curr) != len(time_vec):
                cothread.Sleep(loop_period)
                continue

            # First pulse
            if wf_prev_valid.get() == 0:
                wf_pulse_prev.set(list(wf_curr))
                wf_prev_valid.set(1)

                wf_mask_low.set([v * (1 - tol) for v in wf_curr])
                wf_mask_high.set([v * (1 + tol) for v in wf_curr])

                wf_interlock.set(0)
                cothread.Sleep(loop_period)
                continue

            wf_prev = wf_pulse_prev.get()

            # --- cursors in microseconds ---
            t0 = wf_offset_curs_us.get()
            t1 = t0 + wf_duration_curs_us.get()

            # find index window
            start_idx = 0
            end_idx = len(time_vec) - 1

            for i, t in enumerate(time_vec):
                if t >= t0:
                    start_idx = i
                    break

            for i in range(start_idx, len(time_vec)):
                if time_vec[i] > t1:
                    end_idx = i - 1
                    break

            fault = False

            for i in range(start_idx, end_idx + 1):
                low = wf_prev[i] * (1 - tol)
                high = wf_prev[i] * (1 + tol)
                if wf_curr[i] < low or wf_curr[i] > high:
                    fault = True
                    break

            # update previous pulse ALWAYS
            wf_pulse_prev.set(list(wf_curr))

            # update mask display (full waveform)
            wf_mask_low.set([v * (1 - tol) for v in wf_curr])
            wf_mask_high.set([v * (1 + tol) for v in wf_curr])

            wf_interlock.set(1 if fault else 0)

            # =====================================
            # Combine interlocks
            # =====================================
            if wf_interlock.get() == 1 or vac_id.get() != 0:
                power_raise_status.set(0)
                llrf_fbk_level = caget(prefix_LLRF + ":vm:dsp:sp_amp:power")
                caput(prefix_LLRF + ":app:rf_ctrl", "0")
                conditioning_setpoint.set(llrf_fbk_level * 1e-6 * intlk_hysteresis.get())

                last_intlk_source.set(
                    vacuum_pumps.get()[vac_id.get() - 1]
                    if vac_id.get() != 0 else "WF_PULSE_TO_PULSE"
                )
                last_intlk_datetime.set(str(time.time()))
                vacuum_over_tsh = True

        cothread.Sleep(loop_period)

# dump PVs
softioc.dbl()

with open(args.pvout, "w") as f:
    old_stdout = os.dup(1)
    os.dup2(f.fileno(), 1)
    softioc.dbl()
    os.dup2(old_stdout, 1)
    os.close(old_stdout)

# Launch threads
cothread.Spawn(main, prefix_rf_conditioning, prefix_LLRF)

# Leave IOC running
softioc.interactive_ioc(globals())
