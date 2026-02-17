from softioc import softioc, builder
import json
import argparse
import cothread
from cothread.catools import caget, caput
import time
import os

# ============================================================================
# Configuration Loading
# ============================================================================
parser = argparse.ArgumentParser()
parser.add_argument("-c", "--conf", required=False, default="config.json")
parser.add_argument("-p", "--pvout", required=False, default="pvlist.txt")
args = parser.parse_args()

with open(args.conf, 'r') as f:
    config = json.load(f)

prefix_rf_conditioning = config.get('prefix_rf_conditioning')
prefix_LLRF = config.get('prefix_LLRF')

# ============================================================================
# PV Creation
# ============================================================================
builder.SetDeviceName(prefix_rf_conditioning)

for item in config.get('pv_to_build', []):
    func = getattr(builder, item['type'])
    pv_type = item['type']
    pv_name = item['pvname']
    init_value = item['init_value']

    if pv_type in ["boolIn", "boolOut"]:
        globals()[pv_name.lower()] = func(pv_name, initial_value=int(init_value))
    elif pv_type in ["WaveformIn", "WaveformOut"]:
        if isinstance(init_value, str):
            init_value = eval(init_value)
        globals()[pv_name.lower()] = func(pv_name, initial_value=list(init_value))
    elif pv_type in ["stringIn", "stringOut"]:
        globals()[pv_name.lower()] = func(pv_name, initial_value=str(init_value))
    else:
        globals()[pv_name.lower()] = func(pv_name, initial_value=float(init_value))

builder.LoadDatabase()
softioc.iocInit()

# ============================================================================
# Main Control Loop
# ============================================================================
def main(prefix_rf_conditioning: str, prefix_LLRF: str):

    loop_period = 0.1
    vacuum_over_tsh = False
    last_wf_sel = None
    wf_intlk_holdoff_s = 10.0
    wf_intlk_time = None

    while True:

        # --- Waveform interlock hold-off timer management
        if wf_intlk_time is not None:
            elapsed = time.time() - wf_intlk_time
            if elapsed < wf_intlk_holdoff_s or vac_id.get() != 0:
                cothread.Sleep(loop_period)
                continue
            else:
                wf_intlk_time = None

        # --- Reset power raise counter when RF is disabled
        if not conditioning_status.get():
            raise_count.set(0)
            cothread.Sleep(loop_period)
            continue

        if power_raise_status.get() == 0 and raise_count.get() != 0:
            raise_count.set(0)

        # ====================================================================
        # Vacuum Interlock and Power Control
        # ====================================================================
        if conditioning_status.get():

            wf_min_valid_amp = caget(prefix_LLRF + ":vm:dsp:sp_amp:power") * 0.5

            # Check vacuum status across all pump stations
            vac_id_trigger = 0
            vac_id_reenable = 0

            for i in range(len(vacuum_pumps.get())):
                pv_val = caget(
                    prefix_pumps.get()[i] + vacuum_pumps.get()[i] + suffix_pumps.get()[i]
                )
                if pv_val > vacuum_tsh.get()[i]:
                    vac_id_trigger = i + 1
                elif pv_val > vacuum_tsh.get()[i] * vac_threshold_reenable.get():
                    vac_id_reenable = i + 1

            # Trigger vacuum interlock when pressure exceeds threshold
            if vac_id_trigger != 0 and not vacuum_over_tsh:
                power_raise_status.set(0)
                llrf_fbk_level = caget(prefix_LLRF + ":vm:dsp:sp_amp:power")
                caput(prefix_LLRF + ":app:rf_ctrl", "0")
                conditioning_setpoint.set(llrf_fbk_level * 1e-6 * intlk_hysteresis.get())
                last_intlk_source.set(vacuum_pumps.get()[vac_id_trigger - 1])
                last_intlk_datetime.set(str(time.time()))

                # Reset waveform mask to prevent contamination of vacuum interlock record
                wf_prev_valid.set(0)
                wf_interlock.set(0)
                vacuum_over_tsh = True

            # Re-enable RF when pressure returns below hysteresis threshold
            elif vac_id_trigger == 0 and vacuum_over_tsh:
                if vac_id_reenable == 0:
                    power_raise_status.set(1)
                    caput(prefix_LLRF + ":vm:dsp:sp_amp:power", str(conditioning_setpoint.get() * 1e6))
                    caput(prefix_LLRF + ":app:rf_ctrl", "1")
                    caput(prefix_LLRF + ":vm:dsp:pi_amp:loop_closed", "1")
                    vacuum_over_tsh = False

            # --- Automatic power ramping during conditioning
            if (power_raise_status.get() and 
                caget(prefix_LLRF + ":vm:dsp:sp_amp:power") <= conditioning_target.get() * 1e6):
                
                if (raise_count.get() % (power_raise_wait.get() * 600) == 0 and raise_count.get() != 0):
                    llrf_fbk_level = caget(prefix_LLRF + ":vm:dsp:sp_amp:power")
                    caput(
                        prefix_LLRF + ":vm:dsp:sp_amp:power",
                        str(llrf_fbk_level + power_raise_step.get() * 1e3)
                    )
                    raise_count.set(0)

                raise_count.set(raise_count.get() + 1)

        # ====================================================================
        # Pulse-to-Pulse Waveform Interlock
        # ====================================================================
        if conditioning_status.get() and wf_pulse_intlk_enable.get():

            # Skip waveform processing if RF is disabled by vacuum interlock
            if vacuum_over_tsh:
                cothread.Sleep(loop_period)
                continue

            #wf_sources = wf_source_suffix.get()
            wf_sources = [prefix_LLRF + s for s in wf_source_suffix.get()]
            wf_sel = int(wf_source_sel.get())
            wf_refresh_pvs = [prefix_LLRF + s for s in wf_source_refresh_rate.get()]
            caput(wf_refresh_pvs[wf_sel], ".1 second")
            #caput(wf_source_refresh_rate.get()[wf_sel], ".1 second")

            if wf_sel < 0 or wf_sel >= len(wf_sources):
                cothread.Sleep(loop_period)
                continue

            if last_wf_sel is None:
                last_wf_sel = wf_sel

            # Reset mask when waveform source changes
            if wf_sel != last_wf_sel:
                wf_prev_valid.set(0)
                wf_interlock.set(0)
                last_wf_sel = wf_sel
                cothread.Sleep(loop_period)
                continue

            wf_pv_name = wf_sources[wf_sel]
            wf_curr = list(caget(wf_pv_name))
            time_vec = list(caget(prefix_LLRF + ":app:time_vector"))

            if len(wf_curr) != len(time_vec):
                cothread.Sleep(loop_period)
                continue

            # Extract region of interest based on time window parameters
            tol = wf_mask_percent.get() / 100.0
            t0 = wf_offset_curs_us.get()
            t1 = t0 + wf_duration_curs_us.get()
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

            sig_max = max(abs(wf_curr[i]) for i in range(start_idx, end_idx + 1))

            # Initial mask creation: store first valid pulse and define tolerance bands
            if wf_prev_valid.get() == 0:
                if sig_max < wf_min_valid_amp:
                    wf_interlock.set(0)
                    cothread.Sleep(loop_period)
                    continue

                wf_pulse_prev.set(list(wf_curr))
                wf_prev_valid.set(1)
                wf_mask_low.set([v * (1 - tol) for v in wf_curr])
                wf_mask_high.set([v * (1 + tol) for v in wf_curr])
                wf_interlock.set(0)
                cothread.Sleep(loop_period)
                continue

            # Compare current waveform against established mask
            wf_prev = wf_pulse_prev.get()
            fault = False
            for i in range(start_idx, end_idx + 1):
                low = wf_prev[i] * (1 - tol)
                high = wf_prev[i] * (1 + tol)
                if wf_curr[i] < low or wf_curr[i] > high:
                    fault = True
                    break

            # Update mask with current pulse for trending
            wf_pulse_prev.set(list(wf_curr))
            wf_mask_low.set([v * (1 - tol) for v in wf_curr])
            wf_mask_high.set([v * (1 + tol) for v in wf_curr])
            wf_interlock.set(1 if fault else 0)

            # Trigger interlock when waveform exceeds mask (only if vacuum is normal)
            if fault and not vacuum_over_tsh:
                wf_pulse_postmortem.set(list(wf_curr))
                wf_prev_valid.set(0)

                if wf_intlk_time is None:
                    wf_intlk_time = time.time()

                power_raise_status.set(0)
                llrf_fbk_level = caget(prefix_LLRF + ":vm:dsp:sp_amp:power")
                caput(prefix_LLRF + ":app:rf_ctrl", "0")
                conditioning_setpoint.set(llrf_fbk_level * 1e-6 * intlk_hysteresis.get())
                last_intlk_source.set("WF_PULSE_TO_PULSE")
                last_intlk_datetime.set(str(time.time()))
                vacuum_over_tsh = True

        cothread.Sleep(loop_period)

# ============================================================================
# IOC Startup
# ============================================================================
softioc.dbl()
with open(args.pvout, "w") as f:
    old_stdout = os.dup(1)
    os.dup2(f.fileno(), 1)
    softioc.dbl()
    os.dup2(old_stdout, 1)
    os.close(old_stdout)

cothread.Spawn(main, prefix_rf_conditioning, prefix_LLRF)
softioc.interactive_ioc(globals())
