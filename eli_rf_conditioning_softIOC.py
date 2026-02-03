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
# Linea1 "W1DSIP01","W1GSIP01","W1KSIP01","W1KSIP02","W1KSIP03","GUNSIP00","GUNSIP01","GUNSIP02","RFDSIP01"
# pv_laser_amp_llrf=config.get('pv_laser_amp_llrf')
# prefix_redpitaya02=config.get('prefix_redpitaya02')
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
        elif item['type']=="stringIn":
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
    
    # Init variables
    loop_period=0.1 # Pause in seconds between iterations
    print(vacuum_pumps.get())
    vacuum_over_tsh = False
    #print(caget(prefix_pumps.get()[0]+vacuum_pumps.get()[0]+suffix_pumps.get()[0]))
    #print(caget(prefix_pumps.get()[1]+vacuum_pumps.get()[1]+suffix_pumps.get()[1]))
    # Starting main loop
    while True:

        # WORKS WITH FEEDBACK ALWAYS ON
        # DEFINE INITIAL SETPIOINT
        if caget(prefix_LLRF+":app:rf_ctrl") == False and conditioning_status.get()==True:
            #conditioning_status.set(False)
            #counter_intlk.set(0)
            # c'è anche counter_intlk_tsh che viene creata nel json ma non più usata
            raise_count.set(0)
        else:
            pass
        if power_raise_status.get()==0 and raise_count.get()!=0: # raise_count increase only when power_raise_status=1
            raise_count.set(0)

        # Check vacuum thresholds and perform conditioning actions
        if conditioning_status.get() == True:
            for i in range(len(vacuum_pumps.get())):
                if caget(prefix_pumps.get()[i]+vacuum_pumps.get()[i]+suffix_pumps.get()[i])>vacuum_tsh.get()[i]:
                    vac_id.set(i+1) # at least one pump exceded tsh
                    break
                else: vac_id.set(0)

            # First intlk event above threshold, turn off RF and save info
            if vac_id.get()!=0 and vacuum_over_tsh==False:
                power_raise_status.set(0)
                llrf_fbk_level=caget(prefix_LLRF+":vm:dsp:sp_amp:power")
                caput(prefix_LLRF+":app:rf_ctrl", "0")
                conditioning_setpoint.set(llrf_fbk_level*1e-6*intlk_hysteresis.get())
                last_intlk_source.set(vacuum_pumps.get()[i]) # stores last intlk source
                last_intlk_datetime.set(str(time.time())) #store last intlk time
                vacuum_over_tsh = True

            # All pumps below threshold after an intlk event, restart reduced power
            elif vac_id.get() == 0 and vacuum_over_tsh == True:
                power_raise_status.set(1)
                caput(prefix_LLRF + ":vm:dsp:sp_amp:power", str(conditioning_setpoint.get() * 1e6)) # set power
                caput(prefix_LLRF+":app:rf_ctrl", "1") # RF ON
                caput(prefix_LLRF+":vm:dsp:pi_amp:loop_closed", "1") # AMP FBK ON
                vacuum_over_tsh = False 

            # All pumps below threshold, increase the power
            elif vac_id.get() == 0 and vacuum_over_tsh == False:
                pass # No action, but included for completeness

            # Power raise logic
            if power_raise_status.get() == True and caget(prefix_LLRF+":vm:dsp:sp_amp:power")<=(conditioning_target.get()*1e6): # power increase
                if raise_count.get() % (power_raise_wait.get()*600) == 0 and raise_count.get()!=0 : # the time interval for power increase is set in minutes  
                    llrf_fbk_level=caget(prefix_LLRF+":vm:dsp:sp_amp:power")
                    caput(prefix_LLRF+":vm:dsp:sp_amp:power", str(llrf_fbk_level+power_raise_step.get()*1e3))
                    raise_count.set(0)

                raise_count.set(raise_count.get()+1)
 
            #     if counter_intlk.get() == 0: # first interlock of the series
            #         llrf_fbk_level=caget(prefix_LLRF+":vm:dsp:sp_amp:power")
            #         caput(prefix_LLRF+":vm:dsp:sp_amp:power", str(0)) # feedback setpoint = 0
            #         conditioning_setpoint.set(llrf_fbk_level*1e-6*intlk_hysteresis.get())# conditioning setpoint = llrf setpoint * hysteresis
            #         counter_intlk.set(counter_intlk.get()+1) # increase interlock counter
            #         last_intlk_source.set(vacuum_pumps.get()[i]) # stores last intlk source
            #         last_intlk_datetime.set(str(time.time())) #store last intlk time
            #     elif counter_intlk.get() >= counter_intlk_tsh.get(): #check if the consecutive interlocks exceed the threshold
            #         caput(prefix_LLRF+":app:rf_ctrl", "0") # turn OFF RF
            #         counter_intlk.set(0)
            #         caput(prefix_LLRF+":vm:dsp:sp_amp:power", str(conditioning_setpoint.get()*1e6))  # to verify
            #     elif counter_intlk.get() != 0:
            #         counter_intlk.set(counter_intlk.get()+1) # increase interlock counter
            # elif counter_intlk.get() != 0:  # vacuum has re-entered safe zone
            #     caput(prefix_LLRF+":vm:dsp:sp_amp:power", str(conditioning_setpoint.get()*1e6)) # feedback setpoint = conditioning setpoint (previously decrease by hysteresis)
            #     counter_intlk.set(0)
            #     raise_count.set(0)
              
            # elif power_raise_status.get() == True and caget(prefix_LLRF+":vm:dsp:sp_amp:power")<=(conditioning_target.get()*1e6): # power increase
            #     if raise_count.get() % (power_raise_wait.get()*60) == 0 and raise_count.get()!=0 : # the time interval for power increase is set in minutes  
            #         llrf_fbk_level=caget(prefix_LLRF+":vm:dsp:sp_amp:power")
            #         caput(prefix_LLRF+":vm:dsp:sp_amp:power", str(llrf_fbk_level+power_raise_step.get()*1e3))
            #         raise_count.set(0)

            #     raise_count.set(raise_count.get()+1)
         
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
