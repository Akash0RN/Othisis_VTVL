#---------------------
#This code is for simulating the responses; initially created for dynamic gain tuning ; 
#---------------------


import numpy as np
import matplotlib.pyplot as plt

# Gains:                       //Tuning? 
KP = 1.0
KI = 0.1
KD = 0.01

SIM_TIME = 50.0    
STEP_TIME = 10.0   
TIME_STEP = 0.1    
SETPOINT = 10.0
ERRORS = [5.0, 11.0, 8.0]

def run_pid_simulation(initerr):
    
    times = np.arange(0, SIM_TIME, TIME_STEP)
    pv = np.zeros(len(times) + 1) 
    error = np.zeros_like(times)
    p_term = np.zeros_like(times)
    i_term = np.zeros_like(times)
    d_term = np.zeros_like(times)
    output = np.zeros_like(times)
    
    inte = 0.0
    preverr = initerr  

    for i, t in enumerate(times):
        setpoint = SETPOINT if t >= STEP_TIME else 0.0 
        error[i] = setpoint - pv[i]
        p_term[i] = KP * error[i]
        inte += error[i] * TIME_STEP
        i_term[i] = KI * inte
        d_term[i] = KD * (error[i] - preverr) / TIME_STEP if i > 0 else 0
        output[i] = p_term[i] + i_term[i] + d_term[i]
        pv[i + 1] = pv[i] + output[i] * TIME_STEP
        preverr = error[i]

    return times, pv[:-1], error, p_term, i_term, d_term, output

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 12))

for initial_error in ERRORS:
    times, pv, error, p_term, i_term, d_term, output = run_pid_simulation(initial_error)

    ax1.plot(times, pv, label=f'PV (Err {initial_error})')
    ax1.plot(times, error, linestyle='dashed', alpha=0.7, label=f'Error (Err {initial_error})')

    ax2.plot(times, p_term, label=f'P-Term (Err {initial_error})', linestyle='dotted')
    ax2.plot(times, i_term, label=f'I-Term (Err {initial_error})', linestyle='dashed')
    ax2.plot(times, d_term, label=f'D-Term (Err {initial_error})', linestyle='dashdot')
ax1.axhline(y=SETPOINT, color='black', linestyle='--', label='Setpoint')
ax1.legend()
ax2.legend()
ax1.grid()
ax2.grid()

plt.tight_layout()
plt.show()
