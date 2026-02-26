# optimize.py
import os
import time
import numpy as np
from scipy.optimize import minimize
from impact_slip_sim import sim_cost

x0 = np.array([1.10e+05, 1.362e+02, 1.77e+03, 1.787e+00])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]

mass = 5 # kg

best = {"x": None, "f": np.inf}

powell = False
max_iter = 200

count = 0

test_params = {
    "duration": 1.0,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.5,
    "slope": 40,
}

def run_cost(x):
    global count 
    global start_time

    knee_k = x[0]
    ankle_k = x[1]

    knee_d = x[2] # 2*np.sqrt(knee_k*mass)   # critcally damped
    ankle_d = x[3] # 2*np.sqrt(ankle_k*mass) # crtically damped

    

    joint_params = {
        "rail":  dict(A="base", B="prismatic_coupler",
                    k=0.0, x0=0.00, d=30.50),

        "thigh": dict(A="prismatic_coupler", B="upper_leg",
                    k=0.0, x0=0.00, d=0.00),

        "knee":  dict(A="upper_leg", B="lower_leg",
                    k=knee_k, x0=0.40, d=knee_d),

        "ankle": dict(k=ankle_k, q0=0.00, d=ankle_d),
    }

    pos, vel = sim_cost(joint_params=joint_params, test_params=test_params)

    f=pos[1]

    if powell and np.isfinite(f) and f < best["f"]:
        best["f"] = f
        best["x"] = np.array(x, copy=True)
        print("NEW BEST:", best["f"], "at", best["x"])

    count = count + 1
    end_time = time.perf_counter()
    duration = end_time - start_time
    frac_done = count/max_iter
    time_left = (duration/frac_done) - duration

    print(f"x: {x}    cost: {f}    %: {int(frac_done*100)} tleft: {int(time_left)}")

    return f

start_time = time.perf_counter()
if not powell:
    res = minimize(run_cost, x0, method="Nelder-Mead",
                options={"maxiter": max_iter, "xatol": 1e-3, "fatol": 1e-6})
else:
    res = minimize(lambda x: run_cost(x),
                x0,
                method="Powell",     # good derivative-free baseline
                bounds=bounds,
                options={"maxiter": max_iter})
    print("best seen:", best["x"], best["f"])

print("#### RESULTS ####")
print("scipy returned:", res.x, res.fun)

duration = 2  # seconds
freq = 1040  # Hz
os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))

def proceed_check():
    while True:
        # .lower() handles "Y" or "y"
        choice = input("Do you want to proceed? (y/n): ").lower().strip()
        
        if choice == 'y':
            return True
        elif choice == 'n':
            print("Operation cancelled.")
            return False
        else:
            print("Invalid input. Please enter 'y' or 'n'.")

# Usage
if proceed_check():
    result_x = res.x
    knee_k = result_x[0]
    ankle_k = result_x[1]
    knee_d = result_x[2]
    ankle_d = result_x[3]

    joint_params = {
        "rail":  dict(A="base", B="prismatic_coupler",
                    k=0.0, x0=0.00, d=30.50),

        "thigh": dict(A="prismatic_coupler", B="upper_leg",
                    k=0.0, x0=0.00, d=0.00),

        "knee":  dict(A="upper_leg", B="lower_leg",
                    k=knee_k, x0=0.40, d=knee_d),

        "ankle": dict(k=ankle_k, q0=0.00, d=ankle_d),
    }

    sim_cost(joint_params=joint_params, test_params=test_params, vis=True)