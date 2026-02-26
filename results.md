# Optimisation Results

## Nealder-Mead | Terrain 40 | Velocity 2

### Optimisation Settings
```python
x0 = np.array([100000.0, 50.0, 3000.0, 3.0])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]
```
### Simulation Settings
```python
Rail Damping = 30.50

mass = 5 # kg

test_params = {
    "duration": 1.5,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.0,
    "slope": 40,
}
```
### Terrain Settings
```python
base_size=(1.5, 0.5, 0.05),
base_color=(0.5, 0.5, 0.5, 1.0),
top_color=(0.4, 0.4, 0.7, 1.0),
n=40,
top_box_x=0.01,
top_box_z=0.005,
```

### Simulation Results

| Component | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Knee** | Stiffness ($k$) | 85460 | N/m |
| **Knee** | Damping ($d$) | 3630 | Ns/m |
| **Ankle** | Stiffness ($k$) | 69.4 | N/rad |
| **Ankle** | Damping ($d$) | 2.03 | Ns/rad |

**Final Position:**
> **Y Pos:** 0.04695 m

---

## Powell | Terrain 40 | Velocity 2

### Optimisation Settings
```python
x0 = np.array([100000.0, 50.0, 3000.0, 3.0])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]
```
### Simulation Settings
```python
Rail Damping = 30.50

mass = 5 # kg

test_params = {
    "duration": 1.5,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.0,
    "slope": 40,
}
```
### Terrain Settings
```python
base_size=(1.5, 0.5, 0.05),
base_color=(0.5, 0.5, 0.5, 1.0),
top_color=(0.4, 0.4, 0.7, 1.0),
n=40,
top_box_x=0.01,
top_box_z=0.005,
```

### Simulation Results

| Component | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Knee** | Stiffness ($k$) | 235.9 | N/m |
| **Knee** | Damping ($d$) | 138.1 | Ns/m |
| **Ankle** | Stiffness ($k$) | 6.133 | N/rad |
| **Ankle** | Damping ($d$) | 38.22 | Ns/rad |

**Final Position:**
> **Y Pos:** 0.3284 m

---

## Nealder-Mead | Terrain 60 | Velocity 2.5

### Optimisation Settings
x0 was taken from the optimal result from result of **Terrain 60 | Velocity 2**
```python
x0 = np.array([8.54580366e+04, 6.93604482e+01, 3.62780675e+03, 2.02675029e+00])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]
```
### Simulation Settings
```python
Rail Damping = 30.50

mass = 5 # kg

test_params = {
    "duration": 1.0,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.5,
    "slope": 40,
}
```
### Terrain Settings
```python
base_size=(1.5, 0.5, 0.05),
base_color=(0.5, 0.5, 0.5, 1.0),
top_color=(0.4, 0.4, 0.7, 1.0),
n=60,
top_box_x=0.01,
top_box_z=0.005,
```

### Simulation Results

| Component | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Knee** | Stiffness ($k$) | 110000 | N/m |
| **Knee** | Damping ($d$) | 1770 | Ns/m |
| **Ankle** | Stiffness ($k$) | 136.2 | N/rad |
| **Ankle** | Damping ($d$) | 1.787 | Ns/rad |

**Final Position:**
> **Y Pos:** 0.03373 m

---

## Nealder-Mead | Terrain 60 | Velocity 2

### Optimisation Settings
x0 was taken from the optimal result from result of **Terrain 40**
```python
x0 = np.array([8.54580366e+04, 6.93604482e+01, 3.62780675e+03, 2.02675029e+00])  # initial guess
bounds = [(1, 150000), (1, 10000), (0.1, 5000), (0.1, 1000)]
```
### Simulation Settings
```python
Rail Damping = 30.50

mass = 5 # kg

test_params = {
    "duration": 1.5,
    "y_pos0": 0.645,
    "y_vel0": 0.0,
    "z_pos0": -0.65,
    "z_vel0": 2.0,
    "slope": 40,
}
```
### Terrain Settings
```python
base_size=(1.5, 0.5, 0.05),
base_color=(0.5, 0.5, 0.5, 1.0),
top_color=(0.4, 0.4, 0.7, 1.0),
n=60,
top_box_x=0.01,
top_box_z=0.005,
```

### Simulation Results

| Component | Parameter | Value | Units |
| :--- | :--- | :--- | :--- |
| **Knee** | Stiffness ($k$) | 107500 | N/m |
| **Knee** | Damping ($d$) | 1720 | Ns/m |
| **Ankle** | Stiffness ($k$) | 153.2 | N/rad |
| **Ankle** | Damping ($d$) | 1.598 | Ns/rad |

**Final Position:**
> **Y Pos:** 0.1494 m

---[1.07488834e+05 1.53173929e+02 1.71827703e+03 1.59795729e+00] 0.14941786590158226