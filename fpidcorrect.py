import numpy as np
import matplotlib.pyplot as plt

# Leader and Follower data
leader_velocity_x = [0.24, 0.35, 0.49, 0.45, 0.28, 0.27, 0.33, 0.32, 0.43, 0.59, 0.59, 0.61, 0.46, 0.00, -0.05, -0.23, -0.34, -0.43, -0.45, -0.36, -0.38, -0.40, -0.32, -0.28, -0.25, -0.24, -0.15, -0.34, -0.35, -0.42, -0.55, -0.66, -0.73, -0.40]
leader_velocity_y = [-0.77, -0.88, -0.83, -0.89, -0.75, -0.76, -0.84, -0.96, -1.01, -1.55, -1.57, -1.62, -1.41, -0.04, 0.22, 0.49, 0.90, 0.95, 1.02, 0.97, 1.00, 0.98, 0.93, 0.90, 0.68, 0.64, 0.77, 0.90, 1.14, 1.27, 1.54, 2.09, 2.46, 1.31]
follower_velocity_x = [0.26, 0.45, 0.56, 0.65, 0.16, 0.52, 0.51, 0.54, 0.65, 0.38, 0.87, 0.91, 0.71, 0.02, -0.25, -1.81, -1.77, -0.44, -0.41, -0.38, -0.39, 1.31, -0.20, -0.46, -0.39, -0.27, -0.13, -0.38, -0.47, -0.59, -0.63, -0.83, -0.79, 0.15]
follower_velocity_y = [-0.76, -0.82, -0.71, -0.79, -0.63, -3.03, -0.43, -0.72, -0.98, -1.00, -1.41, -1.43, -1.31, -0.56, 0.11, 0.04, -0.10, 0.83, 1.04, 0.74, 0.88, 0.49, 0.85, 0.84, 0.75, 0.59, 0.87, 0.88, 1.34, 1.23, 1.46, 2.05, 2.32, 0.73]
diff_dist_x = [-0.5571, -0.9521, -1.1945, -1.4478, -0.0666, 1.0997, 0.1047, -3.1709, -3.5048, -1.0970, -2.4139, -3.1729, -0.3653, -2.0150, -2.9943, -2.1088, 1.2916, 3.6539, 3.9333, 3.7046, 3.2887, 2.9097, 1.1515, 0.4072, 0.7832, 0.3020, 0.1929, -0.0681, -0.0158, -0.7157, 0.3269, 0.1417, 1.1640, 0.7533]
diff_dist_y = [-1.4223, -0.7301, 0.1551, 0.8957, -3.2722, -0.6866, 0.3070, -0.6726, -0.0062, -3.2539, -1.7123, -0.5260, -3.5788, -2.0272, -0.7424, -0.8724, -0.4228, -0.3355, -0.6459, 1.9917, 2.6478, 3.1968, 3.6534, 2.3454, 3.4377, 1.4976, 0.5989, 2.1402, 2.0274, -0.1872, -1.1569, -0.5170, 0.2380, 3.9163]

# PID controller constants
Kp = 0.1
Ki = 0.01
Kd = 0.05

# Initialize error values
previous_error_x = 0
previous_error_y = 0
integral_x = 0
integral_y = 0

pid_output_x = []
pid_output_y = []

# Calculate PID output for each time step
for i in range(len(diff_dist_x)):
    error_x = diff_dist_x[i]
    error_y = diff_dist_y[i]
    
    # Proportional term
    P_out_x = Kp * error_x
    P_out_y = Kp * error_y
    
    # Integral term
    integral_x += error_x
    integral_y += error_y
    I_out_x = Ki * integral_x
    I_out_y = Ki * integral_y
    
    # Derivative term
    derivative_x = error_x - previous_error_x
    derivative_y = error_y - previous_error_y
    D_out_x = Kd * derivative_x
    D_out_y = Kd * derivative_y
    
    # PID output
    output_x = P_out_x + I_out_x + D_out_x
    output_y = P_out_y + I_out_y + D_out_y
    
    pid_output_x.append(output_x)
    pid_output_y.append(output_y)
    
    previous_error_x = error_x
    previous_error_y = error_y

# Plotting the results
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(pid_output_x, label='PID Output X')
plt.plot(follower_velocity_x, label='Follower Velocity X')
plt.plot(leader_velocity_x, label='Leader Velocity X')
plt.legend()
plt.title('PID Output vs. Follower & Leader Velocity (X-axis)')
plt.xlabel('Time Steps')
plt.ylabel('Velocity (m/s)')

plt.subplot(2, 1, 2)
plt.plot(pid_output_y, label='PID Output Y')
plt.plot(follower_velocity_y, label='Follower Velocity Y')
plt.plot(leader_velocity_y, label='Leader Velocity Y')
plt.legend()
plt.title('PID Output vs. Follower & Leader Velocity (Y-axis)')
plt.xlabel('Time Steps')
plt.ylabel('Velocity (m/s)')

plt.tight_layout()
plt.show()
