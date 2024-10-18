import numpy as np
import matplotlib.pyplot as plt
import math as m
# Function to create a true trajectory, measurements and measured trajectory

def EKF_vehicleTracking_Trajectory(sigma_nr_data, 
                                   sigma_nb_data, 
                                   N, 
                                   delta, 
                                   trajectory_type='circle'):
    
#trajectory_type # Possible values: 'line', 'circle'
 
    # Generate trajectory: true (x,y) coordinates of the moving vehicle
    if trajectory_type == 'line':
        # Parameters of the line
        a = np.array([1, 0])
        b = np.array([-50, 10])
        trajectory = np.outer(np.arange(N), a) * delta + np.ones((N, 1)) @ b.reshape(1, -1)

    elif trajectory_type == 'circle':
        # Parameters of the circle
        radius = 50  # radius of the circle
        center = np.array([0, 0])  # center of the circle
        # Assume constant angular velocity for the vehicle
        angular_velocity = 2 * np.pi / 100
        angles = angular_velocity * delta * np.arange(N)
        trajectory = radius * np.column_stack((np.cos(angles), np.sin(angles))) + center

    # Assume that the sensor is at (0,0). Generate the measurements
    # Range measurements
    m_range = np.sqrt(trajectory[:, 0]**2 + trajectory[:, 1]**2) + sigma_nr_data * np.random.randn(N)
    # Bearing measurements
    m_bearing = np.arctan2(trajectory[:, 1], trajectory[:, 0]) + sigma_nb_data * np.random.randn(N)
    m_bearing = np.mod(m_bearing + 10 * np.pi, 2 * np.pi)
    measurement = np.column_stack((m_range, m_bearing))  # measurement vector

    # Convert measurements to (x,y) coordinates
    trajectory_measured = np.column_stack((measurement[:, 0] * np.cos(measurement[:, 1]), measurement[:, 0] * np.sin(measurement[:, 1])))

    # # Uncomment to plot the true trajectory and the measurements  without  EKF
    # plt.figure(1)
    # plt.plot(trajectory[:, 0], trajectory[:, 1], '*g-', markersize=10, label='True position')
    # plt.plot(trajectory_measured[:, 0], trajectory_measured[:, 1], 'b--o', markersize=10, label='Measurement')
    # plt.xlim([-80, 80])
    # plt.ylim([-80, 80])
    # plt.title('Trajectory')
    # plt.xlabel('x dimension in m', fontsize=18)
    # plt.ylabel('y dimension in m', fontsize=18)
    # plt.legend(loc='best', fontsize=12)
    # plt.axis('equal')
    # plt.grid(True)
    # plt.xticks(fontsize=16)
    # plt.yticks(fontsize=16)
    # plt.show()

    # # Uncomment to plot velocity
    # velocity = (trajectory[1:, :] - trajectory[:-1, :]) / delta
    # plt.figure(2)
    # plt.plot(velocity[:, 0], ':db', markersize=10, label='x-velocity')
    # plt.plot(velocity[:, 1], ':sr', markersize=10, label='y-velocity')
    # plt.xlabel('Time index', fontsize=18)
    # plt.ylabel('Velocity', fontsize=18)
    # plt.legend(loc='best', fontsize=12)
    # plt.grid(True)
    # plt.xticks(fontsize=16)
    # plt.yticks(fontsize=16)
    # plt.show()

    return trajectory, trajectory_measured, measurement

# Measurement function
def h(x):
    range_measurement = np.sqrt(np.sum(x[0:2]**2))
    bearing_measurement = np.mod(np.arctan2(x[1], x[0]), 2 * np.pi)
    return np.array([range_measurement, bearing_measurement])

# Function to compute angular difference and wrap it to [-pi, pi]
def angdiffwrap(x, y):
    d = np.mod(x-y, 2 * np.pi)
    if d > np.pi:
        d = d - 2 * np.pi
    return d



# Parameters
N = 100  # number of time-steps
delta = 1  # time-step

trajectory_type = 'line'  # Possible values: 'line', 'circle'

# measurement noise levels for creating data
sigma_nr_data = 0.1  # standard deviation of range measurements, used in creating data
sigma_nb_data = 0.01  # standard deviation of bearing measurements, used in creating data 

# Generate trajectory, measurements and measured trajectory
trajectory, trajectory_measured, measurement = EKF_vehicleTracking_Trajectory(sigma_nr_data, sigma_nb_data, N, delta, trajectory_type)



# measurement noise level assumed by EKF
sigma_nr = sigma_nr_data  # standard deviation of range measurements, assumed by EKF
sigma_nb = sigma_nb_data  # standard deviation of bearing measurements, assumed by EKF
R = np.diag([sigma_nr**2, sigma_nb**2])

# process noise level assumed by EKF
sigma_q = 0.1
Q = sigma_q**2 * np.eye(2)


# State vector: [x-position, y-position, x-velocity, y-velocity]
F = np.eye(4)
F[0, 2] = delta
F[1, 3] = delta

B = np.zeros((4, 2))
B[2, 0] = 1
B[3, 1] = 1



# Variables to store the Jacobian, state estimates, and error-covariance matrices
H = np.zeros((2, 4))
X_pred = np.nan * np.ones((N+1, 4))
X_est = np.nan * np.ones((N+1, 4))
Pp_mat = np.nan * np.ones((N+1, 4, 4))
P_mat = np.nan * np.ones((N+1, 4, 4))

# EKF initialization
x_est_0 = np.hstack((trajectory[0, :] + np.array([10, 15]), [1, 1]))
P_0 = 10 * np.diag([1, 0, 0, 1])
P_mat[0:,:,:] = P_0
X_est[0, :] = x_est_0

x_est = x_est_0
P=P_0
y_inov=np.zeros(2)

# EKF iteration
for ii in range(N):
    # STODO: Prediction
    # [Note that we don't need to compute linearized state matrix, F is already linear]    
    x_pred = F @ x_est        # prediction of state
    Pp = F@P@F.transpose() + B @ Q @ B.transpose()   # Updata the prediction MSE matrix
    
    # STODO: to determine the linearized measurement matrix, calculate the Jacobian
      
    H[0, 0] = x_pred[0] / np.sqrt(x_pred[0]**2 + x_pred[1]**2)  # nabla r/x
    H[0, 1] = x_pred[1] / np.sqrt(x_pred[0]**2 + x_pred[1]**2)  # nabla r/y
    H[1, 0] = -x_pred[1] / (x_pred[0]**2 + x_pred[1]**2)        # nabla B/x
    H[1, 1] = x_pred[0] / (x_pred[0]**2 + x_pred[1]**2)         # nabla B/y


    # STODO:Update
    y = h(x_est)
    y_pred= h(x_pred)              # Predicted measurement
    y_inov[0] = y[0] - y_pred[0]   #est         # Innovation in range
    y_inov[1] = y[1] - y_pred[0]   #est        # Innovation in angle. You can use the function angdiffwrap to wrap the angle difference to [-pi, pi]. 
    S = H@Pp@H.transpose() + R                    # Covariance of innovation
    K = Pp@H.transpose()@np.linalg.inv(S)         #est           # Kalman gain matrix
    x_est = x_pred + K @ y_inov                # Correct the estimate
    P = Pp-K@H@Pp                   # Update the MSE matrix

    # Store values
    X_pred[ii+1, :] = x_pred
    X_est[ii+1, :] = x_est
    Pp_mat[ii+1, :, :] = Pp
    P_mat[ii+1, :, :] = P

# Plot Measurements
plt.figure(4)
plt.subplot(2, 1, 1)
plt.plot(measurement[:, 0], linewidth=2)
plt.title('Range Measurements')
plt.xlabel('Time [n]', fontsize=14)
plt.ylabel('Range [m]', fontsize=14)
plt.grid(True)
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)

plt.subplot(2, 1, 2)
plt.plot(measurement[:, 1], linewidth=2)
plt.title('Bearing Measurements')
plt.xlabel('Time [n]', fontsize=14)
plt.ylabel('Bearing [rad]', fontsize=14)
plt.grid(True)
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
plt.savefig('figEKF_RangeBearing.eps', format='eps')
plt.show()



# Plot Trajectory and Estimates with lines
plt.figure(3)
plt.plot(trajectory[:, 0], trajectory[:, 1], color=[.929, .694, .125], linewidth=2, label='True position')
plt.plot(trajectory_measured[:, 0], trajectory_measured[:, 1], 'k:', linewidth=2, label='Measurement')
plt.plot(X_est[:, 0], X_est[:, 1], '--', color=[0, .447, .741], linewidth=2, label='Estimated position')
plt.plot(0, 0, 'rx', linewidth=2, label='Sensor position')
plt.plot(x_est_0[0], x_est_0[1], 'ok', label='Initial estimate')
plt.grid(True)
plt.legend(loc='best')
plt.xlabel('x-position [m]', fontsize=14)
plt.ylabel('y-position [m]', fontsize=14)
plt.title('Trajectory and Estimates')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.gca().set_aspect('equal', adjustable='box')
plt.savefig('figEKF_Estimate.eps', format='eps')
plt.show()

#  Plot Trajectory and Estimates with markers
plt.figure(4)
plt.plot(0, 0, 'rx', linewidth=2, label='Sensor Position')
plt.plot(x_est_0[0], x_est_0[1], 'ok', label='Initial Estimate')
# plt.xlim([-50, 50])
# plt.ylim([-5, 50])
#-----
for ii in range(N): 
    # NOTE:  a breakpoint here will allow you to visualize the estimates at each time step. Use the above plt.xlim/plt.ylim to adjust the limits of the plot if needed 
    plt.plot(trajectory[ii, 0], trajectory[ii, 1], '-o', color=[.929, .694, .125], linewidth=2, label='True Position' if ii == 0 else "")
    plt.plot(trajectory_measured[ii, 0], trajectory_measured[ii, 1], 'dk:', linewidth=2, label='Measurement' if ii == 0 else "")
    plt.plot(X_est[ii+1, 0], X_est[ii+1, 1], '--s', color=[0, .447, .741], linewidth=2, label='Estimated Position' if ii == 0 else "")
plt.grid(True)
plt.legend(loc='best')
plt.xlabel('x-position [m]', fontsize=14)
plt.ylabel('y-position [m]', fontsize=14)
plt.title('Trajectory and Estimates')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.savefig('figEKF_Estimate2.eps', format='eps')
plt.show()

