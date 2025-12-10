import numpy as np

def wheel_rates(x_dot, y_dot, theta_dot, theta, r, R, a1, a2, a3):
    sin = np.sin
    cos = np.cos

    T = np.array([
        [-sin(theta),          cos(theta),          R],
        [-sin(theta + a2),     cos(theta + a2),     R],
        [-sin(theta + a3),     cos(theta + a3),     R]
    ]) * (1.0 / r)

    B = np.diag([cos(theta), cos(theta), cos(theta)])

    v = np.array([[x_dot], [y_dot], [theta_dot]])
    
    speed = np.matmul(T,np.matmul(B,v))
    return speed

# Example usage
theta = 0.5
x_dot = 1
y_dot = 0
theta_dot = np.pi/2

r = 0.0019
R = 0.774
a1 = 0.0
a2 = 2*np.pi/3
a3 = 4*np.pi/3

rates = wheel_rates(x_dot, y_dot, theta_dot, theta, r, R, a1, a2, a3)
print(rates)
