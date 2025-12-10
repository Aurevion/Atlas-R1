import numpy as np

def calculate(R,r,x,y,z,theta,alpha2,alpha3):
#    wheel_1 = ((-np.sin(theta))*(np.cos(theta)*x) + (y*np.cos(theta)**2) + (R*z))
#    wheel_2 = ((-np.sin(theta+alpha2))*(np.cos(theta))*x + (np.cos(theta))*(np.cos(theta+alpha2)) + (R*z))
#    wheel_3 = (-np.sin(theta+alpha3)*np.cos(theta)*x + np.cos(theta)*np.cos(theta+alpha3) + (R*z))
#    speed = np.array([wheel_1,wheel_2,wheel_3])/r
#    print(speed)
    coords = np.array([[x],[y],[w]])
    local_matrix = np.identity(3)*np.cos(theta)
    jacobian = np.array([
        [-np.sin(theta),np.cos(theta),R],
        [-np.sin(theta+alpha2),np.cos(theta+alpha2),R],
        [-np.sin(theta+alpha3),np.cos(theta+alpha3),R]
    ])

    speed = np.matmul(local_matrix,np.matmul(jacobian,coords))/r
    print(speed)
x = 1.0
y = 0.0
w = 0.5

robot_orientation = np.radians(180)

robot_radius = 0.774
wheel_radius = 0.019
a2 = np.radians(120)
a3 = np.radians(240)

calculate(robot_radius,wheel_radius,x,y,w,robot_orientation,a2,a3)