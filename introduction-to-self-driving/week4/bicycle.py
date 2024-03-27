import numpy as np
import matplotlib.pyplot as plt


class Bicycle:
    def __init__(self):
        self.xc = 0
        self.yc = 0
        self.yaw_angle = 0
        self.sample_time = 0.01
        self.slip_angle = 0
        self.l_r = 1.2 #m
        self.L = 2 #m
        self.max_steering_angle_rate = 1.22 #rad/s
        self.turn_radius = 10 #m
        self.steering_angle = np.arctan(self.L/self.turn_radius)


    def reset(self):
        self.xc = 0
        self.yc = 0
        self.yaw_angle = 0
        self.steering_angle = np.arctan(self.L/self.turn_radius)
        self.slip_angle = 0

    def step(self, velocity, steering_angle_rate):
        if steering_angle_rate > self.max_steering_angle_rate:
            steering_angle_rate = self.max_steering_angle_rate
        elif steering_angle_rate < -self.max_steering_angle_rate:
            steering_angle_rate = -self.max_steering_angle_rate
        
        self.steering_angle += steering_angle_rate * self.sample_time
        self.slip_angle = np.arctan((self.l_r*np.tan(self.steering_angle))/self.L)
        yaw_rate = (velocity * np.cos(self.slip_angle)*np.tan(self.steering_angle))/self.L
        self.yaw_angle += yaw_rate * self.sample_time
            
        longitudinal_acc = velocity * np.cos(self.yaw_angle + self.slip_angle)
        lateral_acc = velocity * np.sin(self.yaw_angle + self.slip_angle)
        self.xc += longitudinal_acc * self.sample_time
        self.yc += lateral_acc * self.sample_time

sample_time = 0.01
time_end = 20
model = Bicycle()

# set delta directly
t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)
x_solution = np.zeros_like(t_data)
y_solution = np.zeros_like(t_data)

for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    model.step(np.pi, 0)
    #model.beta = 0
    #solution_model.beta=0
    
#plt.axis('equal')
#plt.plot(x_data, y_data,label='Learner Model')
#plt.legend()
#plt.show()

sample_time = 0.01
time_end = 30
model.reset()
model.turn_radius = 8 #m
max_steering_angle = np.tan(model.L/model.turn_radius)
t_data = np.arange(0,time_end,sample_time)
n = t_data.shape[0]

v_data = np.zeros_like(t_data)
v_data[:] = 2*2*np.pi*model.turn_radius/time_end

x_data = np.zeros_like(t_data)
y_data = np.zeros_like(t_data)

for i in range(t_data.shape[0]):
    x_data[i] = model.xc
    y_data[i] = model.yc
    if i < t_data.shape[0]/8:
        if model.steering_angle < max_steering_angle:
            model.step(v_data[i], model.max_steering_angle_rate)
        else:
            model.step(v_data[i], 0)
    elif i < 5*t_data.shape[0]/8:
        if model.steering_angle > -max_steering_angle:
            model.step(v_data[i], -model.max_steering_angle_rate)
        else:
            model.step(v_data[i], 0)
    else:# i < 6*t_data.shape[0]/8:
        if model.steering_angle < max_steering_angle:
            model.step(v_data[i], model.max_steering_angle_rate)
        else:
            model.step(v_data[i], 0)
        #w_data[i] = model.w_max
        #else:
        #    model.step(v_data[i], 0)
        #    w_data[i] = 0

    #model.beta = 0
    #solution_model.beta=0
    
plt.axis('equal')
plt.plot(x_data, y_data,label='Learner Model')
plt.legend()
plt.show()