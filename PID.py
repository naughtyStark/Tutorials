  
import matplotlib.pyplot as plt
import numpy as np
import math as meth
plt.figure(1)

'''
imagine that you're on a cycle that has 2 rockets, one on the front, one on the rear and you can control the net thrust(force)
generated very precisely at every instant of time. Lets say you are at x=0 and you need to reach x = 5. How will you reach that position?
Well, you could use a PID controller! This piece of code doesn't really explain how PID came about to be, but it should give you an
idea of how to deal with such a controller. The problem described above is actually a second order system (use acceleration (2nd derivative of position)
to control position. Why PID? why not just P? Well, you have the freedom to try that out! 
first, run the code as is.
the blue line is the position of the system with respect to time. the orange line is the position where YOU want the system to be.
observe what happens if you were to use a P controller for a 2nd order system. Make Kd = 0 and Ki =0
observe what happens with a PD system. Keep Kd finite and make Ki 0. Increase the 'gravity' variable and see what happens (This simulates lifting a load against gravity
observe what happens with a PID controller when the input is varied with time. Change the target variable to 'sine'.
You can change the frequency of the sine wave as well as the amplitude. What happens when you do that(frequency)? can you think of why that happens?
observe what happens when you use a P controller for a 1st order system (use acceleration to control speed) with a 'constant' target
why is the output different from when you used the P controller with a 2nd order system?
observe what happens when you use a PID controller for a 1st order system(use these : Kp =1 ,Kd = 0.9, Ki = 0.1).
Is the output similar to the output of the P controller? 
change the target type to 'sine' and compare P and PID controller for 1st order system (keep Kp same)
For both 1st and 2nd order system, observe what happens when the frequency of the input sine wave is increased.
beyond this, you're free to do what you like!
'''


gravity = 0
dt = 0.01 #time step
order = 0 #order of the system : 1st order -> you want to control speed but can control acceleration, 2nd Order -> you want to control position but can only control acceleration
initial_pos = 0 #starting position of system
initial_vel = 0 #starting velocity of system
target = 'constant' #change this to 'sine' to see what happens if you give a sinusoidal input to the system
state_estimate = ''
update_time = 0.01
update_cycles = update_time/dt
amplitude = 5 #pick any random value for the amplitude 
frequency = 0.3 #this applies to sinusoidal input
FF_gain = 0.5
FF_gain_var = 0.4 # FF_gain used by the controller
hunt_FF = True
learning_rate = 0.1
mass = 1 #mass of the object
force_limit = 10
Kc = 0.1*force_limit/amplitude #this is the proportionality gain
Tc = 6
# amplitude_rate = amplitude/Tc
amplitude_rate = amplitude*frequency
Td = 0.33*Tc
Ti = 0.5*Tc
Kp = 0.1
Kd = 0.09
# Kd = 0
Ki = 0

t = np.arange(-dt,15,dt)

def limit(t,bound):
	if(t>bound):
		return bound
	if(t< -bound):
		return -bound
	return t

xA = np.zeros(2)
yA = np.zeros(2)

def LPF(x):  # 2 hz low pass filter at 100 fps
    global xA
    global yA
    xA[0] = xA[1]
    xA[1] = x / 64.65674116
    yA[0] = yA[1]
    yA[1] = (xA[0] + xA[1]) + (0.9690674172 * yA[0])  # first order LPF to predict new speed.
    return yA[1]

_xA = np.zeros(2)
_yA = np.zeros(2)
def first_order_far_pole(x):  # 10 hz low pass filter at 100 fps
    global _xA
    global _yA
    _xA[0] = _xA[1]
    _xA[1] = (x/FF_gain) / 64.65674116	
    _yA[0] = _yA[1]
    _yA[1] = (_xA[0] + _xA[1]) + (0.9690674172 * _yA[0])  # first order LPF to predict new speed.
    return _yA[1]

def PID(t,Kp,Kd,Ki,dt=0.01,order = 2,pos=0,vel=0,target = 'constant',amplitude = 5,mass = 1):
	y = np.zeros_like(t)
	z = np.zeros_like(t)
	w = np.zeros_like(t)
	ff = np.zeros_like(t)
	# initialize the state of the system (speed and all that)
	dError = vel
	if(target == 'constant'):
		lastError = amplitude #prevent derivative kick
	else:
		lastError = 0
	
	sError = 0
	current = pos 
	
	for i in range(t.shape[0]):
		if(target == 'constant'):
			set_point = amplitude
		elif(target == 'sine'):
			set_point = amplitude*meth.sin(2*np.pi*frequency*t[i])
		elif(target == 'step'):
			set_point = amplitude*np.sign(meth.sin(2*np.pi*frequency*t[i]))
		else:
			print("the target input type is not valid, please input either 'constant' or 'sine' as target input type")
		if(state_estimate=='Low_pass'):
			apparent_current = LPF(current)
			w[i] = apparent_current
		elif(state_estimate == 'drift'):
			apparent_current = current + 0.2*t[i]*meth.sin(0.1*t[i])
			w[i] = apparent_current
		elif(state_estimate == 'variance'):
			apparent_current = current + np.random.normal(0,0.5)
			w[i] = apparent_current
		elif(state_estimate == 'sensor_failure'):
			apparent_current = current + np.random.normal(0,t[i]*meth.sin(t[i]))
			w[i] = apparent_current
		elif(state_estimate == 'time_lag'):
			apparent_current = y[i-40] # i can get away with this because y was initialized as np.zeros
			w[i] = apparent_current
		else:
			apparent_current = current
			w[i] = apparent_current

		if(i%update_cycles==0):
			error = set_point - apparent_current #error 
			# error = set_point - current
			dError = (error - lastError)/dt #derivative of error
			sError = limit(sError + error*dt,10) #integral of error 
			lastError = error 

		force = Kp*error + Kd*dError + Ki*sError #PID equation
		force = limit(force, force_limit)
		acc = (force/mass) - gravity

		#effect of PID output on the system
		if(order == 2):
			vel += acc*dt #increment velocity
			current += vel*dt #increment position
		if(order == 1):
			current += acc*dt #increment only velocity
		if(order == 0):
			if(hunt_FF):
				if(meth.fabs(dError<1)):#ensure the system isn't moving already 
					global FF_gain_var
					FF_gain_var += learning_rate*error*dt
					ff[i] = FF_gain_var
				current = first_order_far_pole(set_point*FF_gain_var)
			else:
				current = first_order_far_pole(set_point*FF_gain_var + acc)
		y[i] = current
		z[i] = set_point


	return y,z,w,ff


position,set_point,apparent_current,ff = PID(t, Kp, Kd, Ki, dt, order, initial_pos, initial_vel, target, amplitude,mass) 

plt.title('PID')
plt.xlabel('time')
plt.ylabel('variable')
# plt.plot(t,t*0)
# plt.plot(t*0,t)
if(state_estimate !=''):
	plt.plot(t,apparent_current,label='apparent state estimate')
plt.plot(t, position,label='state')
plt.plot(t, set_point,label='setpoint',linestyle='--')
plt.legend()
plt.axis('equal')
plt.show()

if(hunt_FF):
	plt.xlabel('time')
	plt.ylabel('gain')
	plt.plot(t,ff,label='controller feed forward gain')
	plt.plot(t,np.ones_like(t)*FF_gain, label ='true feed forward gain')
	plt.legend()
	plt.show()
