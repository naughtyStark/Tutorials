import matplotlib.pyplot as plt
import numpy as np
import math as meth
plt.figure(2)


Kp = 1
Kd = 1
Ki = 1
dt = 0.01
order = 2 
initial_pos = 0
initial_vel = 0
target = 'sine' #change this to 'sine' to see what happens if you give a sinusoidal input to the system
amplitude = 5 #pick any random value
kick = False
frequency = 10 #this applies to sinusoidal input

t = np.arange(0,10,dt)

def limit(t,bound):
	if(t>bound):
		return bound
	if(t< -bound):
		return -bound
	return t

def PID(t,Kp,Kd,Ki,dt=0.01,order = 2,pos=0,vel=0,target = 'constant',amplitude = 5,kick = False):
	y = np.empty_like(t)
	z = np.empty_like(t)
	# initialize the state of the system (speed and all that)
	dError = vel
	if( not kick):
		lastError = amplitude #prevent derivative kick
	else:
		lastError = 0
	
	sError = 0
	current = pos 
	
	for i in range(t.shape[0]):
		if(target == 'constant'):
			set_point = amplitude
		elif(target == 'sine'):
			set_point = amplitude*meth.sin(t[i])
		else:
			print("the target input type is not valid, please input either 'constant' or 'sine' as target input type")
		error = set_point - current #error 
		dError = (error - lastError)/dt #derivative of error
		sError = limit(sError + error*dt,10) #integral of error 
		lastError = error 

		acc = Kp*error + Kd*dError + Ki*sError #PID equation

		#effect of PID output on the system
		if(order == 2):
			vel += acc*dt
			current += vel*dt
		if(order == 1):
			current += acc*dt
		y[i] = current
		z[i] = set_point

	return y,z


position,set_point = PID(t, Kp, Kd, Ki, dt, order, initial_pos, initial_vel, target, amplitude, kick) 

plt.title('PID')
plt.xlabel('time')
plt.ylabel('variable')
plt.plot(t, position)
plt.plot(t, set_point)

plt.show()