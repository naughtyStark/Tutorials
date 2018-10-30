import matplotlib.pyplot as plt
import numpy as np
import math as meth
plt.figure(2)


Kp = 1 #this is the proportionality gain
Kd = 1 #derivative gain
Ki = 0.1 #integral gain
dt = 0.01 #time step
order = 2 #order of the system : 1st order -> you want to control speed but can control acceleration, 2nd Order -> you want to control position but can only control acceleration
initial_pos = 0 #starting position of system
initial_vel = 0 #starting velocity of system
target = 'constant' #change this to 'sine' to see what happens if you give a sinusoidal input to the system
amplitude = 5 #pick any random value for the amplitude 
kick = False #make this true to introduce derivative kick
frequency = 10 #this applies to sinusoidal input
mass = 1 #mass of the object

t = np.arange(0,10,dt)

def limit(t,bound):
	if(t>bound):
		return bound
	if(t< -bound):
		return -bound
	return t

def PID(t,Kp,Kd,Ki,dt=0.01,order = 2,pos=0,vel=0,target = 'constant',amplitude = 5,kick = False,mass = 1):
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

		force = Kp*error + Kd*dError + Ki*sError #PID equation
		acc = force/mass

		#effect of PID output on the system
		if(order == 2):
			vel += acc*dt #increment velocity
			current += vel*dt #increment position
		if(order == 1):
			current += acc*dt #increment only velocity
		y[i] = current
		z[i] = set_point

	return y,z


position,set_point = PID(t, Kp, Kd, Ki, dt, order, initial_pos, initial_vel, target, amplitude, kick,mass) 

plt.title('PID')
plt.xlabel('time')
plt.ylabel('variable')
plt.plot(t, position)
plt.plot(t, set_point)

plt.show()
