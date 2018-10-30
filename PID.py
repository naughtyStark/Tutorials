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

Kp = 1 #this is the proportionality gain
Kd = 1 #derivative gain
Ki = 0 #integral gain
gravity = 0
dt = 0.01 #time step
order = 2 #order of the system : 1st order -> you want to control speed but can control acceleration, 2nd Order -> you want to control position but can only control acceleration
initial_pos = 0 #starting position of system
initial_vel = 0 #starting velocity of system
target = 'sine' #change this to 'sine' to see what happens if you give a sinusoidal input to the system
amplitude = 5 #pick any random value for the amplitude 
frequency = 0.1 #this applies to sinusoidal input
mass = 1 #mass of the object

t = np.arange(0,10,dt)

def limit(t,bound):
	if(t>bound):
		return bound
	if(t< -bound):
		return -bound
	return t

def PID(t,Kp,Kd,Ki,dt=0.01,order = 2,pos=0,vel=0,target = 'constant',amplitude = 5,mass = 1):
	y = np.empty_like(t)
	z = np.empty_like(t)
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
		else:
			print("the target input type is not valid, please input either 'constant' or 'sine' as target input type")
		error = set_point - current #error 
		dError = (error - lastError)/dt #derivative of error
		sError = limit(sError + error*dt,10) #integral of error 
		lastError = error 

		force = Kp*error + Kd*dError + Ki*sError #PID equation
		acc = (force/mass) - gravity

		#effect of PID output on the system
		if(order == 2):
			vel += acc*dt #increment velocity
			current += vel*dt #increment position
		if(order == 1):
			current += acc*dt #increment only velocity
		y[i] = current
		z[i] = set_point

	return y,z


position,set_point = PID(t, Kp, Kd, Ki, dt, order, initial_pos, initial_vel, target, amplitude,mass) 

plt.title('PID')
plt.xlabel('time')
plt.ylabel('variable')
plt.plot(t, position)
plt.plot(t, set_point)

plt.show()
