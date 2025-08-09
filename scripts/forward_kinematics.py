# forward kinematics for testing IK
import math
                            
cos = lambda degrees: math.cos(math.radians(degrees)) 
sin = lambda degrees: math.sin(math.radians(degrees)) 
                                                         
# link lengths (cm)
a1 = 10.5    
a2 = 12.4 
a3 = 18.2 

# (degrees)
theta1 = 116.164251
theta2 = 75.037919
theta3 = -86.202170


x = a1*cos(theta1) + a2*cos(theta1+theta2) + a3*cos(theta1+theta2+theta3)
y = a1*sin(theta1) + a2*sin(theta1+theta2) + a3*sin(theta1+theta2+theta3)
psi = theta1 + theta2 + theta3

print("x: {}, y: {}, psi: {}".format(x,y,psi))
