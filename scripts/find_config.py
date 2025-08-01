# to visualise reachable workspace of robot arm with different equilibrium angles
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
import math

cos = lambda degrees: math.cos(math.radians(degrees)) 
sin = lambda degrees: math.sin(math.radians(degrees)) 

# link lengths (cm)
a1 = 10.5
a2 = 12.4 
a3 = 18.2

fig, ax = plt.subplots()

def draw(eqb1,eqb2,eqb3):
    for theta1 in range(eqb1-90,eqb1+90+1):
        for theta2 in range(eqb2-90,eqb2+90+1):
            # coordinates of joint 2
            p2x = a1*cos(theta1) + a2*cos(theta1+theta2)
            p2y = a1*sin(theta1) + a2*sin(theta1+theta2)
            
            arc = Arc(xy=(p2x,p2y),width=2*a3,height=2*a3,
                      angle=theta1+theta2,theta1=eqb3-90,theta2=eqb3+90,
                      edgecolor="blue",lw=0.5)
            ax.add_patch(arc)
            #plt.plot(p2x,p2y,"ro")
 
            print("theta1: {}, theta2: {}".format(theta1,theta2))


draw(45,0,0)

full_radius = a1+a2+a3
ax.set_aspect('equal')
ax.set_xlim(-50, 50)
ax.set_ylim(-50, 50)
ax.axhline(0)
ax.axvline(0)

plt.show()


# theta1 0 degrees = horizontal
# theta2,3 0 degrees = exactly follow link / straight line from link
# the math follows this, so everything has to follow this
# IK implementation also follow this, just convert from theoretical angle to actual servo angle



# servos might be physically limited to turn that much, might need to find max angle
# 0,0,0 for now
