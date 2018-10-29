#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for 
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link 
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed
# at UC Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu). The cloud services integation with ROS was developed
# by Kiet Lam  (kiet.lam@berkeley.edu). The web-server app Dator was 
# based on an open source project by Bruce Wootton
# ---------------------------------------------------------------------------
# revised by: Chen Tang 

from numpy import sin, cos, tan, arctan, array, dot
from numpy import sign, argmin, sqrt
from math import atan2
import rospy

# discrete non-linear bicycle model dynamics
def f_2s(z, u, vhMdl, trMdl, dt, v_x): 
    """
    process model
    input: state z at time k, z[k] := [beta[k], r[k]], (i.e. slip angle and yaw rate)
    output: state at next time step (k+1)
    """
    
    # get states / inputs
    beta    = z[0]
    r       = z[1]
    d_f     = u
    
    # extract parameters
    (a,b,m,I_z) = vhMdl
    (trMdlFront, trMdlRear) = trMdl

    # comptue the front/rear slip  [rad/s]
    # ref: Hindiyeh Thesis, p58
    a_F     = arctan(beta + a*r/v_x) - d_f
    a_R     = arctan(beta - b*r/v_x)

    # compute tire force
    FyF     = f_pajecka(trMdlFront, a_F)
    FyR     = f_pajecka(trMdlRear, a_R)

    # compute next state
    beta_next   = beta  + dt*(-r + (1/(m*v_x))*(FyF*cos(d_f)+FyR))
    r_next      = r    + dt/I_z*(a*FyF*cos(d_f) - b*FyR);
    return array([beta_next, r_next])

# discrete non-linear bicycle model dynamics
def f_3s(z, u, vhMdl, trMdl, F_ext, dt): 
    """
    process model
    input: state z at time k, z[k] := [v_x[k], v_y[k], r[k]])
    output: state at next time step z[k+1]
    """
    
    # get states / inputs
    v_x     = z[0]
    v_y     = z[1]
    r       = z[2]
    d_f     = u[0]
    FxR     = u[1]

    # extract parameters
    (a,b,m,I_z)             = vhMdl
    (a0, Ff)                = F_ext
    (trMdlFront, trMdlRear) = trMdl
    (B,C,mu)                = trMdlFront
    g                       = 9.81
    Fn                      = m*g/2.0         # assuming a = b (i.e. distance from CoG to either axel)

    # limit force to tire friction circle
    if FxR >= mu*Fn:
        FxR = mu*Fn

    # comptue the front/rear slip  [rad/s]
    # ref: Hindiyeh Thesis, p58
    a_F     = arctan((v_y + a*r)/v_x) - d_f
    a_R     = arctan((v_y - b*r)/v_x)

    # compute lateral tire force at the front
    TM_param    = [B, C, mu*Fn]
    FyF         = -f_pajecka(TM_param, a_F)

    # compute lateral tire force at the rear
    # ensure that magnitude of longitudinal/lateral force lie within friction circle
    FyR_paj     = -f_pajecka(TM_param, a_R)
    FyR_max     = sqrt((mu*Fn)**2 - FxR**2)
    FyR         = min(FyR_max, max(-FyR_max, FyR_paj))

    # compute next state
    v_x_next    = v_x + dt*(r*v_y +1/m*(FxR - FyF*sin(d_f)) - a0*v_x**2 - Ff)
    v_y_next    = v_y + dt*(-r*v_x +1/m*(FyF*cos(d_f) + FyR))
    r_next      = r    + dt/I_z*(a*FyF*cos(d_f) - b*FyR)

    return array([v_x_next, v_y_next, r_next])

# discrete non-linear bicycle model dynamics 6-dof
def f_6s(z, u, vhMdl, trMdl, F_ext, dt): 
    """
    process model
    input: state z at time k, z[k] := [X[k], Y[k], phi[k], v_x[k], v_y[k], r[k]])
    output: state at next time step z[k+1]
    """
    
    # get states / inputs
    X       = z[0]
    Y       = z[1]
    phi     = z[2]
    v_x     = z[3]
    v_y     = z[4]
    r       = z[5]

    d_f     = u[0]
    FxR     = u[1]

    # extract parameters
    (a,b,m,I_z)             = vhMdl
    (a0, Ff)                = F_ext
    (trMdlFront, trMdlRear) = trMdl
    (B,C,mu)                = trMdlFront
    g                       = 9.81
    Fn                      = m*g/2.0         # assuming a = b (i.e. distance from CoG to either axel)

    # limit force to tire friction circle
    if FxR >= mu*Fn:
        FxR = mu*Fn

    # comptue the front/rear slip  [rad/s]
    # ref: Hindiyeh Thesis, p58
    a_F     = arctan((v_y + a*r)/v_x) - d_f
    a_R     = arctan((v_y - b*r)/v_x)

    # compute lateral tire force at the front
    TM_param    = [B, C, mu*Fn]
    FyF         = -f_pajecka(TM_param, a_F)

    # compute lateral tire force at the rear
    # ensure that magnitude of longitudinal/lateral force lie within friction circle
    FyR_paj     = -f_pajecka(TM_param, a_R)
    FyR_max     = sqrt((mu*Fn)**2 - FxR**2)
    Fy          = array([FyR_max, FyR_paj])
    idx         = argmin(abs(Fy))
    FyR         = Fy[idx]

    # compute next state
    X_next      = X + dt*(v_x*cos(phi) - v_y*sin(phi)) 
    Y_next      = Y + dt*(v_x*sin(phi) + v_y*cos(phi)) 
    phi_next    = phi + dt*r
    v_x_next    = v_x + dt*(r*v_y +1/m*(FxR - FyF*sin(d_f)) - a0*v_x**2 - Ff)
    v_y_next    = v_y + dt*(-r*v_x +1/m*(FyF*cos(d_f) + FyR))
    r_next      = r    + dt/I_z*(a*FyF*cos(d_f) - b*FyR)

    return array([X_next, Y_next, phi_next, v_x_next, v_y_next, r_next])

def f_PointMass(z, w, u, dt):
	# get states/inputs
	Vx  = z[0]
	Vy  = z[1]
	X   = z[2]
	Y   = z[3]
	psi = z[4]

	ax  = u[0]
	ay  = u[1]
	wz  = u[2]

	# compute next state
	Vx_next  = Vx + dt*((ax + w[0]) + Vy*(wz + w[2]))
	Vy_next  = Vy + dt*((ay + w[1]) - Vx*(wz + w[2]))
	X_next   = X + dt*(Vx*cos(psi) - Vy*sin(psi) + w[3])
	Y_next   = Y + dt*(Vx*sin(psi) + Vy*cos(psi) + w[4])
	psi_next = psi + dt*(wz + w[1])

	return array([Vx_next, Vy_next, X_next, Y_next, psi_next])

def f_BicycleModel(z, w, u, vhMdl, trMdl, dt):
    # get states/inputs
    Vx  = z[0]
    Vy  = z[1]
    X   = z[2]
    Y   = z[3]
    psi = z[4]
    wz  = z[5]

    ax    = u[0]
    delta = u[1]

    # extract parameters
    (a,b,m,I) = vhMdl
    (Caf, Car) = trMdl

    # compute lateral tire forces
    Fyf = -Caf * (atan2(Vy + wz*a, max(Vx, mph2ms(5))) - delta)
    Fyr = -Car * atan2(Vy - wz*b, max(Vx, mph2ms(5)))

    # compute next state
    Vx_next  = Vx + dt*(ax + w[0])
    Vy_next  = Vy + dt*(tan(delta)*(ax - wz*Vy) + (Fyf/cos(delta) + Fyr)/m - wz*Vx + w[1])
    X_next   = X + dt*(Vx*cos(psi) - Vy*sin(psi) + w[2])
    Y_next   = Y + dt*(Vx*sin(psi) + Vy*cos(psi) + w[3])
    psi_next = psi + dt*(wz + w[4])
    wz_next  = wz + dt*(m*a/I*tan(delta)*(ax - wz*Vy) + a*Fyf/(I*cos(delta)) - b*Fyr/I + w[5])

    return array([Vx_next, Vy_next, X_next, Y_next, psi_next, wz_next])

def h_BicycleModel_withoutGPS(z, v):
    return array([z[0] + v[0], z[4] + v[1]])

def h_BicycleModel_withGPS(z, v):
    return array([z[0] + v[0], z[2] + v[1], z[3] + v[2], z[4] + v[3]])

def h_PointMass_withoutGPS(z, v):
	return array([z[0] + v[0], z[4] + v[1]])

def h_PointMass_withGPS(z, v):
	return array([z[0] + v[0], z[2] + v[1], z[3] + v[2], z[4] + v[3]])

def h_2s(x):
    """
    measurement model
    state: z := [beta, r], (i.e. slip angle and yaw rate)
    output h := r (yaw rate)
    """
    C = array([[0, 1]])
    return dot(C, x)
 
def h_3s(x):
    """
    measurement model
    input   := state z at time k, z[k] := [v_x[k], v_y[k], r[k]])
    output  := [v_x, r] (yaw rate)
    """
    C = array([[1, 0, 0],
               [0, 0, 1]])
    return dot(C, x)
    
   
def f_pajecka(trMdl, alpha):
    """
    f_pajecka = d*sin(c*atan(b*alpha))    
    
    inputs :
        * trMdl := tire model, a list or tuple of parameters (b,c,d)
        * alpha := tire slip angle [radians]
    outputs :
        * Fy := lateral force from tire [Newtons]
    """
    (b,c,d) = trMdl
    return  d*sin(c*arctan(b*alpha)) 


def f_KinBkMdl(z,u,vhMdl, dt):
    """
    process model
    input: state z at time k, z[k] := [x[k], y[k], psi[k], v[k]]
    output: state at next time step z[k+1]
    """
    
    # get states / inputs
    x       = z[0]
    y       = z[1]
    psi     = z[2]
    v       = z[3]

    d_f     = u[0]
    a       = u[1]

    # extract parameters
    (L_a, L_b)             = vhMdl

    # compute slip angle
    bta         = arctan( L_a / (L_a + L_b) * tan(d_f) )

    # compute next state
    x_next      = x + dt*( v*cos(psi + bta) ) 
    y_next      = y + dt*( v*sin(psi + bta) ) 
    psi_next    = psi + dt*v/L_b*sin(bta)
    v_next      = v + dt*a

    return array([x_next, y_next, psi_next, v_next])
 
def h_KinBkMdl(x):
    """
    measurement model
    """
    C = array([[0, 0, 1, 0],
               [0, 0, 0, 1]])
    return dot(C, x)
 
def mph2ms(mph):
    return (mph*0.44704)
