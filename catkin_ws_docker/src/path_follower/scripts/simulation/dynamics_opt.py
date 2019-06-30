#!/usr/bin/env python
# -*- coding: utf-8 -*-
from numpy import array, argmin, arctan, sign
from math import sqrt, cos, sin

  
# discrete non-linear bicycle model dynamics 6-dof
def f_6s(z, u, vhMdl, trMdl, F_ext, dt, F_side=0, linear=False): 
    """
    process model
    input: z: state z at time k, z := [X, Y, phi, v_x, v_y, r]
           u: input u at time k, u := [d_f, dvxdt]
           vhMdl: vehicle model
           trMdl: tire model
           F_ext: friction/resistance
           F_side: the side force on the vehicle, direction: Y axis
           dt: timestep length
    output: state at next time step z[k+1]
            a_F: slip angle (angle btw tire speed and orientation)
    """
    
    # get states / inputs
    X     = z[0]
    Y     = z[1]
    phi   = z[2]
    v_x   = z[3]
    v_y   = z[4]
    r     = z[5]

    d_f   = u[0]
    dvxdt = u[1]

    # prevent negative longitudinal speed
    v_x_next    = v_x + dt * dvxdt
    if v_x_next < 0:
        dvxdt = - v_x / dt
        v_x_next = 0

    # extract parameters
    (a, b, m, I_z)          = vhMdl
    (a0, Crr)               = F_ext
    (trMdlFront, trMdlRear) = trMdl
    (Bf, Cf, Df, Ef)        = trMdlFront
    (Br, Cr, Dr, Er)        = trMdlRear
    g                       = 9.81
    Fnr                     = m*g*a/(a+b)
    Fnf                     = m*g*b/(a+b)        

    # comptue the front/rear slip  [rad/s]
    # ref: Hindiyeh Thesis, p58
    if v_x >= 0.5:
        a_F = arctan((v_y + a*r)/v_x) - d_f
        a_R = arctan((v_y - b*r)/v_x)
    else:
        a_F = 0
        a_R = 0

    if linear:
        Caf     = Bf * Cf * Df * Fnf
        Car     = Br * Cr * Dr * Fnr 
        FyF     = - Caf * a_F
        FyR_paj = - Car * a_R

    else:
        TM_param_f  = [Bf, Cf, Df*Fnf, Ef]
        TM_param_r  = [Br, Cr, Dr*Fnr, Er]
        FyF         = -f_pajecka(TM_param_f, a_F)
        FyR_paj     = -f_pajecka(TM_param_r, a_R)
    
    FxR = (dvxdt+sign(v_x)*a0*v_x**2+(v_x!=0)*sign(v_x)*Crr*m*g-r*v_y)*m+FyF*sin(d_f)
    FyR_max     = sqrt(abs((Dr*Fnr)**2 - FxR**2))
    Fy          = array([FyR_max, FyR_paj])
    idx         = argmin(abs(Fy))
    FyR         = Fy[idx]

    # compute next state
    X_next      = X + dt*(v_x*cos(phi) - v_y*sin(phi)) 
    Y_next      = Y + dt*(v_x*sin(phi) + v_y*cos(phi)) 
    phi_next    = phi + dt*r
    dvydt       = -r*v_x + 1/m*(FyF*cos(d_f) + FyR + F_side*cos(phi))
    drdt        = 1/I_z*(a*FyF*cos(d_f) - b*FyR)
    v_y_next    = v_y + dt * dvydt
    r_next      = r   + dt * drdt

    return array([X_next, Y_next, phi_next, v_x_next, v_y_next, r_next])#, a_F

def f_pajecka(trMdl, alpha):
    """
    f_pajecka = d*sin(c*atan(b*alpha))    
    
    inputs :
        * trMdl := tire model, a list or tuple of parameters (b,c,d)
        * alpha := tire slip angle [radians]
    outputs :
        * Fy := lateral force from tire [Newtons]
    """
    (b, c, d, e) = trMdl

    return d*sin(c*arctan(b*alpha-e*(b*alpha-arctan(b*alpha)))) 