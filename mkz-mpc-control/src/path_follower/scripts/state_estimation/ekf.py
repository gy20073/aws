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
# Revised by: Chen Tang 2017.08.06

from numpy import array, dot, eye, copy
from numpy import dot, zeros
from numpy.linalg import inv
import rospy

def ekf(f, mx_k, mw_k, mv_k, P_k, h, y_kp1, Q, R, args):
    """
     EKF   Extended Kalman Filter for nonlinear dynamic systems
     ekf(f,mx,P,h,z,Q,R) returns state estimate, x and state covariance, P 
     for nonlinear dynamic system:
               x_k+1 = f(x_k, w_k)
               y_k   = h(x_k, v_k)
     where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
           v ~ N(0,R) meaning v is gaussian noise with covariance R
    Inputs:    f: function handle for f(x)
               mx_k: "a priori" state estimate
               w_tilt_k: mean of process noise 
               P_k: "a priori" estimated state covariance
               h: fanction handle for h(x)
               y_kp1: current measurement
               Q: process noise covariance 
               R: measurement noise covariance
               args: additional arguments to f(x, *args)
    Output:    mx_km1: "a posteriori" state estimate
               P_km1: "a posteriori" state covariance
               
    Notation: mx_k = E[x_k] and my_k = E[y_k], where m stands for "mean of"
    """


    xDim    = mx_k.size                                      # dimension of the state
    mx_kp1  = f(mx_k, mw_k, *args)                           # predict next state
    A       = numerical_jac_x(f, mx_k, mw_k, *args)          # linearize process model about current state
    L       = numerical_jac_w(f, mx_k, mw_k, *args)          # linearize process model about process noise
    P_kp1   = dot(dot(A,P_k),A.T) + dot(dot(L,Q),L.T)        # proprogate variance
    my_kp1  = h(mx_kp1, mv_k, *args)                         # predict future output
    H       = numerical_jac_x(h, mx_kp1, mv_k, *args)        # linearize measurement model about predicted next state
    M       = numerical_jac_w(h, mx_kp1, mv_k, *args)        # linearize measurement model about measurement noise
    P12     = dot(P_kp1, H.T)                                # cross covariance
    K       = dot(P12, inv(dot(H,P12) + dot(dot(M, R),M.T))) # Kalman filter gain
    mx_km1  = mx_kp1 + dot(K,(y_kp1 - my_kp1))               # state estimate
    P_km1   = dot(dot(K,R),K.T) + dot( dot( (eye(xDim) - dot(K,H)) , P_kp1)  ,  (eye(xDim) - dot(K,H)).T ) 

    return (mx_km1, P_km1)


    
def numerical_jac_x(f, x, w, *args):
    """
    Function to compute the numerical jacobian of a vector valued function 
    using finite differences
    """
    # numerical gradient and diagonal hessian
    y = f(x, w, *args)
    
    jac = zeros( (y.size,x.size) )
    eps = 1e-5
    xp = copy(x)
    
    for i in range(x.size):
        xp[i] = x[i] + eps/2.0
        yhi = f(xp, w, *args)
        xp[i] = x[i] - eps/2.0
        ylo = f(xp, w, *args)
        xp[i] = x[i]
        jac[:,i] = (yhi - ylo) / eps
    return jac


    
def numerical_jac_w(f, x, w, *args):
    """
    Function to compute the numerical jacobian of a vector valued function 
    using finite differences
    """
    # numerical gradient and diagonal hessian
    y = f(x, w, *args)
    
    jac = zeros( (y.size,w.size) )
    eps = 1e-5
    wp = copy(w)
    
    for i in range(w.size):
        wp[i] = w[i] + eps/2.0
        yhi = f(x, wp, *args)
        wp[i] = w[i] - eps/2.0
        ylo = f(x, wp, *args)
        wp[i] = w[i]
        jac[:,i] = (yhi - ylo) / eps
    return jac

