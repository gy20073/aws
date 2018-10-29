from numpy import sin, cos, tan, arctan, array, dot

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
