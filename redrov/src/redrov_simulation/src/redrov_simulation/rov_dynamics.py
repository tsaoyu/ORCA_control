import numpy as np

m = 11.2
xG = 0
yG = 0
zG = 0.08
Ix = 0.16
Ixy = 0
Ixz = 0
Iyx = 0
Iy = 0.16
Iyz = 0
Izx = 0
Izy = 0
Iz = 0.16

Xudot = 5.5  # Abir 2.6
Yvdot = 12.7  # 18.5
Zwdot = 14.57
Kpdot = 0.12
Mqdot = 0.12
Nrdot = 0.12

Xu = 4.03
Xuu = 18.18  # Abir Xu = 0;    Xuu = 34.96;
Yv = 6.22
Yvv = 21.66  # Yv = 0.26; Yvv = 103.25;
Zw = 5.18
Zww = 36.99
Kp = 0.07
Kpp = 1.55
Mq = 0.07
Mqq = 1.55
Nr = 0.07
Nrr = 1.55 # Nr = 4.64; Nrr = 0.43;

Mrb = np.array([[m, 0, 0, 0, m * zG, -m * yG],
                    [0, m, 0, -m * zG, 0, m * xG],
                    [0, 0, m, m * yG, -m * xG, 0],
                    [0, -m * zG, m * yG, Ix, -Ixy, -Ixz],
                    [m * zG, 0, -m * xG, -Iyx, Iy, -Iyz],
                    [-m * yG, m * xG, 0, -Izx, -Izy, Iz]])

Ma = np.diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot])

M = Mrb + Ma
inM = np.linalg.inv(M)




def BlueROV2Dynamic(t, y, tau, print_info=False):
    W = 112.8
    B = 114.8

    # Unpack states
    nu = y[:6]
    eta = y[6:]
    [u, v, w, p, q, r] = nu
    [N, E, D, phi, theta, psi] = eta

    
    cpsi = np.cos(psi)
    spsi = np.sin(psi)
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    cth = np.cos(theta)
    sth = np.sin(theta)


    Crb = np.array([[0, 0, 0, 0, m * w, -m * v],
                    [0, 0, 0, -m * w, 0, m * u],
                    [0, 0, 0, m * v, -m * u, 0],
                    [0, m * w, -m * v, 0, Iz * r, -Iy * q],
                    [-m * v, 0, m * u, -Iz * r, 0, Ix * p],
                    [m * v, -m * u, 0, Iy * q, -Ix * p, 0]])

    Ca = np.array([[0, 0, 0, 0, Zwdot * w, 0],
                   [0, 0, 0, -Zwdot * w, 0, -Xudot * u],
                   [0, 0, 0, -Yvdot * v, Xudot * u, 0],
                   [0, -Zwdot * w, Yvdot * v, 0, -Nrdot * r, Mqdot * q],
                   [Zwdot * w, 0, -Xudot * u, Nrdot * r, 0, -Kpdot * p],
                   [-Yvdot * v, Xudot * u, 0, -Mqdot * q, Kpdot * p, 0]])

    Dnu = np.diag([Xu, Yv, Zw, Kp, Mq, Nr])
    Dnl = np.diag([Xuu * abs(u), Yvv * abs(v), Zww * abs(w),
                   Kpp * abs(p), Mqq * abs(q), Nrr * abs(r)])

    if D < 0:
        geta = np.array( [ W  * sth,
                         - W  * cth * sphi,
                         - W  * cth * cphi,
                           zG * W* cth * sphi,
                           zG * W * sth,
                           0   ])
    else:
        geta = np.array([(W - B) * sth,
                         -(W - B) * cth * sphi,
                         -(W - B) * cth * cphi,
                         zG * W * cth * sphi,
                         zG * W * sth,
                         0])

    nudot = inM.dot(tau - (Crb + Ca).dot(nu) - (Dnu + Dnl).dot(nu) - geta)

    J = np.array([[cpsi * cth, - spsi * cphi + cpsi * sth * sphi, spsi * sphi + cpsi * cphi * sth, 0, 0, 0],
                  [spsi * cth, cpsi * cphi + sphi * sth * spsi, - cpsi * sphi + sth * spsi * cphi, 0, 0, 0],
                  [-sth, cth * sphi, cth * cphi, 0, 0, 0],
                  [0, 0, 0, 1, sphi * sth / cth, cphi * sth / cth],
                  [0, 0, 0, 0, cphi, -sphi],
                  [0, 0, 0, 0, sphi / cth, cphi / cth]])

    etadot = J.dot(nu)

    if print_info:
        return { 'u': u, 'v': v, 'w': w, 'p': p, 'q': q, 'r': r,
                'N': N, 'E': E, 'D': D, 'psi': psi, 'theta': theta, 'phi': phi,
                'Coriolis': (Crb + Ca).dot(nu), 'Damping': (Dnu + Dnl).dot(nu),
                'Restoring': geta, 'TotalForce': tau - (Crb + Ca).dot(nu) - (Dnu + Dnl).dot(nu) - geta,
                'Acceleration': nudot, 'time': t}


    ydot = nudot.tolist()
    ydot.extend(etadot.tolist())
    return ydot
