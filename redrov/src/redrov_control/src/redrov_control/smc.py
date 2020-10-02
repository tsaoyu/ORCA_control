#!/usr/bin/python
#
# This file is part of IvPID.
# Copyright (C) 2015 Ivmech Mechatronics Ltd. <bilgi@ivmech.com>
#
# IvPID is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# IvPID is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# title           :smc.py
# description     :python sliding mode controller
# author          :Lin Bin
# date            :20200914 updated 20200914
# version         :0.1
# notes           :
# python_version  :2.7
# ==============================================================================

"""

"""
import math
import numpy as np
import pandas as pd
import time
def Norm2(array):
    num=0
    if(array.ndim==2):
        rows,cols=array.shape
        if(rows==1):
            num=0
            for i in range(cols):
                num+=array[0][i]*array[0][i]
        if(cols==1):
            num=0
            for i in range(rows):
                num+=array[i][0]*array[i][0]
    else:
        num=0
        for i in range(len(array)):
                num+=array[i]*array[i]
    num=math.sqrt(num)
    return num
def angleCheck(angle):
    while(angle<-math.pi):
        angle+=math.pi*2
    while(angle> math.pi):
        angle-=math.pi*2
    return angle

class SMC:
    """Sliding Mode Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None, angle_error=False):
        ''' receive reference state: eta (x,y,z,phi,theta,psi)
                and current state: eta_d (x,y,z,phi,theta,psi) '''
        self.current_time = time.time()
        self.begin_time=time.time()
        self.last_time = self.current_time
       
        self.eta    =np.zeros(6)        # current position   (x,y,z,phi,theta,psi) earth frame
        self.velocity=np.zeros(6)       # current velocity   (u,v,w,p,q,r) body-fixed frame
        self.eta_d=np.zeros(6)          # reference position (x,y,z,phi,theta,psi)
        self.last_eta_d=np.zeros(6)         # previous reference position (x,y,z,phi,theta,psi)
        self.eta_d_dot=np.zeros(6)      # differential of reference position [m/s,m/s,m/s,rad/s,rad/s,rad/s]
       
        self.v_c=np.zeros(6)
        self.last_v_c=np.zeros(6)
        self.v_c_dot=np.zeros(6)

        self.e_eta=np.zeros(6)
        self.e_v  =np.zeros(6)
        self.last_e_v=np.zeros(6)
        self.e_v_integral=np.zeros(6)

        self.tau_v=np.zeros(6)

        self.Jacobian=np.zeros((6,6))
        
        self.data_arry1=[]      # data to be saved
        self.data_arry2=[]
        self.data_arry3=[]
        self.SetParams()
    def SetParams(self):
        self.M_hat=np.zeros((6,6))                        # contains the mass, inertia, and the hydrodynamic added mass
        self.inM_hat=np.zeros((6,6))       # inverse of M_hat  
        self.C_hat=np.zeros((6,6))      # collects the Coriolis and centripetal terms
        self.D_hat=np.zeros((6,6))      # stands for the linear and quadratic damping matrix
        self.g_hat=np.zeros(6)          # is the vector of restoring forces (i.e., gravity and buoyancy)
        
        self.k=1                            # (>0) constant positive value
        self.K=np.diag([1,1,1,1,1,1])          # constant and positive definite diagonal matrix(6*6)
        self.Gamma=np.diag([1,1,1,1,1,1])                        # (>0)is a constant and positive definite diagonal matrix that needs to be defined by the users
        self.gammas =[1,1,1]                # (>0) constant design gain
        self.lambdas=[0.01,0.01,0.01]                # initial value (>0) of lambda is positive
        
        self.lambdas_dot=[0,0,0]        
    def UpdateJacobian(self):
        ''' Jacobian transformation matrix
            J=[[J1, 0],[0, J2]]] in R(6*6)
             self.eta: (x,y,z,phi,theta,psi) '''
        J=np.zeros((6,6))
        phi,theta,psi=self.eta[3:6]
        cphi  =math.cos(phi)
        ctheta=math.cos(theta)
        cpsi  =math.cos(psi)
        sphi  =math.sin(phi)
        stheta=math.sin(theta)
        spsi  =math.sin(psi)
        ttheta=math.tan(theta)
        J[0:3,0:3]=[[ cpsi*ctheta,-spsi*cphi+cpsi*stheta*sphi, spsi*sphi+cpsi*cphi*stheta],
                    [ spsi*ctheta, cpsi*cphi+spsi*stheta*sphi,-cpsi*sphi+spsi*cphi*stheta],
                    [-stheta     , ctheta*sphi               , ctheta*cphi]] # J1
        J[3:6,3:6]=[[1, sphi*ttheta, cphi*ttheta],
                    [0, cphi       ,-sphi],
                    [0, sphi/ctheta, cphi/ctheta]]  # J2
        self.Jacobian=J  # array
    def UpdateMCDg_hat(self):
        ''' update  dynamics parameters
            M_hat & inM_hat: constant
            C_hat: needs velocity     
            D_hat: needs velocity  
            g_hat: needs eta & velocity '''
        W, B = 112.8, 114.8
        m,xG,yG,zG = 11.2, 0, 0, 0.08
        Ix,Ixy,Ixz = 0.16, 0, 0
        Iy,Iyx,Iyz = 0.16, 0, 0
        Iz,Izx,Izy = 0.16, 0, 0
        Xudot,Yvdot,Zwdot = 5.5,12.7,14.57
        Kpdot,Mqdot,Nrdot = 0.12,0.20,0.24
        Xu,Xuu = 4.03,18.18  
        Yv,Yvv = 6.22,21.66 
        Zw,Zww = 5.18,36.99
        Kp,Kpp = 3.07,0.45
        Mq,Mqq = 3.07,0.45
        Nr,Nrr = 4.64,0.43
        Mrb = np.array([[m, 0, 0, 0, m * zG, -m * yG],
                        [0, m, 0, -m * zG, 0, m * xG],
                        [0, 0, m, m * yG, -m * xG, 0],
                        [0, -m * zG, m * yG, Ix, -Ixy, -Ixz],
                        [m * zG, 0, -m * xG, -Iyx, Iy, -Iyz],
                        [-m * yG, m * xG, 0, -Izx, -Izy, Iz]])
        Ma = np.diag([Xudot, Yvdot, Zwdot, Kpdot, Mqdot, Nrdot])
        M = Mrb + Ma
        self.M_hat=M                        # contains the mass, inertia, and the hydrodynamic added mass
        self.inM_hat=np.linalg.inv(M)       # inverse of M_hat  
        [x, y, z, phi, theta, psi] = self.eta 
        [u, v, w, p, q, r]         = self.velocity
        cpsi = np.cos(psi)
        spsi = np.sin(psi)
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        cth = np.cos(theta)
        sth = np.sin(theta)
        Crb = np.array([[0, 0, 0,  0, m * w, -m * v],
                        [0, 0, 0, -m * w, 0, m * u],
                        [0, 0, 0,  m * v, -m * u, 0],
                        [0, m * w, -m * v, 0, Iz * r, -Iy * q],
                        [-m * w, 0, m * u, -Iz * r, 0, Ix * p],
                        [m * v, -m * u, 0, Iy * q, -Ix * p, 0]])
        Ca = np.array([[0, 0, 0, 0, - Zwdot * w, 0],
                       [0, 0, 0, Zwdot * w, 0, -Xudot * u],
                       [0, 0, 0, -Yvdot * v, Xudot * u, 0],
                       [0, -Zwdot * w, Yvdot * v, 0, -Nrdot * r, Mqdot * q],
                       [Zwdot * w, 0, -Xudot * u, Nrdot * r, 0, -Kpdot * p],
                       [-Yvdot * v, Xudot * u, 0, -Mqdot * q, Kpdot * p, 0]])
        self.C_hat=Crb+Ca
        Dnu = np.diag([Xu, Yv, Zw, Kp, Mq, Nr])
        Dnl = np.diag([Xuu * abs(u), Yvv * abs(v), Zww * abs(w),
                   Kpp * abs(p), Mqq * abs(q), Nrr * abs(r)])
        self.D_hat=Dnu+Dnl
        if z < 0:
            geta = np.array( [ W  * sth,
                             - W  * cth * sphi,
                             - W  * cth * cphi,
                               zG * W * cth * sphi,
                               zG * W * sth,
                               0   ])
        else:
            geta = np.array([ (W - B) * sth,
                             -(W - B) * cth * sphi,
                             -(W - B) * cth * cphi,
                              zG * W * cth * sphi,
                              zG * W * sth,
                              0])
        self.g_hat=geta 
    def updateEtad(self,eta_d,eta_d_dot):
        ''' reference position: 
            eta_d: (x,y,z,phi,theta,psi) --- [m,m,m,rad,rad,rad]
            reference position differential: 
            eta_d_dot: (x,y,z,phi,theta,psi) --- [m,m,m,rad,rad,rad]'''
        self.eta_d=eta_d
        self.eta_d_dot=eta_d_dot
    def UpdateErrors(self):
        ''' error systems
            e_eta: position tracking errors (x,y,z,phi,theta,psi) -- [m,  m,  m,  rad,  rad,  rad  ]
            e_v  : velocity tracking errors (u,v,w,p,q,r)         -- [m/s,m/s,m/s,rad/s,rad/s,rad/s]
            e_v_integral: integral of e_v
            '''
        delta_time=self.current_time-self.last_time
        self.e_eta=self.eta-self.eta_d
        self.e_eta[3]=angleCheck(self.e_eta[3])
        self.e_eta[4]=angleCheck(self.e_eta[4])
        self.e_eta[5]=angleCheck(self.e_eta[5])

        self.UpdateJacobian()                   # Jacobian matrix [6*6]
        J_1=np.linalg.inv(self.Jacobian)                     # inverse of matrix J [6*6]
        self.v_c=np.dot(J_1,(self.eta_d_dot-self.k*self.e_eta)) # [6,]
        self.v_c_dot=(self.v_c-self.last_v_c)/delta_time
        self.e_v=self.velocity-self.v_c
        self.e_v_integral+=(self.e_v+self.last_e_v)/2.0*delta_time
        
        self.last_v_c=self.v_c
        self.last_e_v=self.e_v

    def UpdateS(self): # undo
        ''' sliding variable vector
            '''
        self.s=self.e_v+np.dot(self.K,self.e_v_integral)
    def UpdateLambda(self):
        ''' estimates of lambda i (i=0,1,2)
            '''
        delta_time=self.current_time-self.last_time
        for i in range(3):
            self.lambdas_dot[i]=Norm2(self.s)*np.linalg.det(self.inM_hat)*math.pow(Norm2(self.velocity),i)-self.gammas[i]*self.lambdas[i]
        for i in range(3):
            self.lambdas[i]+=self.lambdas_dot[i]*delta_time
    def UpdateForceCmd(self):
        ''' input forces and moments
            '''
        kesi=np.linalg.det(self.inM_hat)*(self.lambdas[0]+self.lambdas[1]*Norm2(self.velocity)+self.lambdas[2]*math.pow(Norm2(self.velocity),2))
        self.tau_v=np.dot(self.C_hat,self.velocity)+np.dot(self.D_hat,self.velocity)+self.g_hat+np.dot(self.M_hat,self.v_c_dot)-np.dot(np.dot(self.M_hat,self.K),self.e_v)-np.dot(self.Gamma,self.s)-kesi*np.sign(self.s)
    def savedata(self):
        ''' svas data'''
        data1_row=[self.current_time-self.begin_time]
        data1_row.extend(self.e_v)
        data1_row.extend(self.e_v_integral)
        self.data_arry1.append(data1_row)
        columns1=['time']
        columns1.extend(['e_v','1','2','3','4','5'])
        columns1.extend(['e_v_integral','1','2','3','4','5'])
        save1 = pd.DataFrame(self.data_arry1,columns=columns1)
        save1.to_csv('~/rov_ws/src/smc_control/data/data1.csv',index=False,header=True) 

        data2_row=[self.current_time-self.begin_time]
        data2_row.extend(self.eta_d)
        data2_row.extend(self.eta_d_dot)
        self.data_arry2.append(data2_row)
        columns2=['time']
        columns2.extend(['eta_d','1','2','3','4','5'])
        columns2.extend(['eta_d_dot','1','2','3','4','5'])     
        save2 = pd.DataFrame(self.data_arry2,columns=columns2)        
        save2.to_csv('~/rov_ws/src/smc_control/data/data2.csv',index=False,header=True) 

        data3_row=[self.current_time-self.begin_time]
        data3_row.extend(self.e_eta)
        data3_row.extend(self.tau_v)
        self.data_arry3.append(data3_row)
        columns3=['time']
        columns3.extend(['e_eta','1','2','3','4','5'])
        columns3.extend(['tau_v','1','2','3','4','5'])     
        save3 = pd.DataFrame(self.data_arry3,columns=columns3)        
        save3.to_csv('~/rov_ws/src/smc_control/data/data3.csv',index=False,header=True) 

    def update(self,eta,velocity):
        ''' update main variable & parameters
            update current  state
                eta:        (x,y,z,phi,theta,psi) -- [m,m,m,rad,rad,rad]
                velocity:   (u,v,w,p,q,r)         -- [m/s,m/s,m/s,rad/s,rad/s,rad/s]
                '''
        self.current_time =  time.time()

        self.eta=eta
        self.velocity=velocity

        self.UpdateErrors()     # including self.UpdateJacobian()
        self.UpdateMCDg_hat()
        self.UpdateS()
        self.UpdateLambda() 
        self.UpdateForceCmd()
        self.savedata()

        self.last_time = self.current_time