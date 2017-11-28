# -*- coding: utf-8 -*-
"""
Created on Thu Jul 20 10:28:29 2017

@author: Dogan
"""

import numpy as np
import scipy as sci
from scipy.optimize import fsolve
import serial
import time


    
def posit_find(ang,*coord):
    x=coord[0]
    y=coord[1]
    z=coord[2]
    
    a_S=ang[0]
    a_E=ang[1]
    a_B=ang[2]
   
    l1=10.5 #cm
    l2=13 #cm
    
    
    
    r=l1*np.cos(a_S)+l2*np.sin(a_E)
    z2=l1*np.sin(a_S)-l2*np.cos(a_E)
    x2=r*np.sin(np.pi/2-a_B)
    y2=r*np.cos(np.pi/2-a_B)
    
    h=[x-x2]
    h.append(y-y2)
    h.append(z-z2)
    
    return h
    
def calc_steps(current_ang,coord):
        
    a0=[np.pi/2, np.pi/2, np.pi/2]
    aa=fsolve(posit_find, a0, args=coord)
  
    a_S=aa[0]
    a_E=aa[1]
    a_B=aa[2]
    
    aa_rad=[a_S, a_E, a_B]
    ang=np.round(np.rad2deg(aa_rad))%360
    a2g=ang-current_ang;
    print(a2g)
    
    s2t=[0,0,0]
    s2t=np.round(a2g*-11.4)
#    if a2g[0]>0:
#        s2t[0]=np.round((a2g[0])*(11.4))
#    elif a2g[0]<0:
#        s2t[0]=-np.round((a2g[0])*(11.4))
    s2t[1]=-s2t[1]
#
#    if a2g[2]>0:
#        print('larger')
#        s2t[2]=-np.round((a2g[2])*(11.4))
#    elif a2g[2]<0:
#        s2t[2]=np.round((a2g[2])*(11.4))

    return s2t, ang#, a2g

def send_inputs(device,coord,cur_ang):
    
    s2t , ang = calc_steps(cur_ang,coord)
    
    steps= s2t
    print(steps)
    device.reset_input_buffer()
    time.sleep(0.18)
    device.write("s,{:.0f}\n".format(steps[0]).encode('ascii'))
    time.sleep(0.18)
    device.write("e,{:.0f}\n".format(steps[1]).encode('ascii'))
    time.sleep(0.18)
    device.write("b,{:.0f}\n".format(steps[2]).encode('ascii'))
    time.sleep(0.18)

    return ang
    
def go_between(device,c1,c2,ang):
    
    ang=send_inputs(device,c1,ang)
    cr=list(c1)
    
    while cr!=list(c2):
        if c2[0]>round(cr[0],4):
            cr[0]=round(cr[0]+0.1,4)
        elif c2[0]<round(cr[0],3):
            cr[0]=round(cr[0]-0.1,4)
            
        if c2[1]>round(cr[1],3):
            cr[1]=round(cr[1]+0.1,4)
        elif c2[1]<round(cr[1],3):
            cr[1]=round(cr[1]-0.1,4)
            
        if c2[2]>round(cr[2],3):
            cr[2]=round(cr[2]+0.1,4)
        elif c2[2]<round(cr[2],3):
            cr[2]=round(cr[2]-0.1,4)
            
        print(cr)
        #x_pos=read_x_pos(device)
       # print(x_pos)
        ang=send_inputs(device,tuple(cr),ang)
    return ang

def draw_square (device,l,c1,ang):
    
    ang=send_inputs(device,tuple(c1),ang)
    c1=list(c1)
    
    c2=[sum(x) for x in zip(c1,[l,0,0])]
    ang=go_between(device,tuple(c1),tuple(c2),ang)
    
    c3=[sum(x) for x in zip(c1,[l,l,0])]
    ang=go_between(device,tuple(c2),tuple(c3),ang)

    c4=[sum(x) for x in zip(c1,[0,l,0])]
    ang=go_between(device,tuple(c3),tuple(c4),ang)
    
    ang=go_between(device,tuple(c4),tuple(c1),ang)
   
    return ang

def draw_line_x(device,c1,l):
    
    ang=send_inputs(device,c1)
    c1=list(c1)
    
    c2=[sum(x) for x in zip(c1,[l,0,0])]
    go_between(device,tuple(c1),tuple(c2))


def draw_line_y(device,c1,l):
     ang=send_inputs(device,c1)
     c1=list(c1)
    
     c2=[sum(x) for x in zip(c1,[0,l,0])]
     go_between(device,tuple(c1),tuple(c2))
     
def read_x_pos(device):
    r=[0]
    while len(r)!=4:
        line=device.readlines(10)
        r=line[0].split(",")  
 
#     
    return [float(r[0]),float(r[1]),float(r[2])]
    
c1=(150, -30, 20)
c2=(180, -30, 20)
c3=(180, 0, 20)
c4=(150, 0, 20)

#r=calc_angle(c1)
#send_inputs(ardu,coord)

#ardu = serial.Serial('COM3', 38400, timeout=1)
#ardu = serial.Serial('COM3', 115200, timeout=1)

def draw_cal_logo(device,ang):
    import csv
    with open('cal_logo6.csv', 'rb') as f:
        reader = csv.reader(f)
        data_as_list = list(reader)
        
        c_0=[float(l) for l in data_as_list[0]]
        ang=send_inputs(device,tuple(c_0),ang)
        time.sleep(20)

    for i in range(len(data_as_list)-1):
        c_k=[float(l) for l in data_as_list[i]]
        #c_k_1=[float(l) for l in data_as_list[i+1]]
        #ang=go_between(device,tuple(c_k),tuple(c_k_1),ang)
        ang=send_inputs(device,tuple(c_k),ang)
        #time.sleep(0.3)
        
    return ang
                