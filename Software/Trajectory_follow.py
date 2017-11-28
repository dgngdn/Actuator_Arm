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
   
    l1=170
    l2=170
    
    r=l1*np.cos(np.pi-a_S)+l2*np.sin(np.pi/2-a_S+a_E)
    z2=l1*np.sin(np.pi-a_S)-l2*np.cos(np.pi/2-a_S+a_E)
    x2=r*np.sin(a_B)
    y2=r*np.cos(a_B)
    
    h=[x-x2]
    h.append(y-y2)
    h.append(z-z2)
    
    return h
    
def calc_angle(coord):
        
    a0=[np.pi/2, np.pi/2, np.pi/2]
    aa=fsolve(posit_find, a0, args=coord)
  
    a_S=aa[0]
    a_E=aa[1]
    a_W=np.pi/2+np.pi/2-(np.pi/2-a_S+a_E)
    a_B=aa[2]
    
    aa_rad=[a_S, a_E, a_W, a_B]
    aa_deg=np.round(np.rad2deg(aa_rad))
    
#    input_string= "%03d%03d%03d%03dn" % (aa_deg[0],aa_deg[1],aa_deg[2],aa_deg[3])
    return aa_deg

def send_inputs(device,coord):
    
    angles=calc_angle(coord)
    device.reset_input_buffer()
    time.sleep(0.300)
    device.write("s,{:.0f}\n".format(angles[0]).encode('ascii'))
    time.sleep(0.300)
    device.write("e,{:.0f}\n".format(angles[1]).encode('ascii'))
    #time.sleep(0.300)
    #device.write("w,{:.0f}\n".format(angles[2]).encode('ascii'))
    time.sleep(0.300)
    device.write("b,{:.0f}\n".format(angles[3]).encode('ascii'))

    return angles
    
def go_between(device,c1,c2):
    
    send_inputs(device,c1)
    cr=list(c1)
    
    while cr!=list(c2):
        if c2[0]>cr[0]:
            cr[0]=cr[0]+5
        elif c2[0]<cr[0]:
            cr[0]=cr[0]-5
            
        if c2[1]>cr[1]:
            cr[1]=cr[1]+5
        elif c2[1]<cr[1]:
            cr[1]=cr[1]-5
            
        if c2[2]>cr[2]:
            cr[2]=cr[2]+5
        elif c2[2]<cr[2]:
            cr[2]=cr[2]-5
            
        print(cr)
        #x_pos=read_x_pos(device)
       # print(x_pos)
        send_inputs(device,tuple(cr))
         

def draw_square (device,l,c1):
    
    send_inputs(device,c1)
    c1=list(c1)
    
    c2=[sum(x) for x in zip(c1,[l,0,0])]
    go_between(device,tuple(c1),tuple(c2))
    
    c3=[sum(x) for x in zip(c1,[l,l,0])]
    go_between(device,tuple(c2),tuple(c3))

    c4=[sum(x) for x in zip(c1,[0,l,0])]
    go_between(device,tuple(c3),tuple(c4))
    
    go_between(device,tuple(c4),tuple(c1))


def draw_line_x(device,c1,l):
    
    send_inputs(device,c1)
    c1=list(c1)
    
    c2=[sum(x) for x in zip(c1,[l,0,0])]
    go_between(device,tuple(c1),tuple(c2))


def draw_line_y(device,c1,l):
     send_inputs(device,c1)
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

r=calc_angle(c1)
#send_inputs(ardu,coord)

#ardu = serial.Serial('COM3', 38400, timeout=1)
#ardu = serial.Serial('COM3', 115200, timeout=1)
