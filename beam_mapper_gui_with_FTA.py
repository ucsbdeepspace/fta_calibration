# -*- coding: utf-8 -*-
"""
Created on Tue Apr 16 13:53:45 2019
connects to rigol electronic load,
sets current, reads voltage, makes plot, saves data.
http://alexforencich.com/wiki/en/python-vxi11/readme
Add command line inputs for filenames etc
@author: Pain
"""

import vxi11
import numpy as np
import time
import matplotlib
import matplotlib.pyplot as plt
import csv
import os
from tkinter import *
from time import sleep
import h5py
import serial.tools.list_ports
ports= serial.tools.list_ports.comports()
for p in ports:
    print('p: ', p)


steps_per_mm=316
xg60_area = np.pi*(7.9/2.)**2  # mm^2
lm3_area = np.pi*(19./2.)**2  # mm^2

default_params={}
default_params['x_scan_size']=200
default_params['y_scan_size']=200
default_params['y_step']=5
default_params['step_delay']=100
default_params['range'] = 2.5
default_params['TXT_prefix']='sm_telescope'
default_params['Folder']='C:\\Starshot\\beam_measurements'
default_params['Wavelength']=1064

paramstxt=['X_scan_size','y_scan_size','y_step','step_delay','range','TXT_prefix','Folder','Wavelength']
paramskeys=paramstxt


window = Tk()
window.title("2d Beam Mapper Control")
window.geometry('500x500')   


def get_pyboard_fm(ports):
    #go thru ports, find and open ser_pb and ser_fm
    ser_pb_STP=-1
    ser_fm=-1
    ser_pb_FTA=-1
    for p in ports:
        print('p: ', p)
        if 'USB Serial Device' in p[1]:
            print('found serial device')
            try:
                ser_pb = serial.Serial( p.device, 115200, timeout=.1)
                string_to_send = 'pw?\r'
                b = bytes(string_to_send, 'utf-8')
                ser_pb.write( b )
                data_read = ser_pb.readline()
                if data_read:
                    print('read back from board: ', data_read)
                    if 'FTA' in str(data_read,'utf-8').rstrip('\r\n'):
                            print('Found FTA pyboard')
                            ser_pb.close()
                            ser_pb_FTA = serial.Serial( p.device, 115200, timeout=.1)
                    if 'STP' in str(data_read,'utf-8').rstrip('\r\n'):
                            print('Found STP pyboard')
                            ser_pb.close()
                            ser_pb_STP = serial.Serial( p.device, 115200, timeout=.1)
                            
            except:
                continue
        if 'USB Serial Port' in p[1]:
            print('found serial port')
            try:
                ser_fm = serial.Serial( p.device, 9600, timeout=.1)
                string_to_send = 'pw?\r'
                b = bytes(string_to_send, 'utf-8')
                ser_fm.write( b )
                data_read = ser_fm.readline()
                if data_read:
                    print('read back from board: ', data_read)
            except:
                continue
    return ser_pb_STP,ser_fm,ser_pb_FTA

def read_power(ser):
    '''dumb read of power meter. maybe fragile and need error checking'''
    string_to_send = 'pw?\r'
    b = bytes(string_to_send, 'utf-8')
    ser.write( b )
    while True:
        data_read = ser.readline()
        if data_read:
            #print('read back from board: ', data_read)
            break
    dataval = float(str(data_read,'utf-8').rstrip('\r\n'))
    return dataval

def move(ser,axis, steps, delay):
    if axis == 0:
        string_to_send = 'X,'
    else:
        string_to_send = 'Y,'
    string_to_send += str( steps ) + ',' + str(delay) + '\r\n'
    b = bytes(string_to_send, 'utf-8')
    ser.write( b )
    ser_pb_STP.flushInput()
    not_done_moving=True
    return 

def move_wait(ser,axis, steps, delay):
    if axis == 0:
        string_to_send = 'X,'
    else:
        string_to_send = 'Y,'
    string_to_send += str( steps ) + ',' + str(delay) + '\r\n'
    b = bytes(string_to_send, 'utf-8')
    ser.write( b )
    ser_pb_STP.flushInput()
    not_done_moving=True
    while(not_done_moving):
        bytes_to_read=ser_pb_STP.in_waiting
        if bytes_to_read>5:
            not_done_moving=False
            sleep(.1)
    return 

def smoothscan1d(axis=0, distance=400,delay=30,ser_pb_STP=None,ser_fm=None):
    """define smooth scan, just start stepper and measure power as fast as possible, correlate after, distance units are mm, returns dictionary with position and data"""
    steps=distance*steps_per_mm
    powers=[]
    ser_pb_STP.flushInput()
    move(ser_pb_STP,axis,steps,delay)
    not_done_moving=True
    while(not_done_moving):
        #ser_pb_STP.flushInput()
        pwr=read_power(ser_fm)
        #print(pwr)
        powers.append(pwr)
        bytes_to_read=ser_pb_STP.in_waiting
       # print(ser_pb_STP.in_waiting)
        if bytes_to_read>5:
            not_done_moving=False
            sleep(.1)
    ddict={}
    ddict['power']=powers
    print(len(powers))
    if distance > 0: 
        ddict['position'] = list(np.linspace(0,distance,num=len(powers)))
    else:
        ddict['position'] = list(np.linspace(abs(distance),0,num=len(powers)))
    return ddict

def smoothscan2d(xdistance=600,ydistance=600,ystepsize=20,delay=30,ser_pb_STP=None,ser_fm=None):
    '''hardwire to full 600 mm scans both dimensions'''
    #ydistance=600
    #ystepsize=20
    #xdistance=600
    ddict={}
    ddict['xposition']=[]
    ddict['yposition']=[]
    ddict['power'] = []
    i=0 #index to see which direction to run x scan
    stepsum=0
    for ypos in range(0,int(ydistance+ystepsize),int(ystepsize)):  #needed to add +1 to get full square
        ysteps=int(ystepsize*steps_per_mm)
        if i>0: 
            ser_pb_STP.reset_input_buffer()
            move_wait(ser_pb_STP,1,ysteps,100)
            pwr=read_power(ser_fm)
            print('power',pwr)
       
        ypos = stepsum/steps_per_mm
        dist=((-1)**(i))*xdistance
        print('distance = ',dist)
        ser_pb_STP.reset_input_buffer()
        ser_fm.reset_input_buffer()
        print('Scanning Y = ',ypos,' mm')
        scandata = smoothscan1d(0,dist,delay,ser_pb_STP,ser_fm)
        print('xlen',len(scandata['position']))
        ddict['power'] = ddict['power'] + scandata['power']
        ddict['xposition'] = ddict['xposition'] + scandata['position']
        ddict['yposition'] = np.concatenate([ddict['yposition'] ,(np.zeros(len(scandata['position']))+ypos)])
        
        i += 1
        stepsum += ysteps
        
    #move_wait(ser_pb_STP,1, np.int(-ydistance*steps_per_mm), 30)
    return ddict

def norm_and_center(ddict):
    #remove baseline, scale by max, move max point to zero in position
    x=ddict['position']
    y=ddict['power']
    outdict={}
    y=y-np.min(y)
    y=y/np.max(y)
    maxind=np.argmax(y)
    x=x-x[maxind]
    outdict['position']=x
    outdict['power']=y
    return outdict

def save_dict_to_h5(ddict,filename ='test.h5'):
    #use group with attributes to mock up dictionary, save to h5 file 'h5f'
    h5file=filename
    keys=ddict.keys()
    with h5py.File(h5file,'w') as h5f:
        grp = h5f.create_group('dictionary')
        for key in keys:
            grp.attrs[key]=ddict[key]
        h5f.close()

def load_dict_from_h5(filename='test.h5', initialdir='c://Starshot//power_beaming//Power_meter_beam_maps'):
    #if there is a group called 'dictionsary', read its attributes as key/value, otherwise
    #read directly from an h5 file with no groups, datasets convert to key/value dictionary
    h5file=initialdir+'\\'+filename
    with h5py.File (h5file,'r') as h5f:
        h5keys=list(h5f.keys())
        if 'dictionary' in h5keys:
            grp=h5f.get('dictionary')
            out_dict = {}
            for key in grp.attrs.keys():
                out_dict[key]= grp.attrs[key]
        else:
            out_dict={}
            for key in h5keys:
                out_dict[key]=h5f[key][:]
        h5f.close()
    return out_dict
    



def get_date_filename(text_string):
    '''default filename using date and time. 
    creates a directory for each day and returns the file string to be used'''    
    now=time.localtime()[0:6]
    dirfmt = "%4d_%02d_%02d"
    dirname = os.path.join(default_params['Folder'],dirfmt % now[0:3])
    if not os.path.exists(dirname):
        os.mkdir(dirname)
    filefmt = "%02d_%02d_%02d.h5"
    filename= text_string+filefmt % now[3:6]
    ffilename=os.path.join(dirname,filename)
    return(ffilename) 

def get_time_filename(folder,text_string):
    '''default filename using date and time. 
    creates a directory for each day and returns the file string to be used'''    
    now=time.localtime()[0:6]
    filefmt = "%02d_%02d_%02d.csv"
    filename= text_string+filefmt % now[3:6]
    ffilename=os.path.join(folder,filename)
    print(ffilename)
    return(ffilename) 

def get_date_dirname(initialdir='C:\Starshot\beam_measurements'):
    '''default directory using date and time. 
    creates a directory for each day and returns '''    
    now=time.localtime()[0:6]
    dirfmt = "%4d_%02d_%02d"
    dirname = os.path.join(initialdir,dirfmt % now[0:3])
    if not os.path.exists(dirname):
        os.mkdir(dirname)
    return(dirname)     


def start_xscan_callback():
        ftext=entry_prefix.get() + '_x_scan'
        xdistance_mm=float(entry_xdistance.get())
        step_delay=float(entry_delay.get())
        move_wait(ser_pb_STP,0,-1*xdistance_mm*steps_per_mm/2,50)
        print('finished x move')
        datadict_1d = smoothscan1d(axis=0,distance=xdistance_mm,delay=step_delay,ser_pb_STP=ser_pb_STP,ser_fm=ser_fm)
        print('finished x scan')
        move_wait(ser_pb_STP,0,-1*xdistance_mm*steps_per_mm/2,50)
        print('finished x move back to start')
        fname=get_date_filename(ftext)
        print(fname)
        save_dict_to_h5(datadict_1d,fname)
        #now plot also
        fig, ax = plt.subplots()
        ax.plot(datadict_1d['position'],1000*np.array(datadict_1d['power']),label='latest data')
        
        ax.set(xlabel='X Distance [mm]', ylabel='Measured power [mw]', title='Scan %s' %ftext)
        #ax.legend([], loc='best', title='Max power %s W at  %s A' %(str(max_power),str(max_power_current)))
        ax.grid()
        plt.show()
        #dirname=get_date_dirname()
        #figname=os.path.join(dirname,"%s_IV.png" %ftext)
        #fig.savefig(figname)
        plt.close()

def start_yscan_callback():
        ftext=entry_prefix.get() + '_y_scan'
        ydistance_mm=float(entry_ydistance.get())
        step_delay=float(entry_delay.get())
        move_wait(ser_pb_STP,1,-1*ydistance_mm*steps_per_mm/2,50)
        datadict_1d = smoothscan1d(axis=1,distance=ydistance_mm,delay=step_delay,ser_pb_STP=ser_pb_STP,ser_fm=ser_fm)
        move_wait(ser_pb_STP,1,-1*ydistance_mm*steps_per_mm/2,50)
        
        fname=get_date_filename(ftext)
        print(fname)
        save_dict_to_h5(datadict_1d,fname)
        #now plot also
        fig, ax = plt.subplots()
        ax.plot(datadict_1d['position'],1000*np.array(datadict_1d['power']),label='latest data')
        
        ax.set(xlabel='Y Distance [mm]', ylabel='Measured power [mw]', title='Scan %s' %ftext)
        #ax.legend([], loc='best', title='Max power %s W at  %s A' %(str(max_power),str(max_power_current)))
        ax.grid()
        plt.show()
        #dirname=get_date_dirname()
        #figname=os.path.join(dirname,"%s_IV.png" %ftext)
        #fig.savefig(figname)
        plt.close()
        
def start_zxscan_callback():
        ftext=entry_prefix.get() + '_zx_scan'
        xdistance_mm=float(entry_xdistance.get())
        step_delay=float(entry_delay.get())
        zdistance_steps=5000
        zstep=500
        datadict_zx={}
        datadict_zx['xposition']=[]
        datadict_zx['power']=[]
        datadict_zx['zpositions']=[]
        
        move_wait(ser_pb_STP,0,-1*xdistance_mm*steps_per_mm/2,50)
        datadict_1d = smoothscan1d(axis=0,distance=xdistance_mm,delay=step_delay,ser_pb_STP=ser_pb_STP,ser_fm=ser_fm)
        move_wait(ser_pb_STP,0,-1*xdistance_mm*steps_per_mm/2,50)        
        datadict_zx['zpositions'] += list(np.zeros(len(datadict_1d['position']),dtype=int))
        datadict_zx['xposition'] += datadict_1d['position']
        datadict_zx['power'] +=datadict_1d['power']

        for z in range(0,zdistance_steps,zstep):
                FTA_move_z(ser_pb_FTA,zstep)
                move_wait(ser_pb_STP,0,-1*xdistance_mm*steps_per_mm/2,50)
                datadict_1d = smoothscan1d(axis=0,distance=xdistance_mm,delay=step_delay,ser_pb_STP=ser_pb_STP,ser_fm=ser_fm)
                move_wait(ser_pb_STP,0,-1*xdistance_mm*steps_per_mm/2,50)        
                datadict_zx['zpositions'] += list(int(z+zstep)+np.zeros(len(datadict_1d['position']),dtype=int))
                datadict_zx['xposition'] += datadict_1d['position']
                datadict_zx['power'] += datadict_1d['power']
        FTA_move_z(ser_pb_FTA,-zdistance_steps)
        fname=get_date_filename(ftext)
        print(fname)
        save_dict_to_h5(datadict_zx,fname)
        #now plot also
        fig, ax = plt.subplots()
        zs=np.array(datadict_zx['zpositions'])
        zlist=np.array( np.unique(datadict_zx['zpositions']))
        for z in zlist:
                ax.plot(np.array(datadict_zx['xposition'])[zs==z],1000*np.array(datadict_zx['power'])[zs==z],label='Zposition' + str(z)+' steps')
        ax.set(xlabel='X Distance [mm]', ylabel='Measured power [mw]', title='Scan %s' %ftext)
        ax.legend()
        ax.grid()
        plt.show()
        plt.close()


def start_xyscan_callback():
        ftext=entry_prefix.get() + '_2D_scan'
        xdistance_mm=float(entry_xdistance.get())
        ydistance_mm=float(entry_ydistance.get())
        ystep_mm=float(entry_ystep.get())
        step_delay=float(entry_delay.get())
        #let's assume we're starting at center of the scan we want
        print('move back x by',-1*xdistance_mm*steps_per_mm/2)
        move_wait(ser_pb_STP,0,-1*xdistance_mm*steps_per_mm/2,50)
        print('move back y by',-1*ydistance_mm*steps_per_mm/2)
        move_wait(ser_pb_STP,1,-1*ydistance_mm*steps_per_mm/2,50)
        datadict_2d = smoothscan2d(xdistance=xdistance_mm,ydistance=ydistance_mm,ystepsize=ystep_mm,delay=step_delay,ser_pb_STP=ser_pb_STP,ser_fm=ser_fm)
        signx = (-1)**(int(ydistance_mm/ystep_mm)+1)
        print(signx)
        print('move back x by',signx*xdistance_mm*steps_per_mm/2)
        move_wait(ser_pb_STP,0,signx*xdistance_mm*steps_per_mm/2,50)
        print('move back y by',-1*ydistance_mm*steps_per_mm/2)
        move_wait(ser_pb_STP,1,-1*ydistance_mm*steps_per_mm/2,50)
        fname=get_date_filename(ftext)
        print(fname)
        save_dict_to_h5(datadict_2d,fname)


def leftKey(event):
        print("Left key pressed")
        jog_distance_mm = float(entry_jog_distance.get())
        move_wait(ser_pb_STP,0,-1*jog_distance_mm*steps_per_mm,50)

def rightKey(event):
        print ("Right key pressed")
        jog_distance_mm = float(entry_jog_distance.get())
        move_wait(ser_pb_STP,0,jog_distance_mm*steps_per_mm,50)

def upKey(event):
        print ("Up key pressed")
        jog_distance_mm = float(entry_jog_distance.get())
        move_wait(ser_pb_STP,1,jog_distance_mm*steps_per_mm,50)

def downKey(event):
        print ("Down key pressed")
        jog_distance_mm = float(entry_jog_distance.get())
        move_wait(ser_pb_STP,1,-1*jog_distance_mm*steps_per_mm,50)
        

def do_nothing_callback():
        focus_jog_button.focus_set()
        return
        
def do_nothing_FTA_callback():
        focus_zjog_button.focus_set()
        return
        
#FTA FUNCTIONS

def upKey_FTA_z(event):
        print ("Up key pressed positive FTA z motion")
        zjog_distance = int(entry_zjog_distance.get())
        FTA_move_z(ser_pb_FTA, zjog_distance)

def downKey_FTA_z(event):
        print ("Down key pressed negative FTA z motion")
        zjog_distance = int(entry_zjog_distance.get())
        FTA_move_z(ser_pb_FTA, -1*zjog_distance)

def FTA_move_z(ser,steps):
        delay='1'
        test_str = 'move_z,'  + str(steps) +',' + delay +  '\r\n'
        print(test_str)
        ser.write( test_str.encode() ) 
        ser.flushInput()
        not_done_moving=True
        while(not_done_moving):
                bytes_to_read=ser.in_waiting
                if bytes_to_read>5:
                        not_done_moving=False
                sleep(.1)

ser_pb_STP,ser_fm,ser_pb_FTA = get_pyboard_fm(ports)

beam_mapper_label=Label(window,text='2D Beam Mapper',font = "Helvetica 18 bold")
beam_mapper_label.pack(side=TOP)
pwlbls=[]
pwtxts=[]



ffolder=Frame(window)
labelfolder=Label(ffolder,text='Folder',font = "Helvetica 12 bold")
labelfolder.pack(side="left")
entry_folder=Entry(ffolder)
entry_folder.insert(END,'C:\\Starshot\\beam_measurements')
entry_folder.configure(width=50,font = "Helvetica 12 bold")
entry_folder.pack(side="left")
ffolder.pack(side=TOP)

fprefix=Frame(window)
labelprefix=Label(fprefix,text='File prefix',font = "Helvetica 12 bold")
labelprefix.pack(side="left")
entry_prefix=Entry(fprefix)
entry_prefix.insert(END,'Beam')
entry_prefix.configure(width=10,font = "Helvetica 12 bold")
entry_prefix.pack(side="left")
fprefix.pack(side=TOP)

fxscan=Frame(window)
labelxdistance=Label(fxscan,text='X distance [mm]',font = "Helvetica 12 bold")
labelxdistance.pack(side="left")
entry_xdistance=Entry(fxscan)
entry_xdistance.insert(END,'100')
entry_xdistance.configure(width=10,font = "Helvetica 12 bold")
entry_xdistance.pack(side="left")


labeldelay=Label(fxscan,text='step delay time [uS]',font = "Helvetica 12 bold")
labeldelay.pack(side="left")
entry_delay=Entry(fxscan)
entry_delay.insert(END,'100')
entry_delay.configure(width=10,font = "Helvetica 12 bold")
entry_delay.pack(side="left")
fxscan.pack(side=TOP)

fyscan=Frame(window)
labelydistance=Label(fyscan,text='Y distance [mm]',font = "Helvetica 12 bold")
labelydistance.pack(side="left")
entry_ydistance=Entry(fyscan)
entry_ydistance.insert(END,'100')
entry_ydistance.configure(width=10,font = "Helvetica 12 bold")
entry_ydistance.pack(side="left")

labelystep=Label(fyscan,text='Y stepsize [mm]',font = "Helvetica 12 bold")
labelystep.pack(side="left")
entry_ystep=Entry(fyscan)
entry_ystep.insert(END,'5')
entry_ystep.configure(width=10,font = "Helvetica 12 bold")
entry_ystep.pack(side="left")
fyscan.pack(side=TOP)


start_xscan_button=Button(window,command=start_xscan_callback)
start_xscan_button.configure(text='Start X scan',background='Light Green',padx=50,font = "Helvetica 12 bold")
start_xscan_button.pack(side=TOP) 

start_yscan_button=Button(window,command=start_yscan_callback)
start_yscan_button.configure(text='Start Y scan',background='Light Blue',padx=50,font = "Helvetica 12 bold")
start_yscan_button.pack(side=TOP) 

start_xyscan_button=Button(window,command=start_xyscan_callback)
start_xyscan_button.configure(text='Start 2D XY scan',background='Pink',padx=50,font = "Helvetica 12 bold")
start_xyscan_button.pack(side=TOP) 

start_zyscan_button=Button(window,command=start_zxscan_callback)
start_zyscan_button.configure(text='Start ZX scan (FTA)',background='Red',padx=50,font = "Helvetica 12 bold")
start_zyscan_button.pack(side=TOP) 



fjog_distance=Frame(window)
labeljog_distance=Label(fjog_distance,text='Jog distance [mm]',font = "Helvetica 12 bold")
labeljog_distance.pack(side="left")
entry_jog_distance=Entry(fjog_distance)
entry_jog_distance.insert(END,'20')
entry_jog_distance.configure(width=10,font = "Helvetica 12 bold")
entry_jog_distance.pack(side="left")
fjog_distance.pack(side=TOP)

focus_jog_button=Button(window,command=do_nothing_callback)
focus_jog_button.configure(text='Arrow Key Control',background='Yellow',padx=50,font = "Helvetica 12 bold")
focus_jog_button.pack(side=TOP) 
 
focus_jog_button.bind('<Left>',leftKey)
focus_jog_button.bind('<Right>',rightKey)
focus_jog_button.bind('<Up>',upKey)
focus_jog_button.bind('<Down>',downKey)
focus_jog_button.focus_set()

fzjog_distance=Frame(window)
labelzjog_distance=Label(fzjog_distance,text='zjog distance [steps]',font = "Helvetica 12 bold")
labelzjog_distance.pack(side="left")
entry_zjog_distance=Entry(fzjog_distance)
entry_zjog_distance.insert(END,'100')
entry_zjog_distance.configure(width=10,font = "Helvetica 12 bold")
entry_zjog_distance.pack(side="left")
fzjog_distance.pack(side=TOP)

focus_zjog_button=Button(window,command=do_nothing_FTA_callback)
focus_zjog_button.configure(text='FTA Z axis Arrow Key Control',background='Yellow',padx=50,font = "Helvetica 12 bold")
focus_zjog_button.pack(side=TOP) 
 
focus_zjog_button.bind('<Up>',upKey_FTA_z)
focus_zjog_button.bind('<Down>',downKey_FTA_z)
focus_zjog_button.focus_set()

 
 


 
window.mainloop()

