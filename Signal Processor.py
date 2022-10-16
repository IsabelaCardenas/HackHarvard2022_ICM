# -*- coding: utf-8 -*-
"""
Created on Sat Oct 15 17:59:05 2022

@author: 18324
"""
import requests
import serial
import numpy as np
from scipy import signal

#pre-set captured signals of a "1" and a "0" being drawn
one_ax = [14488, 4072, 4108, 4420, 3584, 3896, 3080, 2892, 2680, 2224, -608, 2724, 1700, 2560, 2348, 4340, 2576, 3468, 1208, 2036, 2452, 2944, 3916, 3984, 3888, 2940, 3844, 3104, 3096, 3012, 3152, 3212, 3724, 3420]
one_ay = [-17808, -15276, -15680, -15972, -15824, -15968, -15672, -16188, -15748, -15740, -22616, -16932, -16372, -17008, -16328, -16696, -16940, -17332, -16304, -17092, -16548, -16344, -16364, -16476, -16364, -16500, -16404, -16628, -16400, -16476, -16520, -16408, -16624, -16036]
one_az = [-5500, -3364, -4040, -4148, -4220, -4348, -3896, -3824, -3216, -2240, -48, -1696, -1208, -1412, -720, -1208, -1728, -292, -900, -832, -1072, -1072, -492, -620, -544, -100, -488, -176, -1452, -1776, -1956, -1696, -2056, -2096]
one_rx = [-108, -2507, -723, -503, 346, 1180, 654, 2702, 3491, 5841, 1198, 973, 1712, 999, 1454, 1685, 1710, 1448, 1285, 877, 346, 88, -329, 248, 1159, 1900, -217, -572, -1532, -1161, -1381, -2240, -2551, -4167]
one_ry = [-410, 849, -1047, -2416, -1466, 1146, -1265, -1449, -2387, -3456, -4707, -4198, -817, -2603, -3069, -1999, -1991, -2723, -2459, 279, -697, -520, -286, -807, -3509, -3633, -806, -1204, -1331, -308, -431, -1204, 626, 981]
one_rz = [-1427, -109, -347, 2552, 3187, -685, 1082, 3685, 3291, 2916, 1018, -754, -1947, -2294, -2360, -2349, -1896, -2255, -1019, -1634, -851, -478, -79, 772, 2573, 4073, 348, 660, 787, 334, -1, -403, -434, -333]

zero_ax = [7660, 1624, 3364, 2588, 2728, 2388, 2508, 2244, 1596, 1604, 1248, 2536, 2496, 2596, 4172, 3136, 3652, 2404, 3596, 4420, 2244, -84, 280, 1236, 3632, 9024, 2260, 4332, 4384, 2280, 1588, 300, 4604, 2204, 1916, 1644, 1676, 2164, 2240, 2484, 1980]
zero_ay = [-16888, -16144, -16136, -16416, -15800, -16144, -16396, -15280, -16888, -15748, -15728, -15816, -16960, -16588, -16860, -16672, -16560, -16940, -16568, -16576, -15536, -16524, -15504, -16376, -16612, -18708, -16004, -17168, -17192, -15764, -16404, -14536, -17260, -16772, -16756, -16336, -16656, -16564, -16572, -16808, -16740]
zero_az = [-4592, -2928, -4432, -3164, -3272, -2924, -2724, -1536, -1684, -3776, -5164, -2232, -1816, -2004, -712, -388, -548, -476, -408, -700, 1116, -2496, -1760, -1756, -880, -1684, -1524, -248, 844, -2904, -3280, -5188, 1412, -2004, 1532, 456, -852, -1244, -1544, -1508, -748]
zero_rx = [-285, 247, 235, 168, 997, 2646, 2297, 1556, 812, -87, 27, -25, 539, 85, 448, 1053, -14, -93, 579, 722, 1649, 2503, 2473, 1480, 1214, 61, -1118, -2337, -962, -781, -1134, -1427, -266, 54, -777, -1003, -1561, -1078, -499, 923, 335]
zero_ry = [1101, -1114, -363, 1402, -305, 400, -1981, 1258, -1817, -2073, -2484, -2691, -2390, -2506, -3012, -3363, -1799, -2693, -1988, -280, -639, -787, 2071, 2413, 2154, 2016, 1246, 1026, 996, 53, -229, -2064, -2343, -1903, -1499, -332, 272, 930, 1327, -4048, -16]
zero_rz = [-2211, 330, 533, -677, -148, 567, 987, 882, 482, 183, 520, 336, -802, -837, -1362, -271, -2171, -1738, -3303, -4029, -2171, -457, -447, -171, 2023, 1399, 1756, 1938, 1913, 2088, 313, 720, 143, 274, -183, 108, -8, -555, -606, 1439, 2838]

#variables for parsing
lastLine = ""
currentSignal = []
CS_ax = []
CS_ay = []
CS_az = []
CS_rx = []
CS_ry = []
CS_rz = []

#setting up connection to Serial Port here instead of in Arduino IDE
ser = serial.Serial("COM9", 115200)

#variables for similarity to 1 or 0!
prob_of_one = 0
prob_of_zero = 0

while (True):
    #printing serial monitor data from Teensy/ Arduino IDE
    ln = ser.readline().decode("utf-8")
    
    #take out special charaters from line
    line =  ln.replace('\r', '').replace('\n', '')
    
    #puts all non-repeating lines into an list to be handles later
    if (line != lastLine):
        currentSignal.append(line)
    lastLine = line
    
    if (line == "End of signal"):
        currentSignal.pop(-1)
        
        
        #turn all entries (string of integers) into list of integers
        for e in currentSignal:
            index = currentSignal.index(e)
            temp = e.split()
            #e is not a list of of strings containing a number and a comma
            
            #remove comma from each entry
            for f in temp:
                ind = temp.index(f)
                if (f[-1] == ","):
                    f = f[:-1]
                temp[ind] = int(f)
            currentSignal[index] = temp 
            
           
            #NOW WE HAVE A SIGNAL TO COMAPRE OMG IM SO EXCITED
            
        #Split this 2d array into 6 arrays to represent accleration in x, y, z, and rotation in x, y, z
        #currentSignal = currentSignal.pop()

        print(currentSignal)
        for e in currentSignal:
            CS_ax.append(currentSignal[currentSignal.index(e)][0])
        for e in currentSignal:
            CS_ay.append(currentSignal[currentSignal.index(e)][1])
        for e in currentSignal:
            CS_az.append(currentSignal[currentSignal.index(e)][2])
        for e in currentSignal:
            CS_rx.append(currentSignal[currentSignal.index(e)][3])
        for e in currentSignal:
            CS_ry.append(currentSignal[currentSignal.index(e)][4])
        for e in currentSignal:
            CS_rz.append(currentSignal[currentSignal.index(e)][5])
        
        #now we have to make these arrays the same size as the pre-set arrays
        
    
    