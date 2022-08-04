#!/usr/local/bin/python3
import matplotlib.pyplot as plt
import numpy as np 


with open('test_data.txt') as f:
	array = [[int(x) for x in line.split()] for line in f]
	print(array)
lastEnd=0;
x=[];
y=[];
base=0;
state=1; # 1:new increase, 
	 # 2:old decrease to new.
for item in array:
	if lastEnd>item[0]:
		base = base + 256;
		state = 2;		
	#print(state, base, lastEnd, item[0])
	if state == 1:
		x_sub=[lastEnd+base, base+item[0], base+item[0], base+item[0]+item[1]];
	elif state == 2:
		x_sub=[lastEnd+base-256, base+item[0], base+item[0], base+item[0]+item[1]];
		state = 1;
	x = np.append(x, x_sub);
	y_sub=[0, 0, 1, 1];
	y = np.append(y, y_sub);
	lastEnd=item[0]+item[1];
	print(x_sub, y_sub)

plt.plot(x, y);
plt.show()

