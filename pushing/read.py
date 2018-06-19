from parse import *

title = True
data = []
with open('/u0/home/ngothoskar/Desktop/out.txt') as f:
    angle = 0
    offset = 0
    for line in f:
    	if title:
    		format_string = "{:d} {:f}\n"
    		p = parse(format_string,line)
    		angle, offset = p[0], p[1]
    		print angle, offset
    		title = False
    	else:
    		format_string = "[{:f}, {:f}, {:f}]"
    		p = parse(format_string,line)
    		print p
    		data.append(((angle,offset),(p[0],p[1],p[2])))
    		title = True