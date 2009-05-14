import matplotlib
from pylab import *


f = file("tempresults.txt","r")
lines = f.readlines()

yvalues = []
for l in lines:
	yvalues.append(l)

clf()
xvalues = arange(len(yvalues))
plot(xvalues, yvalues)
hold(False)
title("F_1")
savefig("figure2.png")
