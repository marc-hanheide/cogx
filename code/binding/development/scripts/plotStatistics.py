import matplotlib
from pylab import *


f = file("subarchitectures/comsys.mk4/learning/perceptronstatistics.txt","r")
lines = f.readlines()

yvalues = []
yvalues2 = []

incr = 0.0
nbrOK = 0.0
nbrKO = 0.0

for line in lines:
	if (incr < len(lines)):
		incr = incr + 1.0
		line2 = line.replace("\n","")
		if (line2 == "2"):
			nbrKO = nbrKO +1.0
		elif (line2 == "0"):
			nbrOK = nbrOK +1.0
		percent = (nbrOK/incr)*100
		invpercent = (nbrKO/incr)*100
		yvalues.append(percent)
		yvalues2.append(invpercent)

yvalues3 =[]
for i in range(len(yvalues)):
	yvalues3.append(6.78)

clf()
xvalues = arange(len(yvalues))
p = plot(xvalues, yvalues, 'g--', label="% of training examples matching expected result")
plot(xvalues, yvalues2, 'g:', label="% of training examples requiring a perceptron update")
axhline(6.78, label="Level of intrinsic ambiguity in training data")
xlim(0,20400)
legend(loc=7)
ltext = gca().get_legend().get_texts()
setp(ltext[0], fontsize = 10)
setp(ltext[1], fontsize = 10)
setp(ltext[2], fontsize = 10)
hold(False)
xlabel("Number of training examples processed")
ylabel("Relative percentage (in %)")
#leg = (, )
#l = legend(leg, loc=7)
#leg.set_size(2)
title("Convergence of perceptron learning")
savefig("figure1.png")
