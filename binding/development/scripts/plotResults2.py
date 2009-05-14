import matplotlib, os
from pylab import *
import matplotlib.pyplot as plt

f = file("csvfile_PM_sorted.csv", 'r')
text = f.readlines()

#f2 = file("csvfile_PM_sorted.csv", 'r')
#text2 = f2.readlines()


#NBest1 = []
#NBest3 = []
#NBest5 = []
#NBest10 = []
NB_Pr = []
NB_R = []
NB_F1 = []
configs = []

for line in text:
	data = line.split(",")
	NB_Pr.append(float(data[5]))
	NB_R.append(float(data[6]))
	NB_F1.append(float(data[7]))	
	#NBest1.append(float(data[5]))
	#NBest3.append(float(data[6]))
	#NBest5.append(float(data[7]))
	#NBest10.append(float(data[8]))
	config = {}
	if 'x' in data[0]:
		config['GR'] = 'x'
	else:
		config['GR'] = ''
	if 'x' in data[1]:
		config['Se'] = 'x'
	else:
		config['Se'] = ''
	if 'x' in data[2]:
		config['Sy'] = 'x'
	else:
		config['Sy'] = ''
	if 'x' in data[3]:
		config['A'] = 'x'
	else:
		config['A'] = ''
	if 'x' in data[4]:
		config['C'] = 'x'
	else:
		config['C'] = ''
	configs.append(config)

line = text[len(text)-1]
#linebis = text2[len(text2)-1]
data = line.split(",")
#databis = linebis.split(",")
xvalues = range(4)
#line1 = [data[5],data[8],data[11],data[14]]
#line2 = [data[6],data[9],data[12],data[15]]
#line3 = [data[7],data[10],data[13],data[16]]
#line4 = [databis[5],databis[8],databis[11],databis[14]]
#line5 = [databis[6],databis[9],databis[12],databis[15]]
#line6 = [databis[7],databis[10],databis[13],databis[16]]

cellText =  []
cellText.append([])
cellText.append([])
cellText.append([])
cellText.append([])
cellText.append([])

xvalues = range(len(configs))

fig = plt.figure(figsize=(10.5,8.5))
ax = fig.add_subplot(111)

legend(loc=7)


for v in xvalues:
	cellText[0].append(configs[v]['GR'])
	cellText[1].append(configs[v]['Se'])
	cellText[2].append(configs[v]['Sy'])
	cellText[3].append(configs[v]['A'])
	cellText[4].append(configs[v]['C'])

pa = ax.plot(xvalues,NB_Pr, 'o--', antialiased=True)
pb = ax.plot(xvalues,NB_R, 'o--', antialiased=True)
pc = ax.plot(xvalues, NB_F1, 'o:', antialiased=True)

#p1 = ax.plot(xvalues,NBest1, 'o--', antialiased=True)
#p2 = ax.plot(xvalues,NBest3, 'o--', antialiased=True)
#p3 = ax.plot(xvalues,NBest5, 'o--', antialiased=True)
#p4 = ax.plot(xvalues,NBest10, 'o--', antialiased=True)
ax.axis([-0.5,31.5,5,100])
yticks(range(10,100,5))

#legend( (p1, p2, p3, p4), ('NBest 1', 'NBest 3', "NBest 5", "NBest 10"), 'upper right', shadow=True)

legend((pa,pb,pc), ('Precision', 'Recall', '$F_1$'), 'lower right', shadow=True)

ltext = gca().get_legend().get_texts()
setp(ltext[0], fontsize = 16)
setp(ltext[1], fontsize = 16)
setp(ltext[2], fontsize = 16)
#setp(ltext[3], fontsize = 16)

#ax.set_yticklabels([])
ax.set_xticklabels([])

the_table = table(cellText=cellText,
                  rowLabels=[' GR ', ' Se ', ' Sy ', ' A ', ' C '], 
                  loc='bottom')


#ax.grid(True)
#ylabel("Precision, Recall and $F_1$ for partial-match (in %)", fontsize = 16)
#title("Precision, Recall and $F_1$ for partial-match on NBest 5 (in %)", fontsize = 18)

#plt.xlabel("Activated features", fontsize=16)
#plt.text(5.5,-10.0, 'ahaha',fontdict=None)
plt.ylabel("Precision, recall, and $F_1$, in %", fontsize=16)
plt.title("Precision, recall and $F_1$ for partial-match on NBest 1,\nbroken down by activated features", fontsize=18)
#plt.show()
savefig("PM_NBest1.png")



#savefig("WER.png")

