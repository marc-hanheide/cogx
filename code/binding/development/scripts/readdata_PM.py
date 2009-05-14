import os

grammars = ["robustmoloko", "nonrobustmoloko"]
NbNBests = ["1", "3", "5", "10"]
Se = ["true", "false"]
Sy = ["true", "false"]
A = ["true", "false"]
C = ["true", "false"]

data = {}

lines = []
for g in grammars:
	data[g] =  {}
	for se in Se:
		data[g][se] = {}
		for sy in Sy:
			data[g][se][sy] =  {}
			for a in A:
				data[g][se][sy][a] = {}
				for c in C:
					data[g][se][sy][a][c] = {}
					for n in NbNBests:
						data[g][se][sy][a][c][n] = {}
						line = ""
						config = "NBest"+n+"_"
						if g == "robustmoloko":
							line = line + "x,"
						else:
							line = line +" ,"
						if se == "true":
							config = config + "Se"
							line = line + "x,"
						else:
							line = line +" ,"
						if sy == "true":
							config = config + "Sy"
							line = line + "x,"
						else:
							line = line +" ,"
						if a == "true":
							config = config + "A"
							line = line + "x,"
						else:
							line = line +" ,"
						if c == "true":
							config = config + "C"
							line = line + "x,"
						else:
							line = line +" ,"
							
						if g == "robustmoloko":
							config = config + "_R"
						else:
							config = config + "_NR"
						if "__" in config:
							config = config.replace("__", "_None_")

						f = file("data/accuracy_%s.txt"%config, 'r')
						text = f.readlines()
						print config
						TP_PM = 0
						FP_PM = 0
						TN_PM = 0
						FN_PM = 0
						for t in text:
							pm_text = t.split("partial-match result: ")
							print pm_text
							pm_text_split1 = pm_text[1].split("(")
							if " TP & " in pm_text_split[1]:
								val = pm_text_split[1].split(" TP &")[0]
								TP_PM = TP_PM + float(val)
								val2 = pm_text_split[1].split(" TP &")[1].split(" FP)")[0]
								FP_PM = FP_PM + float(val2)
							elif " TN & " in pm_text_split[1]:
								val = pm_text_split[1].split(" TN &")[0]
								TN_PM = TN_PM + float(val)
								val2 = pm_text_split[1].split(" TN &")[1].split(" FN)")[0]
								FN_PM = FN_PM + float(val2)

						precision_PM = TP_PM / (TP_PM+FP_PM)
						recall_PM = TP_PM / (TP_PM+FN_PM)
						if precision_PM+recall_PM > 0:
							f_1_PM = 2*precision_PM*recall_PM / (precision_PM+recall_PM)
						else:
							f_1_PM = 0.0

						pr_PM_str = str(precision_PM*100)
						if len(pr_PM_str) > 4:
							pr_PM_str = pr_PM_str[:4]
						elif len(pr_PM_str) == 2:
							pr_PM_str = pr_PM_str+".0"
						data[g][se][sy][a][c][n]['pr_PM'] = pr_PM_str
						r_PM_str = str(recall_PM*100)
						if len(r_PM_str) > 4:
							r_PM_str = r_PM_str[:4]
						elif len(r_PM_str) == 2:
							r_PM_str = r_PM_str+".0"
						data[g][se][sy][a][c][n]['r_PM'] = r_PM_str
						f_1_PM_str = str(f_1_PM*100)
						if len(f_1_PM_str) > 4:
							f_1_PM_str = f_1_PM_str[:4]
						elif len(f_1_PM_str) == 2:
							f_1_PM_str = f_1_PM_str+".0"
						data[g][se][sy][a][c][n]['f1_PM'] = f_1_PM_str

					for n in NbNBests:
						line = line + str(data[g][se][sy][a][c][n]['pr_PM']) + " ," + str(data[g][se][sy][a][c][n]['r_PM']) + " , " + str(data[g][se][sy][a][c][n]['f1_PM'])  
						if n != "10":
							line = line + " , "
					line = line + "\n"
					lines.append(line)


#linesstr = ""
#lines.reverse()
#for l in lines:
#	linesstr = linesstr + l

#csvfile = file("csvfile_PM.txt", "w")
#csvfile.write(linesstr)
#csvfile.close()
						




