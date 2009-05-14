import os



def distance(str1, str2):
	words1 = str1.split(" ")
	words2 = str2.split(" ")
	d = []

	for i in range(0, len(words1)):
		newline = []
		for j in range(0, len(words2)):
			newline.append(0)
		d.append(newline)
			
	for i in range(0, len(words1)):
		if words1[i] == words2[0]:
			d[i][0] = i
		else:
			d[i][0] = i+1


	for j in range(0, len(words2)):
		if words1[0] == words2[j]:
			d[0][j] = j
		else:
			d[0][j] = j+1
 
	for i in range(1, len(words1)):
		for j in range(1, len(words2)):
           		if words1[i] == words2[j]:
				cost = 0
			else:
				cost = 1
           		d[i][j] = min(d[i-1][j] + 1, d[i][j-1] + 1, d[i-1][j-1] + cost)

	#for i in range(0, len(words1)):
	#	print d[i]
	#print d[len(words1)-1][len(words2)-1]
	return d[len(words1)-1][len(words2)-1]



def transform (lines):
	examples = []
	for l in lines:
		split = l.split(" --> ")
		if len(split) == 2:
			orig = split[0].replace("\"", "")
			transf = ""
			for o in orig.split(" "):
				if o == "yeah":
					transf = transf + "yes"
				elif o == "that's":
					transf = transf + "that is"
				elif o == "No":
					transf = transf + "no"
				elif o == "don't":
					transf = transf + "do not"
				elif o == "mm":
					transf = transf + ""
				elif o == "uh":
					transf = transf + ""
				elif o == "er":
					transf = transf + ""
				elif o == "err":
					transf = transf + ""
				elif o == "it's":
					transf = transf + "it is"
				elif o == "where's":
					transf = transf + "where is"
				elif o == "I":
					transf = transf + "i"
				elif o == "an":
					transf = transf + "a"
				elif o == "isn't":
					transf = transf + "is not"
				elif o == "howmany":
					transf = transf + "how many"
				else:
					transf = transf + o
				transf = transf + " "
			orig = transf.strip()
			recog = split[1].replace("\"", "")

			recog = recog.replace("rowboat can you put the yellow thing on the green box[recogError-rowboat]", "robot can you put the yellow thing on the green box")
			recog = recog.replace("rowboat all move the green box[recogError-rowboat]", "robot all move the green box")
			recog = recog.replace("rowboat[recogError-rowboat]", "robot")
			recog = recog.replace("rowboat this box is green[recogError-rowboat]", "robot this box is green")
			recog = recog.replace("not take the red triangle[recogError-not]", "now take the red triangle")
			recog = recog.replace("rowboat could you do it[recogError-rowboat]", "robot could you do it")
			recog = recog.replace("ok rowboat could you move the orange object to the left of the blue box[recogError-rowboat]", "ok robot could you move the orange object to the left of the blue box")
			recog = recog.replace("rowboat the blue object is below the red floor[recogError-rowboat]", "robot the blue object is below the red floor")
			recog = recog.replace("what corner is this then[recogError-then]", "what corner is this and")
			recog = recog.replace("rowboat take the red box up to the green box[recogError-rowboat]", "robot take the red box up to the green box")
			recog = recog.replace("rowboat could you move a orange object to the car on the cube[recogError-rowboat]", "robot could you move a orange object to the car on the cube")
			recog = recog.replace("rowboat could you move a orange object to the corner on the table[recogError-rowboat]", "robot could you move a orange object to the corner on the table")
			recog = recog.replace("rowboat can you put the yellow thing on a green box[recogError-rowboat]", "robot can you put the yellow thing on a green box")
			recog = recog.replace("No what colour is that[recogError-that]", "No what colour is it")
			recog = recog.replace("not pick up the red triangle[recogError-not]", "now pick up the red triangle")
			recog = recog.replace("rowboat what do you see[recogError-rowboat]", "robot what do you see")
			recog = recog.replace("and what colour is that[recogError-that]", "and what colour is it")
			recog = recog.replace("what colour is this then[recogError-then]", "what colour is this and")
			recog = recog.replace("rowboat this desk is green[recogError-rowboat]", "robot this desk is green")
			recog = recog.replace("rowboat No move a green box[recogError-rowboat]", "robot No move a green box")
			recog = recog.replace("No that is wrong[recogError-that]", "No it is wrong")
			recog = recog.replace("that is true[recogError-that]", "it is true")
			recog = recog.replace("that is correct[recogError-that]", "it is correct")
			recog = recog.replace("then put it aside[recogError-then]", "and put it aside")
			recog = recog.replace("yes that is correct robot[recogError-that]", "yes it is correct robot")
			recog = recog.replace("ok then[recogError-then]", "ok and")
			recog = recog.replace("No that is a circle[recogError-that]", "No it is a circle")
			recog = recog.replace("then put it in front of the circle[recogError-then]", "and put it in front of the circle")
			recog = recog.replace("that is red[recogError-that]", "it is red")
			recog = recog.replace("yes can you not rotate this triangle[recogError-not]", "yes can you now rotate this triangle")
			recog = recog.replace("yes that is two[recogError-that]", "yes it is two")
			recog = recog.replace("then correct[recogError-then]", "and correct")
			recog = recog.replace("that is wrong to this blue triangle[recogError-that]", "it is wrong to this blue triangle")
			recog = recog.replace("then put it close to the two boxes[recogError-then]", "and put it close to the two boxes")
			recog = recog.replace("ok that is sorry[recogError-that]", "ok it is sorry")
			recog = recog.replace("okay that is fine[recogError-that]", "okay it is fine")
			recog = recog.replace("yes that is the orange cylinder[recogError-that]", "yes it is the orange cylinder")
			recog = recog.replace("that is the red car[recogError-that]", "it is the red car")
			recog = recog.replace("that is sure[recogError-that]", "it is sure")
			recog = recog.replace("that is orange[recogError-that]", "it is orange")
			recog = recog.replace("then do that[recogError-then]", "and do that")
			recog = recog.replace("that is wrong that is it blue triangle[recogError-that]", "it is wrong that is it blue triangle")
			recog = recog.replace("that is wrong that is that blue triangle[recogError-that]", "it is wrong it is it blue triangle")
			recog = recog.replace("yes can you not rotate this triangle there[recogError-not]", "yes can you now rotate this triangle there")			
			recog = recog.replace("that is blue[recogError-that]", "it is blue")
			recog = recog.replace("ok that is fine[recogError-that]", "ok it is fine")
			recog = recog.replace("then now put the orange thing closer to the red cube[recogError-then]", "and now put the orange thing closer to the red cube")
			recog = recog.replace("that is two[recogError-that]", "it is two")
			recog = recog.replace("that is right[recogError-that]", "it is right")
			recog = recog.replace("then yes[recogError-then]", "and yes")
			recog = recog.replace("not take a red triangle[recogError-not]", "now take a red triangle")
			recog = recog.replace("not see a red car[recogError-not]", "now see a red car")
			recog = recog.replace("rowboat this box is green now[recogError-rowboat]", "robot this box is green now")
			recog = recog.replace("rowboat No move the green box[recogError-rowboat]", "robot No move the green box")
			recog = recog.replace("rowboat can you put the yellow thing on the green office[recogError-rowboat]", "robot can you put the yellow thing on the green office")
			recog = recog.replace("not put the red cube on the green cube[recogError-not]", "now put the red cube on the green cube")
			recog = recog.replace("not move the red cube on the green cube[recogError-not]", "now move the red cube on the green cube")
			recog = recog.replace("rowboat now move the green box[recogError-rowboat]", "robot now move the green box")
			recog = recog.replace("rowboat put the red box up to that green box[recogError-rowboat]", "robot put the red box up to that green box")
			recog = recog.replace("yes can we not rotate this triangle[recogError-not]", "yes can we now rotate this triangle")
			recog = recog.replace("put the blue box next to the red books[recogError-books]", "put the blue box next to the red box")
			recog = recog.replace("what colour is the small ball[recogError-ball]", "what colour is the small box")
			recog = recog.replace("take the orange object and put it next to the red box then the blue box[recogError-then]", "take the orange object and put it next to the red box and the blue box")
			recog = recog.replace("not put it in front of the circle[recogError-not]", "now put it in front of the circle")
			recog = recog.replace("rowboat could you move the very orange object to the corner on the cube[recogError-rowboat]", "robot could you move the very orange object to the corner on the cube")
			recog = recog.replace("where is the blue ball[recogError-ball]", "where is the blue box")
			recog = recog.replace("ok then now put the orange cylinder next to that[recogError-then]", "ok and now put the orange cylinder next to that")
			recog = recog.replace("then yellow cone[recogError-then]", "and yellow cone")
			recog = recog.replace("what colour is that small ball[recogError-ball]", "what colour is that small box")
			recog = recog.replace("rowboat can you put the yellow thing on it green box[recogError-rowboat]", "robot can you put the yellow thing on it green box")
			recog = recog.replace("not take that orange object[recogError-not]", "now take that orange object")
			recog = recog.replace("then put it to the right on the red cube[recogError-then]", "and put it to the right on the red cube")
			recog = recog.replace("put that blue box next to the red books[recogError-books]", "put that blue box next to the red box")
			recog = recog.replace("rowboat could you move a orange object to the corner on the cube[recogError-rowboat]", "robot could you move a orange object to the corner on the cube")
			recog = recog.replace("the blocks[recogError-blocks]", "the box")
			recog = recog.replace("rowboat move the red box up to the green box[recogError-rowboat]", "robot move the red box up to the green box")
			recog = recog.replace("where is that red box[recogError-that]", "where is the red box")
			recog = recog.replace("then put this in front of the circle[recogError-then]", "and put this in front of the circle")
			recog = recog.replace("and rotate a ball to the circle[recogError-ball]", "and rotate a box to the circle")
			recog = recog.replace("could you take that and the orange cylinder[recogError-that]", "could you take it and the orange cylinder")
			transf2 = ""
			for o in recog.split(" "):
				if o == "yeah":
					transf2 = transf2 + "yes"
				elif o == "that's":
					transf2 = transf2 + "that is"
				elif o == "No":
					transf2 = transf2 + "no"
				elif o == "don't":
					transf2 = transf2 + "do not"
				elif o == "mm":
					transf2 = transf2 + ""
				elif o == "uh":
					transf2 = transf2 + ""
				elif o == "er":
					transf2 = transf2 + ""
				elif o == "err":
					transf2 = transf2 + ""
				elif o == "it's":
					transf2 = transf2 + "it is"
				elif o == "where's":
					transf2 = transf2 + "where is"
				elif o == "I":
					transf2 = transf2 + "i"
				elif o == "an":
					transf2 = transf2 + "a"
				elif o == "isn't":
					transf2 = transf2 + "is not"
				elif o == "howmany":
					transf2 = transf2 + "how many"
				else:
					transf2 = transf2 + o
				transf2 = transf2 + " "
			recog = transf2.strip()
			if "recogError" in recog:
				print recog
			examples.append([orig,recog])
	return examples
	



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

						f = file("data/WER_%s.txt"%config, 'r')
						text = f.readlines()
						print config
						

						totaldistance = 0
						totalnbofwords = 0
						examples = transform(text)
						for e in examples:
							if e[1] != "":
								dist = distance(e[0],e[1])
							else:
								dist = 	len(e[0].split(" "))
							totaldistance = totaldistance + dist
							totalnbofwords = totalnbofwords + len(e[0].split(" "))
							percent = (float(dist) / len(e[0].split(" "))) * 100

						WER = (float(totaldistance) / totalnbofwords) 

						WER_str = str(WER*100)
						if len(WER_str) > 4:
							WER_str = WER_str[:4]
						elif len(WER_str) == 2:
							WER_str = WER_str+".0"
						data[g][se][sy][a][c][n]['WER'] = WER_str
						print WER_str

					for n in NbNBests:
						line = line + str(data[g][se][sy][a][c][n]['WER']) 
						if n != "10":
							line = line + " , "
					line = line + "\n"
					lines.append(line)


linesstr = ""
lines.reverse()
for l in lines:
	linesstr = linesstr + l

csvfile = file("csvfile_WER.txt", "w")
csvfile.write(linesstr)
csvfile.close()
						




