
f = file("logbatch.txt", "r")
lines = f.readlines()

data = {}

for l in lines:	
	if "Transcription" in l:
		curUtt =  []
		transcription = l.replace("Transcription: ", "")
		transcription = transcription.replace("\r\n", "")
		data[transcription] = curUtt
	elif "Result #0:" in l:
		result = {}
		result["transcription"] = transcription
		line = l.replace("Result #", "")
		line = line.replace("\r\n", "")
		firstSplit = line.split(":")
		result["rank"] = firstSplit[0]
		recog = firstSplit[1].replace(" (conf", "").strip()
		result["string"] = recog
		conf = firstSplit[2].replace(", NL conf", "").strip()
		result["conf"] = conf
		NLconf = firstSplit[3].replace(")", "").strip()
		result["NLconf"] = NLconf
		if int(result["rank"]) < 5:
			curUtt.append(result)
	elif "Rec Errors:" in l:
		split = l.split("sub = ")
		split2 = split[1].split("% of ")
		result["WER"] = float(split2[0])
		

f.close()
#print data

x = file("baseline-WER.txt", "w")

keys = data.keys()

tw = ""
for d in keys:
	utterance = data[d]
	for r in utterance:
		tw = tw + "\"" + r["transcription"] + "\" --> \"" + r["string"] + "\"" + "\n"
tw = tw.replace("how many", "howmany")
x.write(tw)
x.close()
