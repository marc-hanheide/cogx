
f = file("asrerrors.txt", "r")
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
		if "rejected" not in recog and int(result["rank"]) < 5:
			curUtt.append(result)
	elif "Rec Errors:" in l:
		split = l.split("sub = ")
		split2 = split[1].split("% of ")
		result["WER"] = float(split2[0])
		

f.close()
#print data

x = file("subarchitectures/comsys.mk4/learning/gencorpus.txt", "w")

tw = ""
for d in data.keys():
	utterance = data[d]
	for r in utterance:
		if r.has_key("WER") and r["WER"] < 60.00:
			tw = tw + r["transcription"] + ": " + r["string"]+ "\n"
x.write(tw)
x.close()
