
f = file("logbatch.txt", "r")
lines = f.readlines()

data = {}

for l in lines:	
	if "Transcription" in l:
		curUtt =  []
		transcription = l.replace("Transcription: ", "")
		transcription = transcription.replace("\r\n", "")
		data[transcription] = curUtt
	elif "Result #" in l:
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
		if "rejected" not in recog and int(result["rank"]) < 10:
			curUtt.append(result)

f.close()

x = file("tests/data/utterancesToSelect.xml", "w")
tw = "";
tw  = tw + "<utterancesToSelect>\n\n"
keys = data.keys()

for d in keys:
	utterance = data[d]
	tw = tw + "\t<utterance>\n"
	tw = tw + "\t\t<transcription>"
	tw = tw + d
	tw = tw + "</transcription>\n"
	tw = tw + "\t\t<ASRInputs>\n"
	for r in utterance:
		tw = tw + "\t\t\t<input>\n"
		tw = tw + "\t\t\t\t<string>"
		tw = tw + r["string"]
		tw = tw + "</string>\n"
		tw = tw + "\t\t\t\t<conf>"
		tw = tw + r["conf"]
		tw = tw + "</conf>\n"
		tw = tw + "\t\t\t\t<NLconf>"
		tw = tw + r["NLconf"]
		tw = tw + "</NLconf>\n"
		tw = tw + "\t\t\t\t<rank>"
		tw = tw + r["rank"]
		tw = tw + "</rank>\n"
		tw = tw + "\t\t\t</input>\n"
	tw = tw + "\t\t</ASRInputs>\n"		
	tw = tw + "\t</utterance>\n"

tw = tw + "\n\n</utterancesToSelect>\n"
tw = tw.replace("how many", "howmany")
x.write(tw)
x.close()
