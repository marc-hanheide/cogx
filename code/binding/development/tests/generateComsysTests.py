
from xml.dom import minidom

import os

def green(text):
	blackc = "\033[0;30m"
	greenc = "\033[0;32m"
	return greenc+text+blackc

metaConfigFilename = "tests/ComSysMetaConfig.xml"
genConfigFilename = "tests/ComSysTests.xml"
defaultoutput = "false"
defaulttimeout = "25000"

print
print green("Start parsing the meta configuration file %s ..."%metaConfigFilename)
print
print "STEP 1: Generating CAST configuration files"
print "-------------------------------------------"
print

# We start by parsing the meta configuration file
dom = minidom.parse(metaConfigFilename)
configs = dom.getElementsByTagName("config")

tests = []

configset = []

# looping on the individual config components
for c in configs:
	
	strings = []
	parsenumbers =  []
	movetypes = []
	references = []
	visualscenes = []
	groundings = []
	realizations = []
	
	newconfig = {}
	newconfig["strings"] = strings
	newconfig["parsenumbers"] = parsenumbers
	newconfig["movetypes"] = movetypes
	newconfig["references"] = references
	newconfig["visualscenes"] = visualscenes
	newconfig["groundings"] = groundings
	newconfig["realizations"] = realizations
	configset.append(newconfig)
		
	# The skeleton CAST file
	castfilename = c.getElementsByTagName("castfile")[0].childNodes[0].nodeValue
	castfile = file(castfilename, "r")
	newconfig["castfilename"] = castfilename
	casttext = castfile.read()
	
	# the test name 
	testname = c.getElementsByTagName("name")[0].childNodes[0].nodeValue
	newconfig["testname"] = testname
	
	# the test description
	description = c.getElementsByTagName("description")[0].childNodes[0].nodeValue
	newconfig["description"] = description
	
	# the XML data file
	datafilename = c.getElementsByTagName("data")[0].childNodes[0].nodeValue
	newconfig["datafilename"] = datafilename
	datafile = file(datafilename, "r")
	
	output = c.getElementsByTagName("output")[0].childNodes[0].nodeValue
	newconfig["output"] = output
	
	timeout = c.getElementsByTagName("timeout")[0].childNodes[0].nodeValue
	newconfig["timeout"] = timeout
	
	fvfilename = ""
	if len(c.getElementsByTagName("fvfile")) > 0:
		fvfilename = c.getElementsByTagName("fvfile")[0].childNodes[0].nodeValue
	
	
	# Parse the XML data file
	dataparse = minidom.parse(datafile)
		
	# Loop on the components of the XML data file
	number = 0
	for ch in dataparse.childNodes[0].childNodes:
		if (ch.nodeName != "#text" and ch.nodeName != "discourse"):
			number = number + 1
			for chi in ch.childNodes:
				if (chi.nodeName == "string"):
					strings.append(chi.childNodes[0].nodeValue)

				if (chi.nodeName == "transcription"):
					strings.append(chi.childNodes[0].nodeValue)

				if (chi.nodeName == "input"):
					strings.append(chi.childNodes[0].nodeValue)
					
				if (chi.nodeName == "discourse"):
					disc = ""
					incr = 1
					for chil in chi.childNodes:
						if (chil.nodeName == "string"):
							disc = disc + "\n#" + str(incr) + " " + \
							chil.childNodes[0].nodeValue
							incr = incr + 1
					strings.append(disc)
				
				if (chi.nodeName == "parses"):
					nbparses = 0
					for chil in chi.childNodes:
						if (chil.nodeName == "parse"):
							nbparses = nbparses + 1
					parsenumbers.append(nbparses)
					
				if (chi.nodeName == "movetype"):
					movetypes.append(chi.childNodes[0].nodeValue)
					
				if (chi.nodeName == "reference"):
					reference = {}
					for chil in chi.childNodes:
						if (chil.nodeName == "refexpr"):
							reference["refexpr"] = chil.childNodes[0].nodeValue
						if (chil.nodeName == "refpos"):
							reference["refpos"] = chil.childNodes[0].nodeValue
						if (chil.nodeName == "referent"):
							reference["referent"] = chil.childNodes[0].nodeValue
					references.append(reference)
							
				if (chi.nodeName == "visualscene"):
					scene = []
					for chil in chi.childNodes:
						if (chil.nodeName == "object"):
							object = {}
							for child in chil.childNodes:
								if (child.nodeName == "label"):
									object["label"] = child.childNodes[0].nodeValue
								if (child.nodeName == "color"):
									object["color"] = child.childNodes[0].nodeValue
								if (child.nodeName == "size"):
									object["size"] = child.childNodes[0].nodeValue
							scene.append(object)
					visualscenes.append(scene)
				
				if (chi.nodeName == "grounding"):
					grounding = {}
					for chil in chi.childNodes:
						if (chil.nodeName == "shouldbegrounded"):
							grounding["shouldbegrounded"] = chil.childNodes[0].nodeValue
						if (chil.nodeName == "referentobject"):
							grounding["referentobject"] = chil.childNodes[0].nodeValue
					groundings.append(grounding)

		elif (ch.nodeName == "discourse"):
			disc = ""
			incr = 1
			for chi in ch.childNodes:
				if (chi.nodeName == "string"):
					disc = disc + "\n#" + str(incr) + " " + \
					chi.childNodes[0].nodeValue
					incr = incr + 1
			strings.append(disc)
			number = number + 1												
	
	print "%i tests found for skeleton %s"%(number, castfilename)

	for i in range(1,number+1):
		gencastfilename = castfilename.replace(".cast", str(i)+".cast").replace("skeletons", "genconfig")
		gencastfile = file(gencastfilename, "w")
		
		if ("visualgrounding" not in castfilename):			
			text = casttext%(datafilename, i)
						
		else:
			text = casttext%(datafilename, i, "tests/genconfig/visualgrounding"+str(i)+".fv")
			
			skeletonfvfile = file(fvfilename, "r")
			skeletonfv =  skeletonfvfile.read()
			skeletonfvfile.close()
			
			fvfile = file(gencastfilename.replace("cast", "fv"), "w")

			fvfile.write(skeletonfv)
			
			incr = 0
			for o in visualscenes[i-1]:
				fvfile.write("%i.10 %i.25 %i, "%(incr,incr,incr))
				fvfile.write("label " + o["label"] + " 1.0")
				if o.has_key("color"):
					fvfile.write(", color " + o["color"] + " 1.0")
				if o.has_key("size"):
					fvfile.write(", size " + o["size"] + " 1.0")	
				incr = incr +1
				if incr < len(visualscenes[i-1]):		
					fvfile.write("\n")
			fvfile.close()	
			print "FV configuration file %s successfully created"%gencastfilename.replace("cast", "fv")

		gencastfile.write(text)
		gencastfile.close()
		print "CAST configuration file %s successfully created"%gencastfilename
	newconfig["number"] = number
	print

	
print green("--> CAST and FV files successfully generated")
print 
print

print "STEP 2: Generating the %s file and movifying the script file"%genConfigFilename
print "-------------------------------------------------"
print

for c in configset:
	strings = c["strings"]
	parsenumbers = c["parsenumbers"] 
	movetypes = c["movetypes"] 
	references = c["references"]
	visualscenes = c["visualscenes"] 
	groundings = c["groundings"]
	castfilename = c["castfilename"] 
	description = c["description"] 
	testname = c["testname"] 
	number = c["number"]
	
	for i in range(1,number+1):	
		newtest = {}
		newtest["castfilename"] = castfilename.replace(".cast", str(i)+".cast").replace("skeletons", "genconfig")
		newtest["name"] = testname%strings[i-1]
		newtest["description"] = description
		
		if ("parsing" in castfilename):
			newtest["description"] = description%(strings[i-1],parsenumbers[i-1])
			
		if ("packing" in castfilename):
			newtest["description"] = description%(strings[i-1])

		if ("tts" in castfilename):
			newtest["description"] = description%(strings[i-1])

		if ("sdrs" in castfilename):
			newtest["description"] = description%(strings[i-1])

		if ("proxyfactories" in castfilename):
			newtest["description"] = description%(strings[i-1])

		if ("eventstruct" in castfilename):
			newtest["description"] = description%(strings[i-1])
			
		if ("dialoguemoves" in castfilename):
			newtest["description"] = description%(strings[i-1], movetypes[i-1])
			
		if ("discrefbinding" in castfilename):
			newtest["description"] = description % (strings[i-1], references[i-1]["refexpr"], references[i-1]["refpos"], references[i-1]["referent"])

		if ("visualgrounding" in castfilename):
			
			vsdisc = ""
			for object in visualscenes[i-1]:
				vsdisc = vsdisc + "[" 
				if (object.has_key("label")):
					vsdisc = vsdisc + "label:" + object["label"]
				if (object.has_key("color")):
					vsdisc = vsdisc + ", color:" + object["color"]
				if (object.has_key("size")):
					vsdisc = vsdisc + ", size:" + object["size"]
				vsdisc = vsdisc + "]"
				vsdisc = vsdisc + ", "
			vsdisc = vsdisc[0:len(vsdisc)-2] 
			
			grounding = ""
			if (groundings[i-1]["shouldbegrounded"] == "yes"):
				grounding = grounding + "be grounded to \"" + groundings[i-1]["referentobject"]+"\""
			else:
				grounding = grounding +  "NOT be grounded"
			
			newtest["description"] = description % (strings[i-1], vsdisc, grounding)
		
		if (c.has_key("output")):
			newtest["output"] = c["output"]
		else:
			newtest["output"] = defaultoutput
			
		if (c.has_key("timeout")):
			newtest["timeout"] = c["timeout"]
		else:
			newtest["timeout"] = defaultoutput	
				
		tests.append(newtest)
		
	
genConfigFile = file(genConfigFilename, "w")
genConfigFile.write("<?xml version=\"1.0\"?>\n<tests>\n\n")
for t in tests:
	genConfigFile.write("\t<test>\n")
	
	genConfigFile.write("\t\t<name>")
	genConfigFile.write(t["name"])
	genConfigFile.write("</name>\n")
	
	genConfigFile.write("\t\t<file>")
	genConfigFile.write(t["castfilename"])
	genConfigFile.write("</file>\n")
	
	genConfigFile.write("\t\t<description>")
	genConfigFile.write(t["description"])
	genConfigFile.write("</description>\n")
	
	genConfigFile.write("\t\t<output>%s</output>\n\t\t<timeout>%s</timeout>\n"%(t["output"], t["timeout"]))
	
	genConfigFile.write("\t</test>\n\n")
	
genConfigFile.write("\n</tests>")
genConfigFile.close()

print green("--> %s successfully generated"%genConfigFilename)
print
print

shellFile = file("tests/skeletons/runComSysTests.sh", "r")
shellText = shellFile.read()
shellFile.close()


if (shellText.count("%") == 11 and len(configset) == 10):

 	shellFile = file("runComSysTests.sh", "w")
	total = 30
	for i in range(len(configset)):
		total = total + configset[i]["number"]
	shellFile.write(shellText%(configset[0]["number"],configset[1]["number"], 
				   configset[2]["number"], configset[3]["number"], 
				   configset[4]["number"], configset[5]["number"], 
				   configset[6]["number"], configset[7]["number"], 
				   configset[8]["number"], configset[9]["number"],  total))

 	print green("--> runComSysTests.sh successfully modified")
 	print
 	print
 	shellFile.close()
else:
	print "Warning: problem modifying the runComsysTests.sh file!!)"
	print
