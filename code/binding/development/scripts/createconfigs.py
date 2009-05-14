
inittext = """

--inputParamFile=subarchitectures/comsys.mk4/learning/params.txt
--CCGGrammarFile=subarchitectures/comsys.mk4/grammars/openccg/%s/grammar.xml
--testFile=tests/data/utterancesToSelect.xml

--NbNBests=%s
--discardItemsWithoutCorrectParse=true

--extractSemanticFeatures=%s
--extractSyntacticFeatures=%s
--extractAcousticFeatures=%s
--extractContextualFeatures=%s
--extractNoParseFeature=%s

--maxUtteranceLength=14

--WERFileName=data/WER_%s.txt
--AccuracyFileName=data/accuracy_%s.txt


--useFakeSaliency=true
--contextdependentwordsFilename=subarchitectures/comsys.mk4/learning/contextdependentwords.txt
--salientwordsFilename=subarchitectures/comsys.mk4/learning/salientwords.txt
"""

grammars = ["robustmoloko", "nonrobustmoloko"]
NbNBests = ["1", "3", "5", "10"]
Se = ["true", "false"]
Sy = ["true", "false"]
A = ["true", "false"]
C = ["true", "false"]

for g in grammars:
	for n in NbNBests:
		for se in Se:
			for sy in Sy:
				for a in A:
					for c in C:
						config = "NBest"+n+"_"
						if se == "true":
							config = config + "Se"
						if sy == "true":
							config = config + "Sy"
						if a == "true":
							config = config + "A"
						if c == "true":
							config = config + "C"
						if g == "robustmoloko":
							config = config + "_R"
						else:
							config = config + "_NR"
						if "__" in config:
							config = config.replace("__", "_None_")
						
						text = inittext%(g,n, se, sy, a, c, se, config, config)						
						f = file("tests/psconfigs/testconfig_%s.txt"%config, 'w')
						f.write(text)
						f.close()
