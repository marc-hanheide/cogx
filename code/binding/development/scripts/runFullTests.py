import os

count = 1
for f in os.listdir("tests/psconfigs/"):
	print "Test number: %i / 128"%count
	print "Configuration currently processed: %s..."%f
	command = """java -Xmx600m -ea -classpath ./output/classes/ org.cognitivesystems.comsys.general.testers.ParseSelectionTester_standalone --configFile tests/psconfigs/%s > log.txt 2> errors.txt"""%f
	os.system(command)
	print "Testing data successfully collected."
	print "----"
	count = count + 1
