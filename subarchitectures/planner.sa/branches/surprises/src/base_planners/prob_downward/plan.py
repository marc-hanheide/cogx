#! /usr/bin/env python2.5
import os
import subprocess
import sys
import shutil

path = os.path.abspath(os.path.dirname(__file__))  # where this file resides

def main():
	def run(*args, **kwargs):
		input = kwargs.pop("input", None)
		output = kwargs.pop("output", None)
		assert not kwargs
		redirections = {}
		if input:
			redirections["stdin"] = open(input)
		if output:
			redirections["stdout"] = open(output, "w")
		# print args, redirections
		subprocess.check_call(args, **redirections)

	if len(sys.argv) == 3:
		domain, problem = sys.argv[1:]

		# run translator
		run(os.path.join(path, "translate/translate.py"), domain, problem)
	else:
		domain, problem, mutex = sys.argv[1:]
		# run translator
		run(os.path.join(path, "translate/translate.py"), domain, problem, "-m", mutex)

	# run preprocessing
	run(os.path.join(path, "preprocess/preprocess"), input="output.sas")

	# run search
	run(os.path.join(path, "search/search"), "yY", input="output")

	# epsilonize plan
	# shutil.move("%s.1" % result_name, result_name)
	# run("search/epsilonize_plan.py", input=result_name, output="%s_eps" % result_name)
	# shutil.move("%s_eps" % result_name, result_name)

if __name__ == "__main__":
    main()

