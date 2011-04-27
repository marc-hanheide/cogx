#! /usr/bin/env python2.5
import subprocess
import sys
import shutil

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

	domain, problem, result_name = sys.argv[1:]

	# run translator
	run("translate/translate.py", domain, problem, output="output.sas")

	# run preprocessing
	run("preprocess/preprocess", input="output.sas", output="output")

	# run search
	run("search/search", "yY", result_name, input="output")

	# epsilonize plan
	# shutil.move("%s.1" % result_name, result_name)
	# run("search/epsilonize_plan.py", input=result_name, output="%s_eps" % result_name)
	# shutil.move("%s_eps" % result_name, result_name)

if __name__ == "__main__":
    main()

