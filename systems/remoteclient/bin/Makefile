
all: castcontrol ccagent cmake-freeze cmake-apply


castcontrol: castcontrol.in mkcc.sh
	./mkcc.sh castcontrol.in castcontrol
	chmod u+x castcontrol
	ln -sf bin/castcontrol ../

ccagent: ccagent.in mkcc.sh
	./mkcc.sh ccagent.in ccagent
	chmod u+x ccagent
	ln -sf bin/ccagent ../

cmake-freeze: scripts/cmake-freeze
	ln -sf scripts/cmake-freeze ./

cmake-apply: scripts/cmake-apply
	ln -sf scripts/cmake-apply ./

output: ../output/jar

../output/jar:
	if [ ! -d ../output/jar ]; then mkdir -p ../output/jar ; fi
