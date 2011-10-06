
all: castcontrol ccagent


castcontrol: castcontrol.in mkcc.sh
	./mkcc.sh castcontrol.in castcontrol
	chmod u+x castcontrol
	ln -sf bin/castcontrol ../

ccagent: ccagent.in mkcc.sh
	./mkcc.sh ccagent.in ccagent
	chmod u+x ccagent
	ln -sf bin/ccagent ../
