
.PHONY: all

all: uimainwindow.py uiconfigdlg.py castcontrol_rc.py

uimainwindow.py: mainwindow.ui
	@pyuic4 -o uimainwindow.py mainwindow.ui
	@sed -i -e '/^# WARNING!/ a from cctextedit import CCastTextEdit' \
		-e 's/self.buildLogfileTxt = QtGui.QTextEdit(/self.buildLogfileTxt = CCastTextEdit(/' \
		-e 's/self.mainLogfileTxt = QtGui.QTextEdit(/self.mainLogfileTxt = CCastTextEdit(/' \
		uimainwindow.py

uiconfigdlg.py: configdlg.ui
	@pyuic4 -o uiconfigdlg.py configdlg.ui

castcontrol_rc.py: castcontrol.qrc
	@pyrcc4 -o castcontrol_rc.py castcontrol.qrc

#icons.py: res/cogx.xpm
#   @python makeres.py
