
.PHONY: all

all: uimainwindow.py uiconfigdlg.py

uimainwindow.py: mainwindow.ui
	@pyuic4 -o uimainwindow.py mainwindow.ui
	@sed -i -e '/^# WARNING!/ a from cctextedit import CCastTextEdit' \
		-e 's/self.buildLogfileTxt = QtGui.QTextEdit(/self.buildLogfileTxt = CCastTextEdit(/' \
		-e 's/self.mainLogfileTxt = QtGui.QTextEdit(/self.mainLogfileTxt = CCastTextEdit(/' \
		uimainwindow.py


uiconfigdlg.py: configdlg.ui
	@pyuic4 -o uiconfigdlg.py configdlg.ui
