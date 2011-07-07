
.PHONY: all

all: uimainwindow.py uiconfigdlg.py uiselectcomponentsdlg.py castcontrol_rc.py uitextedit.py

uimainwindow.py: mainwindow.ui
	@pyuic4 -o uimainwindow.py mainwindow.ui
	@sed -i -e '/^# WARNING!/ a from cctextedit import CCastTextEdit' \
		-e 's/self.buildLogfileTxt = QtGui.QTextEdit(/self.buildLogfileTxt = CCastTextEdit(/' \
		-e 's/self.mainLogfileTxt = QtGui.QTextEdit(/self.mainLogfileTxt = CCastTextEdit(/' \
		uimainwindow.py

uiconfigdlg.py: configdlg.ui
	@pyuic4 -o uiconfigdlg.py configdlg.ui

uiselectcomponentsdlg.py: selectcomponentsdlg.ui
	@pyuic4 -o uiselectcomponentsdlg.py selectcomponentsdlg.ui

castcontrol_rc.py: castcontrol.qrc res/*.png
	@pyrcc4 -o castcontrol_rc.py castcontrol.qrc

uitextedit.py: textedit.ui
	@pyuic4 -o uitextedit.py textedit.ui

#icons.py: res/cogx.xpm
#   @python makeres.py
