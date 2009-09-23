
.PHONY: all

all: uimainwindow.py uiconfigdlg.py

uimainwindow.py: mainwindow.ui
	pyuic4 -o uimainwindow.py mainwindow.ui

uiconfigdlg.py: configdlg.ui
	pyuic4 -o uiconfigdlg.py configdlg.ui
