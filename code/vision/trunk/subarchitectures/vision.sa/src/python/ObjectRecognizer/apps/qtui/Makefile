
.PHONY: all

all: uiModelBuilder.py uiCaptureSetup.py

uiModelBuilder.py: modelBuilder.ui
	pyuic4 -x -o uiModelBuilder.py modelBuilder.ui

uiCaptureSetup.py: captureSetup.ui
	pyuic4 -x -o uiCaptureSetup.py captureSetup.ui
