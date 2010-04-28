
.PHONY: all

all: uiModelBuilder.py uiCaptureSetup.py

uiModelBuilder.py: modelBuilder.ui
	pyuic4 -x -o uiModelBuilder.py modelBuilder.ui
	# Replace a widget with a custom component.
	# (This could be fixed by defining a custom component in QT Designer)
	# (Tested with pyuic 4.4.4 / Qt 4.5.0)
	patch < uiModelBuilderPatch.txt

uiCaptureSetup.py: captureSetup.ui
	pyuic4 -x -o uiCaptureSetup.py captureSetup.ui
