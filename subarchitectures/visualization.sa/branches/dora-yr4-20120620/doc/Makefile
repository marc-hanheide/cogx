
.PHONY: all

all: BUILD/v11n_manual.pdf

SOURCES=v11n.tex config.tex

BUILD/v11n_manual.pdf: v11n_manual.tex $(SOURCES)
	if [ ! -d BUILD ]; then mkdir BUILD ; fi
	pdflatex --output-directory BUILD v11n_manual.tex

