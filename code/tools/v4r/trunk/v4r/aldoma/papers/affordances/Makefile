SRC = affordances

all:
	pdflatex $(SRC).tex
	bibtex $(SRC)
	pdflatex $(SRC).tex
	pdflatex $(SRC).tex
clean:
	rm -f *.aux *.bbl *.blg *.log *~ 
