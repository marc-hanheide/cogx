# vim: set ts=4 sw=4 noet list:vim

.PHONY: all report graphs

all: graphs report

report: xdata/gy2article.pdf

xdata/gy2article.pdf: gy2article.tex graphs
	pdflatex --output-directory xdata gy2article.tex

graphs: xdata/states.pdf xdata/simple_states.pdf

xdata/simple_states.pdf: simple_states.dot
	dot simple_states.dot -Tpdf -oxdata/simple_states.pdf

xdata/states.pdf: states.dot
	dot states.dot -Tpdf -oxdata/states.pdf
