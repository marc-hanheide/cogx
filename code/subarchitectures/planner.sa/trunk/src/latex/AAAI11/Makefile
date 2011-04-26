## NOTE: Don't use this Makefile as a basis for future Makefiles.
##       Use the fancier ICAPS08_CausalGraph Makefile instead.

PAPER = aaai11
TIKZFIGURES = 
TRACKING = 393
PACKAGE = AAAI2011GoebelbeckerM1
FAMILY_NAME = Goebelbecker
SUBMISSION = aaai2011-$(TRACKING)

#export BIBINPUTS=../bib

paper: pdf

$(PAPER).pdf: pdf

$(PAPER).ps: ps

pdf: FORCE pdf-figures
	pdflatex $(PAPER).tex	
	bibtex --min-crossrefs=99 $(PAPER)
	pdflatex $(PAPER).tex	
	pdflatex $(PAPER).tex

ps: FORCE eps-figures
	latex $(PAPER).tex	
	bibtex --min-crossrefs=99 $(PAPER)
	latex $(PAPER).tex	
	latex $(PAPER).tex
	dvips -j0 -Ppdf -G0 -tletter $(PAPER)

draft: FORCE pdf-figures
	pdflatex $(PAPER).tex

bib:
	bibtex --min-crossrefs=99 $(PAPER)

eps-figures: $(TIKZFIGURES:%=figures/%.eps)

pdf-figures: $(TIKZFIGURES:%=figures/%.pdf)

$(TIKZFIGURES:%=figures/%.eps): %.eps: %.tex
	latex --output-directory=figures $*
	dvips -o $*.ps $*
	ps2epsi $*.ps $*.eps
	rm $*.aux $*.log $*.dvi $*.ps

$(TIKZFIGURES:%=figures/%.pdf): %.pdf: %.eps
	epstopdf --outfile=$*.pdf $*.eps

submission: $(SUBMISSION).pdf

$(SUBMISSION).pdf: FORCE eps-figures
	cp $(PAPER).tex $(SUBMISSION).tex
	latex $(SUBMISSION).tex
	bibtex --min-crossrefs=99 $(SUBMISSION)
	latex $(SUBMISSION).tex
	latex $(SUBMISSION).tex
	dvips -Ppdf -G0 -tletter $(SUBMISSION).dvi
	ps2pdf -dPDFSETTINGS=/printer -dCompatibilityLevel=1.4 \
	  -dMaxSubsetPct=0 -dSubsetFonts=false -dEmbedAllFonts=true \
	  -sPAPERSIZE=letter $(SUBMISSION).ps

supplemental: supplemental.pdf

supplemental.pdf: FORCE
	pdflatex supplemental.tex

package: FORCE
	scripts/aaai/make-aaai-package $(PAPER) $(PACKAGE) $(FAMILY_NAME)

clean:
	rm -f *~
	rm -f $(PAPER).aux $(PAPER).bbl $(PAPER).blg
	rm -f $(PAPER).dvi $(PAPER).log $(PAPER).ps
	rm -f $(TIKZFIGURES:%=figures/%.eps) $(TIKZFIGURES:%=figures/%.pdf)
	rm -f $(SUBMISSION).aux $(SUBMISSION).bbl $(SUBMISSION).blg
	rm -f $(SUBMISSION).dvi $(SUBMISSION).log $(SUBMISSION).ps
	rm -f $(SUBMISSION).tex $(SUBMISSION).pdf
	rm -f supplemental.aux supplemental.log
	rm -f $(FAMILY_NAME).log
	rm -f $(PACKAGE).tgz 
	rm -rf $(PACKAGE)

distclean: clean
	rm -f $(PAPER).pdf supplemental.pdf

FORCE:

.PHONY: paper pdf ps draft bib clean distclean
.PHONY: submission supplemental package
.PHONY: eps-figures pdf-figures FORCE
