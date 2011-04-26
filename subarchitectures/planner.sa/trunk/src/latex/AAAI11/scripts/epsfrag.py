#! /usr/bin/env python

## Do "psfrag" replacements in an eps file, then generate a new eps
## file from the result. Modelled after the "fragmaster" perl script.


import os.path
import re
import subprocess


BOUNDING_BOX_RE = re.compile(
    r"(\s*%%BoundingBox:\s+)(-?\d+)(\s+)(-?\d+)(\s+)(-?\d+)(\s+)(-?\d+)(\s*)$")

LATEX_CODE_PATTERN = r"""\documentclass{article}
\usepackage{times,helvet,courier}
\usepackage{graphicx}
\usepackage{color}
\usepackage{psfrag}
%(frag_text)s
\setlength{\topmargin}{-1in}
\setlength{\headheight}{0pt}
\setlength{\headsep}{0pt}
\setlength{\topskip}{0pt}
\setlength{\textheight}{\paperheight}
\setlength{\oddsidemargin}{-1in}
\setlength{\evensidemargin}{-1in}
\setlength{\textwidth}{\paperwidth}
\setlength{\parindent}{0pt}
\special{! TeXDict begin /landplus90{true}store end }
\pagestyle{empty}
\newsavebox{\pict}
\begin{document}
\begin{lrbox}{\pict}\includegraphics{%(eps_input)s}\end{lrbox}
\special{papersize=\the\wd\pict,\the\ht\pict}
\usebox{\pict}
\end{document}
"""


def call(*args):
    try:
        subprocess.check_call(args)
    except OSError, e:
        raise SystemExist("error calling %r: %s" % (args, e))


def main(eps_input, frag_input, eps_output):
    frag_text = open(frag_input).read()
    latex_code = LATEX_CODE_PATTERN % dict(
        eps_input=eps_input, frag_text=frag_text)
    stem = os.path.splitext(eps_output)[0]
    tex_output = stem + ".tex"
    open(tex_output, "w").write(latex_code)
    call("latex", tex_output)
    call("dvips", "-E", "-P", "pdf", stem + ".dvi")
    lines_iter = iter(list(open(stem + ".ps")))
    eps_output_file = open(eps_output, "w")
    for line in lines_iter:
        match = BOUNDING_BOX_RE.match(line)
        if match:
            line = match.expand(r"\g<1>0\g<3>\g<4>\g<5>\g<6>\g<7>792\g<9>")
        eps_output_file.write(line)
    eps_output_file.writelines(lines_iter)


if __name__ == "__main__":
    import sys
    try:
        eps_input, frag_input, eps_output = sys.argv[1:]
    except ValueError:
        raise SystemExit("usage: %s EPS_INPUT FRAG_INPUT EPS_OUTPUT"
                         % os.path.basename(sys.argv[0]))
    main(eps_input, frag_input, eps_output)
