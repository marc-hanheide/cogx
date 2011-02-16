%% ==========================================================================
%%
%%  Class:  TECHDOC
%%
%%  Author: Geert-Jan M. Kruijff
%%          Language Technology Lab
%%          DFKI
%%          Saarbruecken, Germany
%%  Email:  gj@dfki.de
%%
%%  TECHDOC modifies the standard article class for LaTeX2e.
%%
%%  -------------------------------------------------------------------------
%%
%%  Commands for title: 
%%  * \author : Required; author(s) of the paper
%%  * \title  : Required; title of the paper
%%  * \address: Required; address of the author(s)
%%  * \project: Required; project of the authors
%%  * \email  : Required; email address(es) of the author(s)
%%  * \doctype: type of document; ``Technical Note'' by default
%%
%%  Commands for headers:
%%  * \headertitle:  by default same as \title, change if necessary
%%  * \headerauthor: by default same as \author, change if necessary
%%
%% =========================================================================

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{techdoc}[2004/10/08 technical document style]
\typeout{}
\typeout{TECHDOC style, Geert-Jan M. Kruijff, Nov 2004}
\typeout{=============================================}
\typeout{Note: This package requires ``times'' for improved PDF output}
\RequirePackage{times}


\newcommand{\myemail}{}
\newcommand{\email}[1]{\renewcommand{\myemail}{#1}}
\newcommand{\myaddress}{}
\newcommand{\address}[1]{\renewcommand{\myaddress}{#1}}
\newcommand{\myproject}{}
\newcommand{\project}[1]{\renewcommand{\myproject}{#1}}
\newcommand{\myauthor}{me}
\newcommand{\mytitle}{title}
\newcommand{\headerauthor}[1]{\renewcommand{\myauthor}{#1}}
\newcommand{\headertitle}[1]{\renewcommand{\mytitle}{#1}}
\newcommand{\mydoctype}{Technical Note}
\newcommand{\doctype}[1]{\renewcommand{\mydoctype}{#1}}
\newcommand{\mydocid}{}
\newcommand{\docid}[1]{\renewcommand{\mydocid}{#1}}

\newcommand{\comment}[1]{
        \noindent{\scriptsize \fbox{Comment} \textsf{#1}\\}}
\newcommand{\marginlabel}[1]{
        \mbox{}\marginpar{\raggedleft\hspace{0pt}\scriptsize{\textsf{#1}}}}

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax
\LoadClass{article}


\def\@maketitle{%
  \newpage
  \null
%%  \vskip 2em%
  \begin{flushleft}%
  \let \footnote \thanks
    {
    \noindent\rule{\linewidth}{.2mm}
    \large {\textbf\mydoctype ~\mydocid} 
    \vskip .5em
    \LARGE \@title 
    \par}%
    \vskip 1.5em%
    {\large
      \lineskip .5em%
      \noindent\begin{tabular}{l}
                        \@author  \\[.5em]
                        {\small \myproject}\\
                        {\small {\textsl \myaddress}} \\
                        {\small $\langle${\texttt \myemail}}$\rangle$ 
                \end{tabular}
        \\
      \par}%
    \vskip .8em%
       
  \end{flushleft}%
  \par
  %% \noindent\rule{\linewidth}{.2mm}
  \vskip 1em
}

\if@titlepage
  \renewenvironment{abstract}{%
      \titlepage
      \null\vfil
      \@beginparpenalty\@lowpenalty
      \begin{flushleft}%
        \bfseries \abstractname
        \@endparpenalty\@M
      \end{flushleft}
      \noindent\rule{\linewidth}{.2mm}
     }%
     {\par    
       \vspace{-.5em}
       \noindent\rule{\linewidth}{.2mm}\par
       \vfil\null\endtitlepage}
\else
  \renewenvironment{abstract}{%
      \if@twocolumn
        %% \section*{\abstractname}%
          \noindent{\bf \abstractname.} \hspace{.1em} %\vspace{-.5em}\vspace{\z@}}%
      \else
        \footnotesize
        %% \vspace{-1em}
        %%\noindent\begin{flushleft}%
          \noindent{\bf \abstractname.} \hspace{.1em} %\vspace{-.5em}\vspace{\z@}}%
        %%\end{flushleft}%
        %%\noindent\quotation      
      \fi 
      }
      {\if@twocolumn\else\endquotation\fi}
\fi 




%\def\@oddhead{\noindent\rule{\linewidth}{.2mm}\\\par \myauthor, \mytitle}
%\def\@evenhead{\noindent\rule{\linewidth}{.2mm}\\  \myauthor, \mytitle}

\def\@oddhead{\hfill\begin{tabular}{lcr} \textsl{\small{\mytitle}} &  \hfill & \textsl{\small{\myauthor}}\end{tabular}\hfill}
\def\@evenhead{\hfill\begin{tabular}{lcr} \textsl{\small{\mytitle}} & \hfill & \textsl{\small{\myauthor}}\end{tabular}\hfill}

\renewcommand\tableofcontents{%
%%    \vspace{-.5em}
%%     \noindent\rule{\linewidth}{.2mm}\par

%\begin{flushleft}\small\sc Contents\end{flushleft}
    \@starttoc{toc}%
    }


%% ============================================================
%% 
%% Development Log: 
%% * Begin edit: 2004 Nov 1
%% * Last  edit: 2004 Nov 1
%%
%% ------------------------------------------------------------
%% [Nov 1,GJ]  Created class file 
%%
%%















