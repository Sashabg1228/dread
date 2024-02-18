% =============================================================================
% tu_coursework.cls
% Шаблон за курсова работа в ТУ-София, адаптиран от:
%
% l4proj.cls
%
% Template for final year projects in the School of Computing Science at the
% University of Glasgow. Adapted from the level 3 project template originally
% provided by Colin Perkins <csp@dcs.gla.ac.uk>.
%
% =============================================================================

\ProvidesClass{tu_coursework}[%
2009/01/09 Colin Perkins <csp@dcs.gla.ac.uk>.%
2009/09/16 Modified by Tim Storer <tws@dcs.gla.ac.uk> to support non pdf modes.%
2011/02/15 Modified by Gregg Hamilton <hamiltgr@dcs.gla.ac.uk> for use in final year projects.%
2012/10/18 Modified by Patrick Prosser, use geometry to control margins, simplified.%
2018/09/14 Modified by John Williamson, improve typographic appearance%
2020/05/30 Adapted by Vladimir Garistov for Technical University - Sofia%
]

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{scrreprt}}
\ProcessOptions

\LoadClass[
	twoside=true,				% onesite or twoside printing
	open=any,					% doublepage cleaning ends up right side
	parskip=half,				% spacing value / method for paragraphs
	chapterprefix=false,		% do not display a prefix for chapters
	appendixprefix=true,		% but display a prefix for appendix chapter
	fontsize=12pt,				% font size
	headings=normal,			% size of headings
	bibliography=totoc,			% include bib in toc
	listof=totoc,				% include listof entries in toc
	titlepage=true,				% own page for each title page
	captions=belowtable,		% display table captions below the float env
	draft=false,				% value for draft version
	DIV=9,						% suppress DIV warnings, overwritten by cleanthesis.sty
]{scrreprt}

% Monospace font was Fira Mono Sans and the body font was FBB (Bembo).
% Removed as cleanthesis overwrites them anyway.
\RequirePackage{iftex}
\ifPDFTeX
	\RequirePackage[utf8]{inputenc}
	\RequirePackage[T2A,T1]{fontenc}
	\RequirePackage[main=bulgarian,english]{babel}
	\pdfminorversion=7
\else
	% make sure the fonts are actually set
	\RequirePackage{fontspec}
	\RequirePackage{polyglossia}
\fi

% cleanthesis style
\RequirePackage[
	figuresep=colon,
	sansserif=true,
	hangfigurecaption=false,
	hangsection=true,
	hangsubsection=true,
	configurelistings=false,
	colorize=full,
	colortheme=bluemagenta,
	configurebiblatex=true,
	bibsys=biber,
	bibfile=references.bib,
	bibstyle=ieee,
	bibsorting=nty,
]{cleanthesis}

% Load packages
\RequirePackage{indentfirst}
\RequirePackage{scrhack}								% fixes a KOMA-Script related warning
\RequirePackage{xifthen}
\RequirePackage[titletoc]{appendix}
\RequirePackage{pdfpages}
\RequirePackage{placeins}
\RequirePackage{multirow}
\RequirePackage{makecell}
\RequirePackage{csvsimple}
\RequirePackage[cal=boondoxo]{mathalpha}				% mathcal
\RequirePackage{amsmath,amsfonts,amsbsy}
\RequirePackage{mathtools}
\RequirePackage[style]{abstract}						% abstract styling
\RequirePackage{etoolbox}
\RequirePackage{float}
\RequirePackage{numprint}								% nice number printing
\RequirePackage{subcaption}								% allow subfloats
\RequirePackage{booktabs}
\RequirePackage{tcolorbox}
\RequirePackage{soul}
\RequirePackage{lstautogobble}
%====================
\RequirePackage{wrapfig}
\RequirePackage{array}
%====================

\RequirePackage[exponent-product=\cdot]{siunitx}		% units for nice formatting of numbers
\sisetup{mode = match}

\RequirePackage[americancurrents, americanvoltages, europeanresistors, americaninductors, siunitx, nooldvoltagedirection]{circuitikz}

\RequirePackage{multicol}
\setlength{\columnsep}{1.5pc}

\RequirePackage{listings}

% from: https://gist.github.com/FelipeCortez/10729134
% set up listings for prettier output
\definecolor{bluekeywords}{rgb}{0.13, 0.19, 0.7}
\definecolor{greencomments}{rgb}{0.1, 0.5, 0.2}
\definecolor{redstrings}{rgb}{0.8, 0.15, 0.1}
\definecolor{graynumbers}{rgb}{0.5, 0.5, 0.5}
\definecolor{subtlegray}{rgb}{0.98, 0.98, 0.98}

\lstset{
	autogobble,	
	columns=fullflexible,
	showspaces=false,
	showtabs=false,
	breaklines=true,
	showstringspaces=false,
	breakatwhitespace=true,
	escapeinside={(*@}{@*)},
	rulecolor=\color{lightgray},
	backgroundcolor=\color{subtlegray},
	commentstyle=\color{greencomments},
	keywordstyle=\color{bluekeywords},
	stringstyle=\color{redstrings},
	numbers=left,
	numberstyle=\color{graynumbers},
	basicstyle=\ttfamily\linespread{1.15}\footnotesize,
	frame=tb,
	framesep=12pt,
	framexleftmargin=12pt,
	tabsize=4,
	captionpos=b
}
\renewcommand{\lstlistingname}{Програмен код}

% Algorithm typesetting package
\RequirePackage[plain, noline, shortend, linesnumberedhidden]{algorithm2e}
\SetAlFnt{\sffamily \small}

\RequirePackage{tikz}
\usetikzlibrary{shapes.geometric, arrows}
\tikzstyle{startstop} = [rectangle, rounded corners, minimum width=3cm, minimum height=1cm,text centered, draw=black, fill=red!30]
\tikzstyle{io} = [trapezium, trapezium left angle=70, trapezium right angle=110, minimum width=3cm, minimum height=1cm, text centered, draw=black, fill=blue!30]
\tikzstyle{process} = [rectangle, minimum width=3cm, minimum height=1cm, text centered, draw=black, fill=orange!30]
\tikzstyle{decision} = [diamond, minimum width=1cm, minimum height=1cm, text centered, draw=black, fill=green!30, inner sep = -0.4cm]
\tikzstyle{arrow} = [thick,->,>=stealth]
\tikzstyle{arrow_no_head} = [thick]

% Footnote font size
\renewcommand{\footnotesize}{\fontsize{9pt}{11pt}\selectfont}

% make all cross-references clickable
\RequirePackage[hidelinks,unicode]{hyperref}
\hypersetup{						% hyperref settings
	plainpages=false,				%
	pdfborder={0 0 0},				%
	breaklinks=true,				% allow newlines in links
	bookmarksnumbered=true,			%
	bookmarksopen=true				%
}

% Must be loaded after hyperref
\RequirePackage[nameinlink,bulgarian]{cleveref_bg}

\textheight=24cm

% Metadata
\newcommand{\@doctype}{}
\newcommand{\doctype}[1]{\renewcommand{\@doctype}{#1}}
\newcommand{\@speciality}{}
\newcommand{\speciality}[1]{\renewcommand{\@speciality}{#1}}
%====================
\newcommand{\@supervisor}{}
\newcommand{\supervisor}[1]{\renewcommand{\@supervisor}{#1}}
\newcommand{\@supervisorchair}{}
\newcommand{\supervisorchair}[1]{\renewcommand{\@supervisorchair}{#1}}
\newcommand{\@reviewer}{}
\newcommand{\reviewer}[1]{\renewcommand{\@reviewer}{#1}}
\newcommand{\@reviewerchair}{}
\newcommand{\reviewerchair}[1]{\renewcommand{\@reviewerchair}{#1}}
\newcommand{\@admgroup}{}
\newcommand{\admgroup}[1]{\renewcommand{\@admgroup}{#1}}
\newcommand{\@faculty}{}
\newcommand{\faculty}[1]{\renewcommand{\@faculty}{#1}}
\newcommand{\@fac}{}
\newcommand{\fac}[1]{\renewcommand{\@fac}{#1}}
\newcommand{\@facnumber}{}
\newcommand{\facnumber}[1]{\renewcommand{\@facnumber}{#1}}
\newcommand{\@city}{}
\newcommand{\city}[1]{\renewcommand{\@city}{#1}}

% Define the title page
\renewcommand\maketitle
{
	\begin{titlepage}
		\definecolor{UniBlue}{HTML}{00355F}

		%\let\footnotesize\small
		\let\footnoterule\relax
		\let\footnote \thanks

		% the logo,
        \begin{figure}[h]
            %\setcapwidth{0.6\textwidth}
            \makebox[\textwidth][l]{%
                \begin{minipage}[t]{1.2\textwidth}
                    %\vspace{1in}
                    \begin{wrapfigure}{l}{0.15\linewidth}
            			\includegraphics[width=3cm]{images/elsys-logo.png}
                    \end{wrapfigure}
            
                    \color{UniBlue}
            	    \sffamily\@faculty
                \end{minipage}%
            }
        \end{figure}
        
		% title and author
		\vspace{1cm}
		\large
		\vskip 8em%
		\begin{center}
			\noindent
			\fontsize{35}{35}
			\upshape\@doctype\par
			\normalsize
			\Large
			на тема\par
			\noindent
			\upshape\@title\par
		\end{center}
		\vfill
		
		\begin{multicols}{2}
			\noindent
			Дипломант:\\
			{
				\textbf{\@author}\\
				\textbf
				{
					специалност \@speciality
				}
			}
			\vfill\null
			\columnbreak
			\noindent
			Дипломен ръководител:\\
			{
				\textbf
				{
					\@supervisor\\
					\ifthenelse{\equal{\@supervisorchair}{}}{}
					{
						катедра 
						\@supervisorchair}
					}
			}
		
			\ifthenelse{\equal{\@reviewer}{}}{}
			{
				\noindent
				Рецензент:\\
				{
					\large
					\textbf
					{
						\@reviewer\\
						\ifthenelse{\equal{\@reviewerchair}{}}{}
						{
							катедра 
							\@reviewerchair}
						}
				}
			}
		
			\vfill
		\end{multicols}
		
		\vfill
		
		\begin{center}
			\@date\\
			\@city
		\end{center}
		\normalsize
	\end{titlepage}

	\setcounter{footnote}{0}%
	\global\let\thanks\relax
	\global\let\maketitle\relax
	\global\let\@thanks\@empty
	\global\let\@author\@empty
	\global\let\@supervisor\@empty
	\global\let\@reviewer\@empty
	\global\let\@admgroup\@empty
	\global\let\@faculty\@empty
	\global\let\@fac\@empty
	\global\let\@facnumber\@empty
	\global\let\@date\@empty
	\global\let\@city\@empty
	\global\let\@consentname\@empty
	\global\let\@consentdate\@empty

	\global\let\@title\@empty
	\global\let\title\relax
	\global\let\author\relax
	\global\let\supervisor\relax
	\global\let\reviewer\relax
	\global\let\admgroup\relax
	\global\let\faculty\relax
	\global\let\fac\relax
	\global\let\facnumber\relax
	\global\let\date\relax
	\global\let\city\relax
	\global\let\and\relax
}

% Educational consent form
\newcommand{\educationalconsent}{
	\ifdefined\consentname
		  \newpage
		  \chapter*{Education Use Consent}

		  I hereby grant my permission for this project to be stored, distributed and shown to other
		  University of Glasgow students and staff for educational purposes. 
		  \textbf{Please note that you are under no obligation to sign 
		  this declaration, but doing so would help future students.}

		  \begin{tabular}{@{}llll}
				&					 &			&					 \\
				&					 &			&					 \\
		  Signature: & \consentname & Date: & \consentdate \\
				&					 &			&					 \\
				&					 &			&					 \\
		  \end{tabular}
	\else
	\newpage
		\chapter*{Education Use Consent}
		Consent for educational reuse withheld. Do not distribute.
	\fi
}

% Highlight boxes
\newenvironment{highlight}
{\begin{tcolorbox}[notitle,boxrule=0pt,colback=green!10,colframe=green!10]}
{
\end{tcolorbox}
}
\newenvironment{highlight_title}[1]
{\begin{tcolorbox}[title=#1,boxrule=2pt,colback=green!10,colframe=green!20,coltitle=black,fonttitle=\bfseries]}
{
\end{tcolorbox}
}

% Adjust margins
\setlength{\parindent}{12pt}

% Use (a) in subfigures
\captionsetup{subrefformat=parens}

% Switch to roman numeral for frontmatter
\pagenumbering{Roman}

% Fix itemise so it looks reasonable
\renewcommand{\@listI}{%
	  \leftmargin=25pt
	  \rightmargin=0pt
	  \labelsep=5pt
	  \labelwidth=20pt
	  \itemindent=0pt
	  \listparindent=0pt
	  \topsep=0pt plus 2pt minus 4pt
	  \partopsep=0pt plus 1pt minus 1pt
	  \parsep=1pt plus 1pt
	  \itemsep=\parsep
}

% Allow highlighting of text for todo notes
\newcommand{\todo}[1]{{\large \hl{TODO: #1}\PackageWarning{TODO:}{#1!}}}

% make urls less bulky and ugly
\renewcommand{\UrlFont}{\ttfamily\small}

% Dirty fix for a formatting issues, caused by redefining \appendix in hyperref
\renewcommand{\appendix}
{%
	\setcounter{chapter}{0}
	\setcounter{section}{0}
	\renewcommand\chapapp{\appendixname}
	\renewcommand\thechapter{\Alph{chapter}}
	\renewcommand\theHchapter{\Alph{chapter}}
	\renewcommand{\chaptermark}[1]{%
		\markboth{%
			\ctfontfootertext
			% use \@chapapp instead of \chaptername to avoid
			% 'Chapter A Appendix ...', thanks to @farbverlust (issue #47)
			{\color{ctcolorfootermark}\textbf{\chapapp\ \thechapter}}%
			\hspace{.25cm}%
			%\protect\begin{minipage}[t]{.65\textwidth}%
				##1%
			%\protect\end{minipage}%
		}{}%
	}
	%\cleardoublepage
	\clearpage
	\addcontentsline{toc}{chapter}{Приложения}%
}

% Font size for mathematical expressions
\DeclareMathSizes{10}{12}{10}{9}
