:- module test_lf.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module lf.

main(!IO) :-
	LF = at(of_sort("v1", "object"), and(p("box"), p("something"))),
	std_lf(LF),

	print(LF, !IO),
	nl(!IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred std_lf(lf::in) is det.

std_lf(_).
