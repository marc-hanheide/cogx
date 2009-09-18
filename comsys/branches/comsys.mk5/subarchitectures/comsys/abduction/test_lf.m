:- module test_lf.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module lf.

main(!IO) :-
	LF = at(of_sort("v1", "object"), and(p("box"), r("colour", and(i(of_sort("c", "colour")), p("something"))))),
	std_lf(LF),

	print(lf_to_string(LF), !IO),
	nl(!IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred std_lf(lf::in) is det.

std_lf(_).
