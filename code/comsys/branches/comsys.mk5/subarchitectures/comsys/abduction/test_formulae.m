% $Id$

:- module test_formulae.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module string, pair, list.
:- import_module formulae.

main(!IO) :-
	test_term_parse("a(b).", !IO),
	test_term_parse("a(X).", !IO),
	test_term_parse("a(Y).", !IO),
	test_term_parse("t : a(x).", !IO).
	
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred test_term_parse(string::in, io::di, io::uo) is det.

test_term_parse(S, !IO) :-
	string_to_term_varset(S, T, V),
	format("* `%s':\n  parsed=%s\n\n", [s(S), s(string(T-V))], !IO).
