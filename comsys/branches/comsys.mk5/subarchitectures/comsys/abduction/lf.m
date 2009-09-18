% $Id: lf.m 1213 2009-07-07 10:51:16Z janicek $

:- module lf.

:- interface.

:- import_module string.

:- typeclass ontological_sort(T) where [

		% subsumes(A, B)
		% True iff A < B.
	pred subsumes(T, T),
	mode subsumes(in, in) is semidet
].

:- type proposition == string.

:- type id(I, S)
	--->	of_sort(I, S).

:- type lf(I, S, R)
	--->	at(id(I, S), lf(I, S, R))
	;	i(id(I, S))
	;	r(R, lf(I, S, R))
	;	p(proposition)
	;	and(lf(I, S, R), lf(I, S, R))
	.

:- type lf == lf(string, string, string).

:- instance ontological_sort(string).

%------------------------------------------------------------------------------%

:- implementation.

:- instance ontological_sort(string) where [
	pred(subsumes/2) is string_subsumes
].

:- pred string_subsumes(string::in, string::in) is semidet.

string_subsumes(S, S).
