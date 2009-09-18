% $Id: lf.m 1213 2009-07-07 10:51:16Z janicek $

:- module lf.

:- interface.

:- import_module string.

:- typeclass ontological_sort(T).

:- type index == string.
:- type sort == string.
:- type relation == string.
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

%------------------------------------------------------------------------------%

:- implementation.

:- typeclass ontological_sort(T) where [
	pred subsumes(T, T),
	mode subsumes(in, in) is semidet
].
