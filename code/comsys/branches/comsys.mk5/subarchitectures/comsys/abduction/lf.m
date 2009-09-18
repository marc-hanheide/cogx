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

:- func lf_to_string(lf) = string.

%------------------------------------------------------------------------------%

:- implementation.

:- typeclass ontological_sort(T) where [
	pred subsumes(T, T),
	mode subsumes(in, in) is semidet
].

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func id_to_string(id(string, string)) = string.

id_to_string(of_sort(I, S)) = I ++ ":" ++ S.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

lf_to_string(at(Id, LF)) = "@" ++ id_to_string(Id) ++ "(" ++ lf_to_string(LF) ++ ")".
lf_to_string(i(Id)) = id_to_string(Id).
lf_to_string(r(Rel, LF)) = "<" ++ Rel ++ ">" ++ "(" ++ lf_to_string(LF) ++ ")".
lf_to_string(p(Prop)) = Prop.
lf_to_string(and(LF1, LF2)) = lf_to_string(LF1) ++ " ^ " ++ lf_to_string(LF2).
