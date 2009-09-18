:- module ontology.

:- interface.

:- typeclass isa_ontology(T) where [

		% direct_isa(B, A)
		% True iff B `is-a` A.
		%
		% That is, w:B -> w:A.
	pred direct_isa(T, T),
	mode direct_isa(in, in) is semidet,
	mode direct_isa(in, out) is nondet
].

	% reflexive and transitive closure of direct_isa
:- pred isa(T, T) <= isa_ontology(T).
:- mode isa(in, in) is semidet.

	% more_specific(X, Y) = Z
	%
	% X is-a Y -> Z = X
	% Y is-a X -> Z = Y
	%
	% fail otherwise
	%
:- func more_specific(T, T) = T <= isa_ontology(T).
:- mode more_specific(in, in) = out is semidet.

%------------------------------------------------------------------------------%

:- implementation.

isa(X, X).
isa(X, Y) :-
	direct_isa(X, Z),
	Z \= X,
	isa(Z, Y).

more_specific(X, Y) = Z :-
	(isa(X, Y) -> Z = X ;
	(isa(Y, X) -> Z = Y ;
	fail
	)).
