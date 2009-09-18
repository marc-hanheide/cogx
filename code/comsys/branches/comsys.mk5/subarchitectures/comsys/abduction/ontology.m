:- module ontology.

:- interface.

:- typeclass isa_ontology(T) where [

		% isa(B, A)
		% True iff B `is-a` A.
		%
		% That is, w:B -> w:A.
	pred isa(T, T),
	mode isa(in, in) is semidet
].
