:- module modality.

:- interface.

:- import_module list.

:- typeclass modality(T) where [
	func axiom = T,
	func compose(T::in, T::in) = (T::out) is semidet
].

:- func compose_list(list(T)) = list(T) <= modality(T).

:- pred match(list(T)::in, list(T)::in) is semidet <= modality(T).

%------------------------------------------------------------------------------%

:- implementation.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% axiom has S4 semantics:
	%  (K) ... [](B -> A) -> ([]B -> []A)
	%  (T) ... []A -> A
	%  (4) ... []A -> [][]A
	%
	% plus specialisation:
	%  []A -> [a]A
	%    for every a

match([], []).
match([axiom], []).
match([], [axiom]).
match([HL|TL], [HR|TR]) :-
	(if HL = axiom
	then
		( match(TL, [HR|TR])  % empty sequence, end
		; match([HL|TL], TR)  % eat one, continue
		)
	else
		(if HR = axiom
		then
			match([HR|TR], [HL|TL])
		else
			HL = HR,
			match(TL, TR)
		)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

compose_list([]) = [].
compose_list([H]) = [H].

compose_list([H,I|T]) = C :-
	(if compose(H, I) = J
	then C = [J|compose_list(T)]
	else C = [H, I|compose_list(T)]
	).
