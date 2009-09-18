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

match([], []).
match([H|TL], R) :-
	(if H = axiom
	then
		( match(TL, R)  % empty sequence, end
		; R = [_|TR], match([H|TL], TR)  % eat one, continue
		)
	else
		R = [H|TR], 
		match(TL, TR)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

compose_list([]) = [].
compose_list([H]) = [H].

compose_list([H,I|T]) = C :-
	(if compose(H, I) = J
	then C = [J|compose_list(T)]
	else C = [H, I|compose_list(T)]
	).
