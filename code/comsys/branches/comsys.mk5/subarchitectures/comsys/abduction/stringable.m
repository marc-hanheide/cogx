:- module stringable.

:- interface.

:- import_module string.

	% a in injective mapping T -> string
:- typeclass stringable(T) where [
	func to_string(T) = string,
	func from_string(string::in) = (T::out) is semidet
].

:- func det_from_string(string) = T <= stringable(T).

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

det_from_string(S) = Rep :-
	(if Rep0 = from_string(S)
	then Rep = Rep0
	else error("failed to parse string `" ++ S ++ "' in func det_from_string/1.")
	).
