:- module stringable.

:- interface.

:- import_module string.

:- typeclass stringable(T) where [
	func to_string(T) = string
].

:- typeclass parsable(T) where [
	func from_string(string::in) = (T::out) is semidet
].

:- func det_from_string(string) = T <= parsable(T).

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

det_from_string(S) = Rep :-
	(if Rep0 = from_string(S)
	then Rep = Rep0
	else error("failed to parse string `" ++ S ++ "' in func det_from_string/1.")
	).
