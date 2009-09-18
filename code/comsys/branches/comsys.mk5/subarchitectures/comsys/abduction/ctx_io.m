:- module ctx_io.

:- interface.
:- import_module stringable.

:- import_module ctx_impl, ctx_modality.

:- instance stringable(ctx_modality).
:- instance parsable(ctx_modality).

%------------------------------------------------------------------------------%

:- implementation.

:- instance stringable(ctx_modality) where [
	func(to_string/1) is ctx_modality_to_string
].

:- instance parsable(ctx_modality) where [
	func(from_string/1) is ctx_modality_from_string
].

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred ctx_modality_as_string(ctx_modality, string).
:- mode ctx_modality_as_string(in, out) is det.
:- mode ctx_modality_as_string(out, in) is semidet.

ctx_modality_as_string(e(past), "eP").
ctx_modality_as_string(e(this), "e0").
ctx_modality_as_string(e(next), "eN").
ctx_modality_as_string(e(future), "eF").

ctx_modality_as_string(a(past), "aP").
ctx_modality_as_string(a(this), "a0").
ctx_modality_as_string(a(next), "aN").
ctx_modality_as_string(a(future), "aF").

ctx_modality_as_string(i(past), "iP").
ctx_modality_as_string(i(this), "i0").
ctx_modality_as_string(i(next), "iN").
ctx_modality_as_string(i(future), "iF").

ctx_modality_as_string(any, "[]").

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func ctx_modality_to_string(ctx_modality) = string.

ctx_modality_to_string(Rep) = S :-
	ctx_modality_as_string(Rep, S).

:- func ctx_modality_from_string(string::in) = (ctx_modality::out) is semidet.

ctx_modality_from_string(S) = Rep :-
	ctx_modality_as_string(Rep, S).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
