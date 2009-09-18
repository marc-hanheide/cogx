% $Id: context.m 1266 2009-07-13 01:58:02Z janicek $

:- module ctx.

:- interface.

:- import_module modality.
:- import_module stringable.

:- type ref
	--->	this
	;	next
	;	future
	;	past
	.

:- type ctx_modality
	--->	a(ref)
	;	i(ref)
	;	e(ref)
	;	any
	.

:- instance modality(ctx_modality).
:- instance stringable(ctx_modality).

:- pred is_ctx_modality(ctx_modality::in) is det.

%==============================================================================%

:- implementation.

:- import_module require.
:- import_module set, pair.

%------------------------------------------------------------------------------%

:- instance modality(ctx_modality) where [
	func(axiom/0) is ctx_modality_axiom,
	func(compose/2) is ctx_modality_compose
].

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func ctx_modality_axiom = ctx_modality.

ctx_modality_axiom = any.

:- func ctx_modality_compose(ctx_modality::in, ctx_modality::in) = (ctx_modality::out) is semidet.

ctx_modality_compose(_, _) = _ :-
	fail.

%------------------------------------------------------------------------------%

:- instance stringable(ctx_modality) where [
	func(to_string/1) is ctx_modality_to_string,
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

is_ctx_modality(_).
