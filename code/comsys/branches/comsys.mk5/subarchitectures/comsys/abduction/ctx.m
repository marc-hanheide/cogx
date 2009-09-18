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

:- type ctx
	--->	a(ref)
	;	i(ref)
	;	e(ref)
	;	any
	.

:- instance modality(ctx).
:- instance stringable(ctx).

:- pred is_ctx(ctx::in) is det.

%==============================================================================%

:- implementation.

:- import_module require.
:- import_module set, pair.

%------------------------------------------------------------------------------%

:- instance modality(ctx) where [
	func(axiom/0) is ctx_axiom,
	func(compose/2) is ctx_compose
].

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func ctx_axiom = ctx.

ctx_axiom = any.

:- func ctx_compose(ctx::in, ctx::in) = (ctx::out) is semidet.

ctx_compose(_, _) = _ :-
	fail.

%------------------------------------------------------------------------------%

:- instance stringable(ctx) where [
	func(to_string/1) is ctx_to_string,
	func(from_string/1) is ctx_from_string
].

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred ctx_as_string(ctx, string).
:- mode ctx_as_string(in, out) is det.
:- mode ctx_as_string(out, in) is semidet.

ctx_as_string(e(past), "eP").
ctx_as_string(e(this), "e0").
ctx_as_string(e(next), "eN").
ctx_as_string(e(future), "eF").

ctx_as_string(a(past), "aP").
ctx_as_string(a(this), "a0").
ctx_as_string(a(next), "aN").
ctx_as_string(a(future), "aF").

ctx_as_string(i(past), "iP").
ctx_as_string(i(this), "i0").
ctx_as_string(i(next), "iN").
ctx_as_string(i(future), "iF").

ctx_as_string(any, "[]").

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func ctx_to_string(ctx) = string.

ctx_to_string(Rep) = S :-
	ctx_as_string(Rep, S).

:- func ctx_from_string(string::in) = (ctx::out) is semidet.

ctx_from_string(S) = Rep :-
	ctx_as_string(Rep, S).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

is_ctx(_).
