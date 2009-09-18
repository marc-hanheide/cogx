% $Id: context.m 1266 2009-07-13 01:58:02Z janicek $

:- module ctx_modality.

:- interface.

:- import_module modality.

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

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

is_ctx_modality(_).
