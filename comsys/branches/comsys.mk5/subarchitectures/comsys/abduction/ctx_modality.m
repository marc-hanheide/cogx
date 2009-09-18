% $Id: context.m 1266 2009-07-13 01:58:02Z janicek $

:- module ctx_modality.

:- interface.

:- import_module modality.
:- import_module set.

:- type foreground
	--->	com
	.

:- type agent
	--->	human
	;	robot
	.

:- type belief
	--->	private(agent)
	;	attrib(agent, agent)
	;	mutual(set(agent))
	.

:- type stf
	--->	now.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type ctx_modality
	--->	any  % the axiom
	;	a(foreground)
	;	e(stf)
	;	i
	;	k(stf, belief)
	;	t(stf, belief)
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

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

is_ctx_modality(_).

%------------------------------------------------------------------------------%

:- func stf_compose(stf::in, stf::in) = (stf::out) is semidet.

stf_compose(now, now) = now.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func ctx_modality_compose(ctx_modality::in, ctx_modality::in) = (ctx_modality::out) is semidet.

ctx_modality_compose(_, _) = _ :-
	fail.
