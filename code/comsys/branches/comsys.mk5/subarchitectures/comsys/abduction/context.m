% $Id$

:- module context.

:- interface.

:- import_module list, set, pair, string.
:- import_module kb, formulae, abduction.

:- type ref
	--->	this
	;	next
	.

:- type ctx == ctx_ref.

:- type ctx_ref
	--->	att(ref)
	;	info(ref)
	;	evt(ref)
	;	axiom
	.

	% subsumes(C, C1)
	% True iff C subsumes C1.
	%
:- pred compatible(list(ctx)::in, list(ctx)::in) is semidet.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type d_ctx
	--->	d_ctx(
		d_kb :: kb,
		d_focus :: set(string)
	).

:- func new_d_ctx = d_ctx.

:- type ctx_change == pred(d_ctx, d_ctx).
:- inst ctx_change == (pred(in, out) is det).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred effect(ctxterm) `with_type` ctx_change.
:- mode effect(in) `with_inst` ctx_change.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred dialogue_turn(proof::in, d_ctx::in, d_ctx::out) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module set.
:- import_module term.

new_d_ctx = d_ctx(kb.init, set.init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% TODO check this again to be sure.
compatible(C, C1) :-
	SC = set.from_list(C),
	SC1 = set.from_list(C1),
	(if member(axiom, SC)
	then true
	else superset(SC, SC1)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred add_to_focus(string) `with_type` ctx_change.
:- mode add_to_focus(in) `with_inst` ctx_change.

add_to_focus(A, DC0, DC) :-
	F0 = DC0^d_focus,
	set.insert(F0, A, F),
	DC = DC0^d_focus := F.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

effect(Term, !DC) :-
	(if Term = _Ctx-functor(atom(":"), [_, functor(atom("in_focus"), [functor(atom(Arg), [], _)], _)], _)
		%member(att(next), Ctx)
	then add_to_focus(Arg, !DC)
	else true
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% TODO: check that it's a completed proof
dialogue_turn(Pr, !DC) :-

	% blank focus
	!:DC = !.DC^d_focus := set.init,

	(if Pr^p_goals = [LastGoal|_]
	then
		% call the effects
		list.foldl((pred((Ctx-Term)-Marking::in, !.DC::in, !:DC::out) is det :-
			effect(Ctx-Term, !DC)
			), LastGoal, !DC)

	else error("No goals in the proof.")
	).
