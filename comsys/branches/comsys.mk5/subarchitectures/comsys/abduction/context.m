% $Id$

:- module context.

:- interface.

:- import_module list, set, string.
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

:- pred string_as_ctx_ref(string, ctx_ref).
:- mode string_as_ctx_ref(in, out) is semidet.
:- mode string_as_ctx_ref(out, in) is det.

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

:- pred effect(vsmprop) `with_type` ctx_change.
:- mode effect(in) `with_inst` ctx_change.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred dialogue_turn(proof::in, d_ctx::in, d_ctx::out) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module set, pair.

string_as_ctx_ref("e0", evt(this)).
string_as_ctx_ref("e1", evt(next)).
string_as_ctx_ref("a0", att(this)).
string_as_ctx_ref("a1", att(next)).
string_as_ctx_ref("i0", info(this)).
string_as_ctx_ref("i1", info(next)).
string_as_ctx_ref("all", axiom).

%------------------------------------------------------------------------------%

new_d_ctx = d_ctx(kb.init, set.init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% TODO check this again to be sure.
	% TODO: "all"
compatible(C, C).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred add_to_focus(string) `with_type` ctx_change.
:- mode add_to_focus(in) `with_inst` ctx_change.

add_to_focus(A, DC0, DC) :-
	F0 = DC0^d_focus,
	set.insert(F0, A, F),
	DC = DC0^d_focus := F.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

effect(VSMProp, !DC) :-
	(if VSMProp = vs(m(_, f("in_focus", [a(Arg)])), _VS)
		%member(att(next), Ctx)
	then add_to_focus(Arg, !DC)
	else true
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% TODO: check that it's a completed proof
dialogue_turn(Pr, !DC) :-

	% blank focus
	!:DC = !.DC^d_focus := set.init,

	(if Pr^p_goals = vs([LastGoal|_], Varset)
	then
		% call the effects
		list.foldl((pred(MProp-_Marking::in, !.DC::in, !:DC::out) is det :-
			effect(vs(MProp, Varset), !DC)  % XXX XXX XXX
			), LastGoal, !DC)

	else error("No goals in the proof.")
	).
