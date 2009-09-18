% $Id$

:- module context.

:- interface.

:- import_module set, string.
:- import_module formula, abduction.

:- func apply_cost_function(d_ctx, string, vsmprop) = float.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type ctx.

:- func new_ctx = ctx.

:- pred add_fact(vsmprop::in, ctx::in, ctx::out) is det.
:- pred add_rule(vsmrule::in, ctx::in, ctx::out) is det.

:- pred fact(ctx::in, vsmprop::out) is nondet.
:- pred rule(ctx::in, vsmrule::out) is nondet.
:- pred assumable(ctx::in, vsmprop::out) is nondet.

	% for debugging purposes only!
:- func facts(ctx) = set(vsmprop).
:- func rules(ctx) = set(vsmrule).

%------------------------------------------------------------------------------%

:- type d_ctx
	--->	d_ctx(
		d_ctx :: ctx,
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
:- import_module set, list, pair.

:- type ctx
	--->	ctx(
		ctx_facts :: set(vsmprop),
		ctx_rules :: set(vsmrule)  % this doesn't really belong here, does it?
	).

new_ctx = ctx(init, init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

add_fact(Prop, Ctx0, Ctx) :-
	Facts = Ctx0^ctx_facts,
	Ctx = Ctx0^ctx_facts := set.insert(Facts, Prop).

add_rule(Rule, Ctx0, Ctx) :-
	Rules = Ctx0^ctx_rules,
	Ctx = Ctx0^ctx_rules := set.insert(Rules, Rule).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

facts(Ctx) = Ctx^ctx_facts.

rules(Ctx) = Ctx^ctx_rules.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

fact(Ctx, Fact) :-
	set.member(Fact, Ctx^ctx_facts).

rule(Ctx, Rule) :-
	set.member(Rule, Ctx^ctx_rules).

assumable(_Ctx, _) :-
	fail.

%------------------------------------------------------------------------------%
%------------------------------------------------------------------------------%
%------------------------------------------------------------------------------%

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

new_d_ctx = d_ctx(new_ctx, set.init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred add_to_focus(string) `with_type` ctx_change.
:- mode add_to_focus(in) `with_inst` ctx_change.

add_to_focus(A, DC0, DC) :-
	F0 = DC0^d_focus,
	set.insert(F0, A, F),
	DC = DC0^d_focus := F.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

effect(VSMProp, !DC) :-
	(if VSMProp = vs(m(_, p("in_focus", [t(Arg, [])])), _VS)
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

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

apply_cost_function(DCtx, FName, VSMProp) = Cost :-
	(if
		Cost0 = apply_cost_function0(DCtx, FName, VSMProp)
	then
		Cost = Cost0
	else
		error("can't find cost function " ++ FName)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func apply_cost_function0(d_ctx, string, vsmprop) = float is semidet.

apply_cost_function0(_, "f1", _) = 1.0.
