:- module ctx_impl.

:- interface.

:- import_module set.
:- import_module formula.
:- import_module context.
:- import_module ctx_modality.

:- type ctx.
:- instance context(ctx, ctx_modality).

:- func new_ctx = ctx.

:- pred add_fact(vscope(mprop(ctx_modality))::in, ctx::in, ctx::out) is det.
:- pred add_rule(vscope(mrule(ctx_modality))::in, ctx::in, ctx::out) is det.

	% for debugging purposes only!
:- func facts(ctx) = set(vscope(mprop(ctx_modality))).
:- func rules(ctx) = set(vscope(mrule(ctx_modality))).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- import_module abduction.

:- type d_ctx
	--->	d_ctx(
		d_ctx :: ctx,
		d_focus :: set(string)
	).

:- func new_d_ctx = d_ctx.

:- type ctx_change == pred(d_ctx, d_ctx).
:- inst ctx_change == (pred(in, out) is det).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred effect(vscope(mprop(M))) `with_type` ctx_change.
:- mode effect(in) `with_inst` ctx_change.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred dialogue_turn(proof(M)::in, d_ctx::in, d_ctx::out) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module list, pair.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type ctx
	--->	ctx(
		ctx_facts :: set(vscope(mprop(ctx_modality))),
		ctx_rules :: set(vscope(mrule(ctx_modality)))  % this doesn't really belong here, does it?
	).

:- instance context(ctx, ctx_modality) where [
	pred(fact/2) is ctx_fact,
	pred(vrule/2) is ctx_rule,
	pred(assumable/2) is ctx_assumable
].

new_ctx = ctx(set.init, set.init).

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

:- pred ctx_fact(ctx::in, vscope(mprop(ctx_modality))::out) is nondet.

ctx_fact(Ctx, Fact) :-
	set.member(Fact, Ctx^ctx_facts).

:- pred ctx_rule(ctx::in, vscope(mrule(ctx_modality))::out) is nondet.

ctx_rule(Ctx, Rule) :-
	set.member(Rule, Ctx^ctx_rules).

:- pred ctx_assumable(ctx::in, vscope(mprop(ctx_modality))::out) is nondet.

ctx_assumable(_Ctx, _) :-
	fail.

%------------------------------------------------------------------------------%

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

