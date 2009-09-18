:- module ctx_specific.

:- interface.

:- import_module set, map, pair.
:- import_module formula.
:- import_module context.
:- import_module ctx_modality.
:- import_module costs.

:- import_module belief_model, world_model.
:- import_module abduction, modality.
:- import_module ctx_ontology, enumerable.

:- import_module unit.

:- type ctx
	--->	ctx(
		ctx_rules :: set(vscope(mrule(ctx_modality))),  % this doesn't really belong here, does it?
		bm :: belief_model(string, string, string),
		rrel :: unit,
		ont :: ctx_ontology
	).
:- instance context(ctx, ctx_modality).

:- func new_ctx = ctx.

:- type assumable_function_def(M) == pair(cost_function_name, map(mgprop(M), float)).

:- pred add_rule(vscope(mrule(ctx_modality))::in, ctx::in, ctx::out) is det.

:- pred set_rules(set(vscope(mrule(ctx_modality)))::in, ctx::in, ctx::out) is det.

	% for debugging purposes only!
:- func rules(ctx) = set(vscope(mrule(ctx_modality))).

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module list, pair, map, float.
:- import_module costs, varset, lf, lf_io, ontology.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- instance context(ctx, ctx_modality) where [
	pred(fact_found/3) is ctx_fact,
	pred(rule_found/2) is ctx_rule,
	pred(assumable_func/4) is ctx_assumable_func,
	func(min_assumption_cost/2) is ctx_min_assumption_cost
].

new_ctx = ctx(set.init, belief_model.init, unit, ctx_ontology.init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

add_rule(Rule, Ctx0, Ctx) :-
	Rules = Ctx0^ctx_rules,
	Ctx = Ctx0^ctx_rules := set.insert(Rules, Rule).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

set_rules(Rules, Ctx0, Ctx) :-
	Ctx = Ctx0^ctx_rules := Rules.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

rules(Ctx) = Ctx^ctx_rules.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func ctx_min_assumption_cost(ctx, ctx_modality) = float.

ctx_min_assumption_cost(_, _) = 0.1.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred ctx_rule(ctx::in, vscope(mrule(ctx_modality))::out) is nondet.

ctx_rule(Ctx, Rule) :-
	set.member(Rule, Ctx^ctx_rules).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred ctx_fact(ctx::in, vscope(mprop(ctx_modality))::in, vscope(mprop(ctx_modality))::out) is nondet.

ctx_fact(Ctx, vs(m(Mod, _), _), VSMProp) :-
	compose_list(Mod) = [k(STF, Belief)],
	k_fact(Ctx^ont, Ctx^rrel, Ctx^bm, STF, Belief, LF),
	VSMProp = vs(m(Mod, ground_formula_to_formula(lf_to_ground_atomic_formula(LF))), varset.init).

ctx_fact(Ctx, vs(m(Mod, Prop), VS), vs(m(Mod, Prop), VS)) :-
	compose_list(Mod) = [k(STF, Belief)],
	k_model(Ctx^ont, Ctx^rrel, Ctx^bm, STF, Belief, M),
	satisfies(Ctx^ont, M, ground_atomic_formula_to_lf(formula_to_ground_formula(Prop))).

ctx_fact(Ctx, vs(m(Mod, p("<<", [_, _])), VS), vs(m(Mod, p("<<", [t(Sub, []), t(Super, [])])), VS)) :-
	generate(Ctx^ont, Sub),
	generate(Ctx^ont, Super).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- import_module ling.
:- import_module io, string, world_model_io.

:- pred ctx_assumable_func(ctx::in, cost_function_name::in, mgprop(ctx_modality)::out, float::out) is nondet.

ctx_assumable_func(Ctx, _, m(Mod, GProp), Cost) :-
	Mod = [a(com)],
	att_model(Ctx^ont, Ctx^rrel, Ctx^bm, AM),
	map.member(AM^worlds, WName, Sort),
	(
		map.search(AM^props, i(WName), Props),
		set.member(Prop, Props),
		noun_gender(Prop, neut)
	;
		noun_gender(Sort, neut)
	),
	Dist = min_dist_from_set(Ctx^ont, AM, fg_anchors(Ctx^bm), WName),
	Cost = 0.5 + float(Dist),
	GProp = p("it", [t(WName, [])]).

%ctx_assumable_func(_Ctx, _FuncName, _GProp, _Cost) :-
%	fail.
