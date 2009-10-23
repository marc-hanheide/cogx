:- module ctx_loadable.

:- interface.

:- import_module set, map, pair.
:- import_module formula.
:- import_module context.
:- import_module ctx_modality.
:- import_module costs.

:- type ctx.
:- instance context(ctx, ctx_modality).

:- func new_ctx = ctx.

:- type assumable_function_def(M) == pair(cost_function_name, map(mgprop(M), float)).

:- pred add_fact(vscope(mprop(ctx_modality))::in, ctx::in, ctx::out) is det.
:- pred add_rule(vscope(mrule(ctx_modality))::in, ctx::in, ctx::out) is det.
:- pred add_assumable(assumable_function_def(ctx_modality)::in, ctx::in, ctx::out) is det.

:- pred set_facts(set(vscope(mprop(ctx_modality)))::in, ctx::in, ctx::out) is det.
:- pred set_rules(set(vscope(mrule(ctx_modality)))::in, ctx::in, ctx::out) is det.
:- pred set_assumables(map(cost_function_name, map(mgprop(ctx_modality), float))::in, ctx::in, ctx::out)
		is det.

	% for debugging purposes only!
:- func facts(ctx) = set(vscope(mprop(ctx_modality))).
:- func rules(ctx) = set(vscope(mrule(ctx_modality))).
:- func assumables(ctx) = map(cost_function_name, map(mgprop(ctx_modality), float)).

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module list, pair, map, float.
:- import_module costs.
:- import_module varset.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type ctx
	--->	ctx(
		ctx_facts :: set(vscope(mprop(ctx_modality))),
		ctx_rules :: set(vscope(mrule(ctx_modality))),  % this doesn't really belong here, does it?
		ctx_assumables :: map(cost_function_name, map(mgprop(ctx_modality), float))
	).

:- instance context(ctx, ctx_modality) where [
	pred(find_fact/4) is find_ctx_fact,
	pred(find_rule/4) is find_ctx_rule,
	pred(fact_found/3) is ctx_fact,
	pred(rule_found/3) is ctx_rule,
	pred(assumable_func/4) is ctx_assumable_func,
	func(min_assumption_cost/2) is ctx_min_assumption_cost
].

new_ctx = ctx(set.init, set.init, map.init).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

add_fact(Prop, Ctx0, Ctx) :-
	Facts = Ctx0^ctx_facts,
	Ctx = Ctx0^ctx_facts := set.insert(Facts, Prop).

add_rule(Rule, Ctx0, Ctx) :-
	Rules = Ctx0^ctx_rules,
	Ctx = Ctx0^ctx_rules := set.insert(Rules, Rule).

add_assumable(FuncName-Costs, Ctx0, Ctx) :-
	AssumFuncs = Ctx0^ctx_assumables,
	Ctx = Ctx0^ctx_assumables := map.set(AssumFuncs, FuncName, Costs).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

set_facts(Facts, Ctx0, Ctx) :-
	Ctx = Ctx0^ctx_facts := Facts.

set_rules(Rules, Ctx0, Ctx) :-
	Ctx = Ctx0^ctx_rules := Rules.

set_assumables(Assumables, Ctx0, Ctx) :-
	Ctx = Ctx0^ctx_assumables := Assumables.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

facts(Ctx) = Ctx^ctx_facts.

rules(Ctx) = Ctx^ctx_rules.

assumables(Ctx) = Ctx^ctx_assumables.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred find_ctx_fact(ctx::in, list(ctx_modality)::in, string::in, vscope(mprop(ctx_modality))::out) is nondet.

find_ctx_fact(Ctx, Ms, PredSym, vs(m(Ms, p(PredSym, Args)), VS)) :-
	set.member(vs(m(Ms, p(PredSym, Args)), VS), Ctx^ctx_facts).

find_ctx_fact(_Ctx, Ms, "=", vs(m(Ms, p("=", [v(V), v(V)])), VS)) :-
	new_named_var(varset.init, "X", V, VS).

:- pred find_ctx_rule(ctx::in, list(ctx_modality)::in, string::in, vscope(mrule(ctx_modality))::out) is nondet.

find_ctx_rule(Ctx, Ms, PredSym, VSMRule) :-
	set.member(VSMRule, Ctx^ctx_rules).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- import_module io.

:- pred ctx_fact(ctx::in, vscope(mprop(ctx_modality))::in, vscope(mprop(ctx_modality))::out) is nondet.

ctx_fact(Ctx, vs(m(_, p(PredSym, _)), _), vs(m(Mod, p(PredSym, Args)), VS)) :-
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "F", !IO) ),
	set.member(vs(m(Mod, p(PredSym, Args)), VS), Ctx^ctx_facts).

ctx_fact(_Ctx, vs(m(Mod, p("=", [T01, T02])), VS), vs(m(Mod, p("=", [T1, T2])), VS)) :-
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "=", !IO) ),
	unify_terms(T01, T02, Subst),
	T1 = apply_subst_to_term(Subst, T01),
	T2 = apply_subst_to_term(Subst, T02).

/*
ctx_fact(_Ctx, vs(m(Mod, p("\\=", [T1, T2])), VS),
		vs(m(Mod, p("=", [T1, T2])), VS)) :-
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "!", !IO) ),
	(if
		unify_terms(T1, T2, _Subst)
	then
		% unifiable -> possibly equal
		fail
	else
		% not unifiable -> certainly not equal
		true
	).
*/

:- pred ctx_rule(ctx::in, vscope(mprop(ctx_modality))::in, vscope(mrule(ctx_modality))::out) is nondet.

ctx_rule(Ctx, vs(m(_, p(PredSym, _)), _), vs(m(ModR, Ante-Head), VS)) :-
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "R", !IO) ),
	set.member(vs(m(ModR, Ante-Head), VS), Ctx^ctx_rules),
	(
		Head = std(m(_, p(PredSym, _)))
	;
		Head = test(MTest),
		( MTest = prop(m(_, p(PredSym, _)))
		; MTest = impl(_, m(_, p(PredSym, _)))
		)
	).

:- pred ctx_assumable_func(ctx::in, cost_function_name::in, mgprop(ctx_modality)::out, float::out) is nondet.

ctx_assumable_func(Ctx, FuncName, GProp, Cost) :-
	map.search(Ctx^ctx_assumables, FuncName, MapCosts),
	trace[compile_time(flag("debug")), io(!IO)] ( print(stderr_stream, "A", !IO) ),
	map.member(MapCosts, GProp, Cost).

:- func ctx_min_assumption_cost(ctx, ctx_modality) = float.

ctx_min_assumption_cost(_, _) = 0.1.

%------------------------------------------------------------------------------%

/*
:- pred add_to_focus(string) `with_type` ctx_change.
:- mode add_to_focus(in) `with_inst` ctx_change.

add_to_focus(A, DC0, DC) :-
	F0 = DC0^d_focus,
	set.insert(F0, A, F),
	DC = DC0^d_focus := F.
*/
