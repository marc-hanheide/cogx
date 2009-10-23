% $Id$

:- module context.

:- interface.

:- import_module formula, modality.
:- import_module list.
:- import_module costs.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- typeclass context(C, M) <= modality(M) where [

	pred fact_found(C, vscope(mprop(M)), vscope(mprop(M))),
	mode fact_found(in, in, out) is nondet,

	pred find_fact(C, list(M), string, vscope(mprop(M))),
	mode find_fact(in, in, in, out) is nondet,

%	pred fact(C, vscope(mprop(M))),
%	mode fact(in, out) is nondet,

	pred rule_found(C, vscope(mprop(M)), vscope(mrule(M))),
	mode rule_found(in, in, out) is nondet,

	pred find_rule(C, list(M), string, vscope(mrule(M))),
	mode find_rule(in, in, in, out) is nondet,

	pred assumable_func(C, cost_function_name, mgprop(M), float),
	mode assumable_func(in, in, out, out) is nondet,

	func min_assumption_cost(C, M) = float
].

:- pred assumable(C::in, vscope(mprop(M))::in, cost_function::in, vscope(mprop(M))::out, float::out) is nondet
		<= (context(C, M), modality(M)).

:- func cost(C, cost_function, vscope(mprop(M))) = float <= (context(C, M), modality(M)).

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require, solutions.
:- import_module set, list, pair, string.
:- import_module varset.

:- import_module io.

%------------------------------------------------------------------------------%
%------------------------------------------------------------------------------%
%------------------------------------------------------------------------------%

assumable(C, vs(m(Mod, PropIn), VS), f(FuncName), vs(m(Mod, Prop), VS), Cost) :-
	assumable_func(C, FuncName, m(Mod, GroundProp), Cost),
	ground_formula(Prop, GroundProp),
	unify_formulas(PropIn, Prop, _).  % XXX this?

assumable(_C, Prop, const(Cost), Prop, Cost) :-
	trace[compile_time(flag("debug")), io(!IO)] ( format(stderr_stream, "/%f\\", [f(Cost)], !IO) ).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

cost(C, F, Prop) = Cost :-
	solutions_set((pred(SolCost::out) is nondet :-
		assumable(C, Prop, F, Prop, SolCost)  % XXX look at the Prop, Prop here
			), Costs),
	(if singleton_set(Costs, Cost0)
	then Cost = Cost0
	else error("error in cost/3: prop=" ++ string(Prop) ++ ", length=" ++ string.from_int(set.count(Costs)))
	).
