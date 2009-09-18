% $Id$

:- module context.

:- interface.

:- import_module formula, modality.
:- import_module costs.

%:- func apply_cost_function(d_ctx, string, vscope(mprop(M))) = float.
%:- func apply_cost_function(ctx(M), cost_function, vscope(mprop(M))) = float <= modality(M).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- typeclass context(C, M) <= modality(M) where [
	pred fact(C, vscope(mprop(M))),
	mode fact(in, out) is nondet,

		% XXX find a better name
	pred vrule(C, vscope(mrule(M))),
	mode vrule(in, out) is nondet,

	pred assumable_func(C, cost_function_name, mgprop(M), float),
	mode assumable_func(in, in, out, out) is nondet,

	func min_assumption_cost(C, M) = float
].

:- pred assumable(C::in, vscope(mprop(M))::in, cost_function::in, vscope(mprop(M))::out, float::out) is nondet
		<= (context(C, M), modality(M)).

:- func cost(C, cost_function, vscope(mprop(M))) = float <= (context(C, M), modality(M)).

%:- func apply_cost_function(C, cost_function, vscope(mprop(M))) = float <= (modality(M), context(C, M)).

%------------------------------------------------------------------------------%

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require, solutions.
:- import_module set, pair, string.
:- import_module formula_io.
:- import_module varset.

%------------------------------------------------------------------------------%
%------------------------------------------------------------------------------%
%------------------------------------------------------------------------------%

assumable(C, vs(m(Mod, PropIn), VS), f(FuncName), vs(m(Mod, Prop), VS), Cost) :-
	assumable_func(C, FuncName, m(Mod, GroundProp), Cost),
	ground_formula(Prop, GroundProp),
	unify_formulas(PropIn, Prop, _).  % XXX this?

assumable(_C, Prop, const(Cost), Prop, Cost).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

cost(C, F, Prop) = Cost :-
	solutions_set((pred(SolCost::out) is nondet :-
		assumable(C, Prop, F, Prop, SolCost)  % XXX look at the Prop, Prop here
			), Costs),
	(if singleton_set(Costs, Cost0)
	then Cost = Cost0
	else error("error in cost/3: prop=" ++ string(Prop) ++ ", length=" ++ string.from_int(set.count(Costs)))
	).

%apply_cost_function(C, CostFunction, VSMProp) = Cost :-
%	(if Cost0 = apply_cost_function0(C, CostFunction, VSMProp)
%	then Cost = Cost0
%	else error("can't find cost function " ++ string(CostFunction))
%	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

%:- func apply_cost_function0(C, cost_function, vscope(mprop(M))) = float is semidet
%		<= (modality(M), context(C, M)).

%apply_cost_function0(_, const(Cost), _) = Cost.
%apply_cost_function0(_, f("f1"), _) = 1.0.
