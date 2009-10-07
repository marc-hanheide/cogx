% $Id$

:- module context.

:- interface.

:- import_module formula, modality.
:- import_module costs.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- typeclass context(C, M) <= modality(M) where [

	pred fact_found(C, vscope(mprop(M)), vscope(mprop(M))),
	mode fact_found(in, in, out) is nondet,

%	pred fact(C, vscope(mprop(M))),
%	mode fact(in, out) is nondet,

	pred rule_found(C, vscope(mrule(M))),
	mode rule_found(in, out) is nondet,

	pred assumable_func(C, cost_function_name, mgprop(M), float),
	mode assumable_func(in, in, out, out) is nondet,

	func min_assumption_cost(C, M) = float
].

:- pred assumable(C::in, vscope(mprop(M))::in, cost_function::in, vscope(mprop(M))::out, float::out) is nondet
		<= (context(C, M), modality(M)).

:- func cost(C, cost_function, vscope(mprop(M))) = float <= (context(C, M), modality(M)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type context_change(C) == pred(C, C).
:- inst context_change == (pred(in, out) is det).

:- pred effect(mgprop(M)) `with_type` context_change(C) <= (context(C, M), modality(M)).
:- mode effect(in) `with_inst` context_change.

:- type geffect(C, M) == pred(mgprop(M), C, C).
:- inst geffect == (pred(in, in, out) is det).

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require, solutions.
:- import_module set, list, pair, string.
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

%------------------------------------------------------------------------------%

effect(MGProp, !Ctx) :-
	(if MGProp = m(_, p("in_focus", [t(_Arg, [])]))
		%member(att(next), Ctx)
	then true
		%add_to_focus(Arg, !DC)
	else true
	).
