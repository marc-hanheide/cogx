% $Id$

:- module context.

:- interface.

:- import_module set, string.
:- import_module formula, abduction, modality.
:- import_module costs.
:- import_module ctx_modality.

%:- func apply_cost_function(d_ctx, string, vscope(mprop(M))) = float.
%:- func apply_cost_function(ctx(M), cost_function, vscope(mprop(M))) = float <= modality(M).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- typeclass context(T, M) <= modality(M) where [
	pred fact(T, vscope(mprop(M))),
	mode fact(in, out) is nondet,

		% XXX find a better name
	pred vrule(T, vscope(mrule(M))),
	mode vrule(in, out) is nondet,

	pred assumable(T, vscope(mprop(M))),
	mode assumable(in, out) is nondet
].

:- func apply_cost_function(C, cost_function, vscope(mprop(M))) = float <= (modality(M), context(C, M)).

%------------------------------------------------------------------------------%

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module set, list, pair.

%------------------------------------------------------------------------------%
%------------------------------------------------------------------------------%
%------------------------------------------------------------------------------%

apply_cost_function(C, CostFunction, VSMProp) = Cost :-
	(if Cost0 = apply_cost_function0(C, CostFunction, VSMProp)
	then Cost = Cost0
	else error("can't find cost function " ++ string(CostFunction))
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func apply_cost_function0(C, cost_function, vscope(mprop(M))) = float is semidet
		<= (modality(M), context(C, M)).

apply_cost_function0(_, const(Cost), _) = Cost.
apply_cost_function0(_, f("f1"), _) = 1.0.
