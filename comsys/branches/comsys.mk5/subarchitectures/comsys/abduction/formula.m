% $Id$

:- module formula.

:- interface.

:- import_module term, varset, pair, list, map, string.
:- import_module costs, ctx_modality.
:- import_module stringable, modality.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type atomic_formula
	--->	p(
		string,  % predicate_symbol
		list(formula.term)
	).

:- type term
	--->	v(var)
	;	t(string, list(formula.term))
	.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type vscope(T)
	--->	vs(
		body :: T,
		vars :: varset
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type modalized(M, T)
	--->	m(
		m :: M,
		p :: T
	).

:- type with_cost_function(T)
	--->	cf(T, cost_function).

:- type mprop(M) == modalized(list(M), atomic_formula).
:- type mrule(M) == modalized(list(M), pair(list(with_cost_function(mprop(M))), mprop(M))).

%:- type mprop == mprop(list(ctx_modality)).
%:- type vsmprop == vscope(mprop).

%:- type mrule == mrule(list(ctx_modality)).
%:- type vsmrule == vscope(mrule).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

%:- pred string_as_vsmprop(string, vscope(mprop(M))) <= (modality(M), stringable(M), parsable(M)).
%:- mode string_as_vsmprop(in, out) is semidet.
%:- mode string_as_vsmprop(out, in) is det.

	% parsing
:- func string_to_vsmprop(string) = vscope(mprop(M)) is semidet <= (modality(M), parsable(M)).
:- func det_string_to_vsmprop(string) = vscope(mprop(M)) <= (modality(M), parsable(M)).

	% generation
:- func mprop_to_string(varset, mprop(M)) = string <= (modality(M), stringable(M)).
:- func vsmprop_to_string(vscope(mprop(M))) = string <= (modality(M), stringable(M)).

:- func atomic_formula_to_string(varset, atomic_formula) = string.
:- func formula_term_to_string(varset, formula.term) = string.


%:- pred string_as_vsmrule(string, vscope(mrule(M))) <= (modality(M), stringable(M), parsable(M)).
%:- mode string_as_vsmrule(in, out) is semidet.
%:- mode string_as_vsmrule(out, in) is det.

:- func string_to_vsmrule(string) = vscope(mrule(M)) is semidet <= (modality(M), parsable(M)).
:- func det_string_to_vsmrule(string) = vscope(mrule(M)) <= (modality(M), parsable(M)).

:- func vsmrule_to_string(vscope(mrule(M))) = string <= (modality(M), stringable(M)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred term_to_mprop(term.term::in, mprop(M)::out) is semidet <= (modality(M), parsable(M)).
:- pred term_to_mrule(term.term::in, mrule(M)::out) is semidet <= (modality(M), parsable(M)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type subst == map(var, formula.term).

:- func apply_subst_to_formula(subst, atomic_formula) = atomic_formula.
:- func apply_subst_to_mprop(subst, mprop(M)) = mprop(M) <= modality(M).

:- func rename_vars_in_term(map(var, var), formula.term) = formula.term.
:- func rename_vars_in_formula(map(var, var), atomic_formula) = atomic_formula.
:- func rename_vars_in_mprop(map(var, var), mprop(M)) = mprop(M) <= modality(M).
:- func rename_vars_in_annot_mprop(map(var, var), with_cost_function(mprop(M))) = with_cost_function(mprop(M))
		<= modality(M).
:- func rename_vars_in_mrule(map(var, var), mrule(M)) = mrule(M) <= modality(M).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred unify_formulas(atomic_formula::in, atomic_formula::in, subst::out) is semidet.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred is_ground(atomic_formula::in) is semidet.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module int.
:- import_module term_io, parser, formula_io.

:- import_module stringable.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

string_to_vsmprop(Str) = vs(P, Varset) :-
	read_term_from_string("", Str, _, term(Varset, T)),
	generic_term(T),
	term_to_mprop(T, P).

det_string_to_vsmprop(S) = P :-
	(if P0 = string_to_vsmprop(S)
	then P = P0
	else error("Can't convert string \"" ++ S ++ "\" to a proposition.")
	).

mprop_to_string(Varset, MP) = vsmprop_to_string(vs(MP, Varset)).

vsmprop_to_string(vs(m(K, P), Varset)) = Str :-
	Str = modality_to_string(K) ++ atomic_formula_to_string(Varset, P).

:- func annot_vsmprop_to_string(vscope(with_cost_function(mprop(M)))) = string
		<= (modality(M), stringable(M)).

annot_vsmprop_to_string(vs(cf(MP, F), Varset)) = vsmprop_to_string(vs(MP, Varset))
		++ "/" ++ cost_function_to_string(F).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

string_to_vsmrule(Str) = vs(R, Varset) :-
	read_term_from_string("", Str, _, term(Varset, T)),
	generic_term(T),
	term_to_mrule(T, R).

det_string_to_vsmrule(S) = R :-
	(if R0 = string_to_vsmrule(S)
	then R = R0
	else error("Can't convert string \"" ++ S ++ "\" to a rule.")
	).

vsmrule_to_string(vs(m(K, As-H), Varset)) = Str :-
	ModStr = modality_to_string(K),
	RuleStr = vsmprop_to_string(vs(H, Varset)) ++ " <- "
			++ string.join_list(", ", list.map((func(A) = annot_vsmprop_to_string(vs(A, Varset))), As)),
	(if ModStr = ""
	then Rest = RuleStr
	else Rest = "(" ++ RuleStr ++ ")"
	),
	Str = ModStr ++ Rest.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

term_to_mprop(T, m(Mod, P)) :-
	(if T = functor(atom(":"), [TM, TP], _)
	then 
		term_to_list_of_ctx_refs(TM, Mod),
		term_to_atomic_formula(TP, P)
	else
		Mod = [],
		term_to_atomic_formula(T, P)
	).

term_to_mrule(T, m(Mod, R)) :-
	(if T = functor(atom(":"), [TM, TR], _)
	then 
		term_to_list_of_ctx_refs(TM, Mod),
		term_to_nonmod_rule(TR, R)
	else
		Mod = [],
		term_to_nonmod_rule(T, R)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred term_to_atomic_formula(term.term::in, atomic_formula::out) is semidet.

term_to_atomic_formula(functor(atom(PredSym), TermArgs, _), p(PredSym, Args)) :-
	list.map(term_to_formula_term, TermArgs, Args).


:- pred term_to_formula_term(term.term::in, formula.term::out) is semidet.

term_to_formula_term(variable(Var, _), v(Var)).
term_to_formula_term(functor(atom(Functor), TermArgs, _), t(Functor, Args)) :-
	list.map(term_to_formula_term, TermArgs, Args).

atomic_formula_to_string(Varset, p(PredSym, Args)) = PredSym ++ "(" ++ ArgStr ++ ")" :-
	ArgStr = string.join_list(", ", list.map(formula_term_to_string(Varset), Args)).

formula_term_to_string(Varset, Arg) = S :-
	(
		Arg = t(Functor, []),
		S = Functor
	;
		Arg = t(Functor, [H|T]),
		S = Functor ++ "(" ++ string.join_list(", ", list.map(formula_term_to_string(Varset), [H|T])) ++ ")"
	;
		Arg = v(Var),
		S = varset.lookup_name(Varset, Var)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred term_to_list_of_ctx_refs(term.term::in, list(M)::out) is semidet <= (modality(M), parsable(M)).

term_to_list_of_ctx_refs(functor(atom(S), [], _), [from_string(S)]).
term_to_list_of_ctx_refs(functor(atom(":"), [Ms, M], _), LMs ++ LM) :-
	term_to_list_of_ctx_refs(Ms, LMs),
	term_to_list_of_ctx_refs(M, LM).

:- func modality_to_string(list(M)) = string <= (modality(M), stringable(M)).

modality_to_string([]) = "".
modality_to_string([H|T]) = string.join_list(":", list.map(to_string, [H|T])) ++ ":".

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred term_to_nonmod_rule(term.term::in, pair(list(with_cost_function(mprop(M))), mprop(M))::out) is semidet
		<= (modality(M), parsable(M)).

term_to_nonmod_rule(functor(atom("<-"), [THead, TAnte], _), Ante-Head) :-
	term_to_mprop(THead, Head),
	term_to_list_of_annot_mprops(TAnte, Ante).

:- pred term_to_list_of_annot_mprops(term.term::in, list(with_cost_function(mprop(M)))::out) is semidet
		<= (modality(M), parsable(M)).

term_to_list_of_annot_mprops(T, List) :-
	(if
		T = functor(atom(","), [TMP, TMPs], _)
	then
		func_annotation(TMP, F, MP1),
		term_to_mprop(MP1, MP),
		term_to_list_of_annot_mprops(TMPs, MPs),
		List = [cf(MP, F)|MPs]
	else
		func_annotation(T, F, T1),
		term_to_mprop(T1, MP),
		List = [cf(MP, F)]
	).

:- pred func_annotation(term.term::in, cost_function::out, term.term::out) is semidet.

func_annotation(functor(atom("/"), [T, functor(atom(FName), [], _)], _), f(FName), T).
func_annotation(functor(atom("/"), [T, functor(float(FValue), [], _)], _), const(FValue), T).

%------------------------------------------------------------------------------%

apply_subst_to_mprop(Subst, m(M, Prop)) = m(M, apply_subst_to_formula(Subst, Prop)).

apply_subst_to_formula(Subst, p(F, Args)) = p(F, SubstArgs) :-
	SubstArgs0 = list.map((func(Arg) = SubstArg :-
		(
			Arg = t(Functor, TermArgs),
			SubstArg = t(Functor, TermArgs)
		;
			Arg = v(Var),
			(if Value = Subst^elem(Var)
			then SubstArg = Value
			else SubstArg = Arg
			)
		)), Args),

	(if SubstArgs0 = Args
	then SubstArgs = SubstArgs0
	else p(_, SubstArgs) = apply_subst_to_formula(Subst, p(F, SubstArgs0))
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

rename_vars_in_term(Renaming, v(Var)) = v(map.lookup(Renaming, Var)).
rename_vars_in_term(Renaming, t(F, Args)) = t(F, SubstArgs) :-
	SubstArgs = list.map(rename_vars_in_term(Renaming), Args).

rename_vars_in_formula(Renaming, p(PS, Args)) = p(PS, SubstArgs) :-
	SubstArgs = list.map(rename_vars_in_term(Renaming), Args).

rename_vars_in_mprop(Renaming, m(M, Prop)) = m(M, rename_vars_in_formula(Renaming, Prop)).

rename_vars_in_annot_mprop(Renaming, cf(MProp, F)) = cf(rename_vars_in_mprop(Renaming, MProp), F).

rename_vars_in_mrule(Renaming, m(M, Ante-Succ)) =
		m(M, list.map(rename_vars_in_annot_mprop(Renaming), Ante)-rename_vars_in_mprop(Renaming, Succ)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

unify_formulas(A, B, U) :-
	atomic_formula_to_term(A, TermA),
	atomic_formula_to_term(B, TermB),

	unify_term(TermA, TermB, init, TermU),
	map.map_values((pred(_::in, TermTgt::in, Tgt::out) is det :-
		(if term_to_formula_term(TermTgt, Tgt0)
		then Tgt = Tgt0
		else error("error un unify_formulas")
		)), TermU, U).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred formula_term_to_term(formula.term::in, term.term::out) is det.

formula_term_to_term(v(Var), variable(Var, context("", 0))).
formula_term_to_term(t(F, Args), functor(atom(F), TermArgs, context("", 0))) :-
	list.map(formula_term_to_term, Args, TermArgs).

:- pred atomic_formula_to_term(atomic_formula::in, term.term::out) is det.

atomic_formula_to_term(p(PredSym, Args), functor(atom(PredSym), TermArgs, context("", 0))) :-
	list.map(formula_term_to_term, Args, TermArgs).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred term_ground(formula.term::in) is semidet.

term_ground(t(_, Terms)) :-
	list.all_true(term_ground, Terms).

is_ground(p(_, Args)) :-
	list.all_true(term_ground, Args).
