% $Id$

:- module formula.

:- interface.

:- import_module term, varset, pair, list, map, string, bool.
:- import_module context, costs.

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
	--->	vs(T, varset).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type modalized(M, T)
	--->	m(M, T).

:- type with_cost_function(T)
	--->	cf(T, cost_function).

:- type mprop == modalized(list(ctx_ref), atomic_formula).
:- type vsmprop == vscope(mprop).

:- type mrule == pair(bool, modalized(list(ctx_ref), pair(list(with_cost_function(mprop)), mprop))).
:- type vsmrule == vscope(mrule).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred string_as_vsmprop(string, vscope(mprop)).
:- mode string_as_vsmprop(in, out) is semidet.
:- mode string_as_vsmprop(out, in) is det.

:- func det_string_to_vsmprop(string) = vscope(mprop).

:- func mprop_to_string(varset, mprop) = string.
:- func vsmprop_to_string(vscope(mprop)) = string.

:- func atomic_formula_to_string(varset, atomic_formula) = string.
:- func formula_term_to_string(varset, formula.term) = string.


:- pred string_as_vsmrule(string, vscope(mrule)).
:- mode string_as_vsmrule(in, out) is semidet.
:- mode string_as_vsmrule(out, in) is det.

:- func det_string_to_vsmrule(string) = vscope(mrule).

:- func vsmrule_to_string(vscope(mrule)) = string.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type subst == map(var, formula.term).

:- func apply_subst_to_formula(subst, atomic_formula) = atomic_formula.
:- func apply_subst_to_mprop(subst, mprop) = mprop.

:- func rename_vars_in_term(map(var, var), formula.term) = formula.term.
:- func rename_vars_in_formula(map(var, var), atomic_formula) = atomic_formula.
:- func rename_vars_in_mprop(map(var, var), mprop) = mprop.
:- func rename_vars_in_annot_mprop(map(var, var), with_cost_function(mprop)) = with_cost_function(mprop).
:- func rename_vars_in_mrule(map(var, var), mrule) = mrule.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred unify_formulas(atomic_formula::in, atomic_formula::in, subst::out) is semidet.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred is_ground(atomic_formula::in) is semidet.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require.
:- import_module int.
:- import_module term_io, parser.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma promise_equivalent_clauses(string_as_vsmprop/2).

string_as_vsmprop(Str::in, vs(P, Varset)::out) :-
	read_term_from_string("", Str, _, term(Varset, T)),
	generic_term(T),
	term_to_mprop(T, P).

string_as_vsmprop(Str::out, vs(m(K, P), Varset)::in) :-
	Str = modality_to_string(K) ++ atomic_formula_to_string(Varset, P).

det_string_to_vsmprop(S) = P :-
	(if string_as_vsmprop(S, P0)
	then P = P0
	else error("Can't convert string \"" ++ S ++ "\" to a proposition.")
	).

mprop_to_string(Varset, MP) = Str :-
	string_as_vsmprop(Str, vs(MP, Varset)).

vsmprop_to_string(vs(MP, Varset)) = Str :-
	string_as_vsmprop(Str, vs(MP, Varset)).

:- func annot_vsmprop_to_string(vscope(with_cost_function(mprop))) = string.

annot_vsmprop_to_string(vs(cf(MP, F), Varset)) = Str ++ "/" ++ cost_function_to_string(F) :-
	string_as_vsmprop(Str, vs(MP, Varset)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma promise_equivalent_clauses(string_as_vsmrule/2).

string_as_vsmrule(Str::in, vs(R, Varset)::out) :-
	read_term_from_string("", Str, _, term(Varset, T)),
	generic_term(T),
	term_to_mrule(T, R).

string_as_vsmrule(Str::out, vs(Ax-m(K, As-H), Varset)::in) :-
	(if Ax = yes then axiom(AxStr0), AxStr = AxStr0 ++ " : " else AxStr = ""),
	ModStr = AxStr ++ modality_to_string(K),
	RuleStr = string.join_list(", ", list.map((func(A) = annot_vsmprop_to_string(vs(A, Varset))), As))
			++ " -> " ++ vsmprop_to_string(vs(H, Varset)),
	(if ModStr = ""
	then Rest = RuleStr
	else Rest = "(" ++ RuleStr ++ ")"
	),
	Str = ModStr ++ Rest.

det_string_to_vsmrule(S) = R :-
	(if string_as_vsmrule(S, R0)
	then R = R0
	else error("Can't convert string \"" ++ S ++ "\" to a rule.")
	).

vsmrule_to_string(vs(MR, Varset)) = Str :-
	string_as_vsmrule(Str, vs(MR, Varset)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred term_to_mprop(term.term::in, mprop::out) is semidet.

term_to_mprop(T, m(Mod, P)) :-
	(if T = functor(atom(":"), [TM, TP], _)
	then 
		term_to_list_of_ctx_refs(TM, Mod),
		term_to_atomic_formula(TP, P)
	else
		Mod = [],
		term_to_atomic_formula(T, P)
	).

:- pred term_to_mrule(term.term::in, mrule::out) is semidet.

term_to_mrule(T, Ax-m(Mod, R)) :-
	axiom(AxStr),
	(if T = functor(atom(":"), [TM, TR], _)
	then 
		(if
			TM = functor(atom(AxStr), [], _)
		then
			Ax = yes,
			Mod = []
		else
			(if
				TM = functor(atom(":"), [functor(atom(AxStr), [], _), TM1], _)
			then
				Ax = yes,
				term_to_list_of_ctx_refs(TM1, Mod)
			else
				Ax = no,
				term_to_list_of_ctx_refs(TM, Mod)
			)
		),
		term_to_nonmod_rule(TR, R)
	else
		Mod = [],
		Ax = no,
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

:- pred term_to_list_of_ctx_refs(term.term::in, list(ctx_ref)::out) is semidet.

term_to_list_of_ctx_refs(functor(atom(Ref), [], _), [CtxRef]) :-
	string_as_ctx_ref(Ref, CtxRef).
term_to_list_of_ctx_refs(functor(atom(":"), [Ms, M], _), LMs ++ LM) :-
	term_to_list_of_ctx_refs(Ms, LMs),
	term_to_list_of_ctx_refs(M, LM).

:- func modality_to_string(list(ctx_ref)) = string.

modality_to_string([]) = "".
modality_to_string([H|T]) = string.join_list(" : ", list.map((func(Ref) = S is det :-
		string_as_ctx_ref(S, Ref)), [H|T])) ++ " : ".

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred term_to_nonmod_rule(term.term::in, pair(list(with_cost_function(mprop)), mprop)::out) is semidet.

term_to_nonmod_rule(functor(atom("->"), [TAnte, THead], _), Ante-Head) :-
	term_to_list_of_annot_mprops(TAnte, Ante),
	term_to_mprop(THead, Head).

:- pred term_to_list_of_annot_mprops(term.term::in, list(with_cost_function(mprop))::out) is semidet.

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

rename_vars_in_mrule(Renaming, Ax-m(M, Ante-Succ)) =
		Ax-m(M, list.map(rename_vars_in_annot_mprop(Renaming), Ante)-rename_vars_in_mprop(Renaming, Succ)).

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
