% $Id$

:- module formulae.

:- interface.

:- import_module term, varset, pair, list, map, string.
:- import_module context.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type atomic_formula
	--->	f(
		string,  % functor
		list(atomic_arg)
	).

:- type atomic_arg
	--->	a(string)  % atom
	;	v(var)
	.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type vscope(T)
	--->	vs(T, varset).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type modalized(M, T)
	--->	m(M, T).

:- type mprop == modalized(list(ctx_ref), atomic_formula).
:- type vsmprop == vscope(mprop).

:- type mrule == modalized(list(ctx_ref), pair(list(mprop), mprop)).
:- type vsmrule == vscope(mrule).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred string_as_vsmprop(string, vscope(mprop)).
:- mode string_as_vsmprop(in, out) is semidet.
:- mode string_as_vsmprop(out, in) is det.

:- func det_string_to_vsmprop(string) = vscope(mprop).

:- func mprop_to_string(varset, mprop) = string.
:- func vsmprop_to_string(vscope(mprop)) = string.

:- func atomic_formula_to_string(varset, atomic_formula) = string.
:- func atomic_arg_to_string(varset, atomic_arg) = string.


:- pred string_as_vsmrule(string, vscope(mrule)).
:- mode string_as_vsmrule(in, out) is semidet.
:- mode string_as_vsmrule(out, in) is det.

:- func det_string_to_vsmrule(string) = vscope(mrule).

:- func vsmrule_to_string(vscope(mrule)) = string.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type subst == map(var, atomic_arg).

:- func apply_subst_to_formula(subst, atomic_formula) = atomic_formula.
:- func apply_subst_to_mprop(subst, mprop) = mprop.

:- func rename_vars_in_formula(map(var, var), atomic_formula) = atomic_formula.
:- func rename_vars_in_mprop(map(var, var), mprop) = mprop.
:- func rename_vars_in_mrule(map(var, var), mrule) = mrule.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred unify_formulas(atomic_formula::in, atomic_formula::in, subst::out) is semidet.

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

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pragma promise_equivalent_clauses(string_as_vsmrule/2).

string_as_vsmrule(Str::in, vs(R, Varset)::out) :-
	read_term_from_string("", Str, _, term(Varset, T)),
	generic_term(T),
	term_to_mrule(T, R).

string_as_vsmrule(Str::out, vs(m(K, As-H), Varset)::in) :-
	ModStr = modality_to_string(K),
	RuleStr = string.join_list(", ", list.map((func(A) = vsmprop_to_string(vs(A, Varset))), As))
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

:- pred term_to_mprop(term::in, mprop::out) is semidet.

term_to_mprop(T, m(Mod, P)) :-
	(if T = functor(atom(":"), [TM, TP], _)
	then 
		term_to_list_of_ctx_refs(TM, Mod),
		term_to_atomic_formula(TP, P)
	else
		Mod = [],
		term_to_atomic_formula(T, P)
	).

:- pred term_to_mrule(term::in, mrule::out) is semidet.

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

:- pred term_to_atomic_formula(term::in, atomic_formula::out) is semidet.

term_to_atomic_formula(functor(atom(Functor), TermArgs, _), f(Functor, Args)) :-
	list.map((pred(TermArg::in, Arg::out) is semidet :-
		( TermArg = functor(atom(Atom), [], _), Arg = a(Atom)
		; TermArg = variable(Var, _), Arg = v(Var)
		)), TermArgs, Args).

atomic_formula_to_string(Varset, f(Functor, Args)) = Functor ++ Rest :-
	(
		Args = [],
		Rest = ""
	;
		Args = [_|_],
		Rest = "(" ++ string.join_list(", ", list.map(atomic_arg_to_string(Varset), Args)) ++ ")"
	).

atomic_arg_to_string(Varset, Arg) = S :-
	(
		Arg = a(Atom),
		S = Atom
	;
		Arg = v(Var),
		S = varset.lookup_name(Varset, Var)
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred term_to_list_of_ctx_refs(term::in, list(ctx_ref)::out) is semidet.

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

:- pred term_to_nonmod_rule(term::in, pair(list(mprop), mprop)::out) is semidet.

term_to_nonmod_rule(functor(atom("->"), [TAnte, THead], _), Ante-Head) :-
	term_to_list_of_mprops(TAnte, Ante),
	term_to_mprop(THead, Head).

:- pred term_to_list_of_mprops(term::in, list(mprop)::out) is semidet.

term_to_list_of_mprops(T, List) :-
	(if
		T = functor(atom(","), [TMP, TMPs], _)
	then
		term_to_mprop(TMP, MP),
		term_to_list_of_mprops(TMPs, MPs),
		List = [MP|MPs]
	else
		term_to_mprop(T, MP),
		List = [MP]
	).

%------------------------------------------------------------------------------%

apply_subst_to_mprop(Subst, m(M, Prop)) = m(M, apply_subst_to_formula(Subst, Prop)).

apply_subst_to_formula(Subst, f(F, Args)) = f(F, SubstArgs) :-
	SubstArgs0 = list.map((func(Arg) = SubstArg :-
		(
			Arg = a(Atom),
			SubstArg = a(Atom)
		;
			Arg = v(Var),
			(if Value = Subst^elem(Var)
			then SubstArg = Value
			else SubstArg = Arg
			)
		)), Args),

	(if SubstArgs0 = Args
	then SubstArgs = SubstArgs0
	else f(_, SubstArgs) = apply_subst_to_formula(Subst, f(F, SubstArgs0))
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

rename_vars_in_formula(Renaming, f(F, Args)) = f(F, SubstArgs) :-
	SubstArgs = list.map((func(Arg) = SubstArg :-
		(
			Arg = a(Atom),
			SubstArg = a(Atom)
		;
			Arg = v(Var),
			SubstArg = v(map.lookup(Renaming, Var))
		)), Args).

rename_vars_in_mprop(Renaming, m(M, Prop)) = m(M, rename_vars_in_formula(Renaming, Prop)).

rename_vars_in_mrule(Renaming, m(M, Ante-Succ)) =
		m(M, list.map(rename_vars_in_mprop(Renaming), Ante)-rename_vars_in_mprop(Renaming, Succ)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

unify_formulas(A, B, U) :-
	atomic_formula_to_term(A, TermA),
	atomic_formula_to_term(B, TermB),

	unify_term(TermA, TermB, init, TermU),
	map.map_values((pred(_::in, TermTgt::in, Tgt::out) is det :-
		(if
			( TermTgt = functor(atom(Atom), [], _), Tgt0 = a(Atom)
			; TermTgt = variable(Var, _), Tgt0 = v(Var)
			)
		then
			Tgt = Tgt0
		else
			error("error in unify_formulas")
		)), TermU, U).

:- pred atomic_formula_to_term(atomic_formula::in, term::out) is det.

atomic_formula_to_term(f(Functor, Args), functor(atom(Functor), TermArgs, context("", 0))) :-
	list.map((pred(Arg::in, TermArg::out) is det :-
		( Arg = a(Atom), TermArg = functor(atom(Atom), [], context("", 0))
		; Arg = v(Var), TermArg = variable(Var, context("", 0))
			)), Args, TermArgs).
