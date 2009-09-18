:- module lf_io.

:- interface.

:- import_module lf.
:- import_module string, formula.

:- func ground_term_to_id(ground_term::in) = (id(string, string)::out) is semidet.
:- func ground_term_to_lf(ground_term::in) = (lf::out) is semidet.

:- func ground_atomic_formula_to_lf(ground_atomic_formula::in) = (lf::out) is semidet.
:- func det_ground_atomic_formula_to_lf(ground_atomic_formula) = lf.

:- func lf_to_string(lf) = string.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module require.
:- import_module list.

%------------------------------------------------------------------------------%

ground_term_to_id(t("::", [t(Name, []), t(Sort, [])])) = of_sort(Name, Sort).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

ground_term_to_lf(t("@", [NameSortTerm, LFTerm])) =
		at(ground_term_to_id(NameSortTerm), ground_term_to_lf(LFTerm)).

ground_term_to_lf(T) = i(ground_term_to_id(T)) :-
	T = t("::", [_,_]).

ground_term_to_lf(t(RelName, [LFTerm])) = r(RelName, ground_term_to_lf(LFTerm)).

ground_term_to_lf(t(Prop, [])) = p(Prop).

ground_term_to_lf(t("^", [LFA, LFB])) = and(ground_term_to_lf(LFA), ground_term_to_lf(LFB)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

ground_atomic_formula_to_lf(p("@", Args)) = ground_term_to_lf(t("@", Args)).

det_ground_atomic_formula_to_lf(GF) = LF :-
	(if ground_atomic_formula_to_lf(GF) = LF0
	then LF = LF0
	else error("error in det_ground_atomic_formula_to_lf/1")
	).

%------------------------------------------------------------------------------%

:- func id_to_string(id(string, string)) = string.

id_to_string(of_sort(I, S)) = I ++ ":" ++ S.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

lf_to_string(at(Id, LF)) = "@" ++ id_to_string(Id) ++ "(" ++ lf_to_string(LF) ++ ")".
lf_to_string(i(Id)) = id_to_string(Id).
lf_to_string(r(Rel, LF)) = "<" ++ Rel ++ ">" ++ "(" ++ lf_to_string(LF) ++ ")".
lf_to_string(p(Prop)) = Prop.
lf_to_string(and(LF1, LF2)) = lf_to_string(LF1) ++ " ^ " ++ lf_to_string(LF2).
