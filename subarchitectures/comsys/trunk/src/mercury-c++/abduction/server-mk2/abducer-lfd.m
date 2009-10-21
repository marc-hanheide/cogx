:- module 'abducer-lfd'.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require, solutions.
:- import_module map, set, list, pair, assoc_list, string, float, int, bag, bool, maybe.
:- import_module utils.
:- import_module abduction, formula, context, costs.
:- import_module loading.

:- import_module context, ctx_modality, ctx_loadable, ctx_io, ctx_loadable_io.
:- import_module modality, stringable.

:- import_module parser, term_io, term, varset, formula_io, formula_ops, costs.
:- import_module gc.
:- import_module protocol.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type srv_ctx
	--->	srv_ctx(
		cx :: ctx,
		best_proof :: maybe(goal(ctx_modality))
	).

main(!IO) :-
	inner_loop(srv_ctx(new_ctx, no), _, !IO).

:- pred inner_loop(srv_ctx::in, srv_ctx::out, io::di, io::uo) is det.

inner_loop(!Ctx, !IO) :-
	io.read_line_as_string(ReadResult, !IO),
	(
		ReadResult = ok(ReqStr),
		read_term_from_string("ICE request", ReqStr, _Pos, ReadTermResult),
		(
		 	ReadTermResult = term(_Varset, Term),
			(if
				term_to_type(Term, Request),
				is_request(Request)
			then
				print(stderr_stream, "[abd] got a valid request: " ++ ReqStr, !IO),
				process_request(Request, !Ctx, !IO),
				inner_loop(!Ctx, !IO)
			else
				print(stderr_stream, "failed to parse the request\n", !IO)
			)
		;
			ReadTermResult = error(_Err, _LineNum),
			print(stderr_stream, "got an error in read_term\n", !IO)
		;
			ReadTermResult = eof,
			print(stderr_stream, "end of file in read_term\n", !IO)
		)
	;
		ReadResult = error(_Err),
		print(stderr_stream, "error in read_line\n", !IO),
		true
	;
		ReadResult = eof,
		print(stderr_stream, "end of file in read_line\n", !IO),
		true
	).

:- pred process_request(protocol.request::in, srv_ctx::in, srv_ctx::out, io::di, io::uo) is det.

process_request(init_ctx, _, srv_ctx(new_ctx, no), !IO) :-
	print(stderr_stream, "[REQUEST] init_ctx\n", !IO).

process_request(load_file(Filename), !SCtx, !IO) :-
	print(stderr_stream, "[REQUEST] load_file\n", !IO),
	loading.load_file(Filename, _Result, !.SCtx^cx, NewCtx, !IO),
	!:SCtx = !.SCtx^cx := NewCtx,
	print("ok.\n", !IO),
	flush_output(!IO),
	print(stderr_stream, "[done] load_file\n", !IO).

process_request(clear_rules, !SCtx, !IO) :-
	print(stderr_stream, "[REQUEST] clear_rules\n", !IO),
	set_rules(set.init, !.SCtx^cx, NewCtx),
	!:SCtx = !.SCtx^cx := NewCtx,
	print(stderr_stream, "[done] clear_rules\n", !IO).

process_request(clear_facts, !SCtx, !IO) :-
	print(stderr_stream, "[REQUEST] clear_facts\n", !IO),
	set_facts(set.init, !.SCtx^cx, NewCtx),
	!:SCtx = !.SCtx^cx := NewCtx,
	print(stderr_stream, "[done] clear_rules\n", !IO).

process_request(clear_facts_by_modality(k), !SCtx, !IO) :-
	print(stderr_stream, "[REQUEST] clear_facts_by_modality(k)\n", !IO),
	set_facts(set.filter((pred(vs(m(Mod, _), _)::in) is semidet :-
		Mod \= [k(_, _)|_]
			), !.SCtx^cx^facts), !.SCtx^cx, NewCtx),
	!:SCtx = !.SCtx^cx := NewCtx,
	print(stderr_stream, "[done] clear_facts_by_modality(k)\n", !IO).

process_request(clear_assumables, !SCtx, !IO) :-
	print(stderr_stream, "[REQUEST] clear_assumables\n", !IO),
	set_assumables(map.init, !.SCtx^cx, NewCtx),
	!:SCtx = !.SCtx^cx := NewCtx,
	print(stderr_stream, "[done] clear_rules\n", !IO).

process_request(add_fact(FactStr), !SCtx, !IO) :-
	print(stderr_stream, "[REQUEST] add_fact\n", !IO),
	vs(MProp, VS) = det_string_to_vsmprop(FactStr),
	add_fact(vs(MProp, VS), !.SCtx^cx, NewCtx),
	!:SCtx = !.SCtx^cx := NewCtx,
	print(stderr_stream, "[done] add_fact\n", !IO).

process_request(add_assumable(Function, MPropStr, Cost), !SCtx, !IO) :-
	print(stderr_stream, "[REQUEST] add_assumable\n", !IO),
	vs(m(Mod, Prop), _VS) = det_string_to_vsmprop(MPropStr),
	Ass = !.SCtx^cx^assumables,
	(if map.search(Ass, Function, Mapping0)
	then Mapping = Mapping0
	else Mapping = map.init
	),
	map.set(Mapping, m(Mod, det_formula_to_ground_formula(Prop)), Cost, MappingNew),
	map.set(Ass, Function, MappingNew, Ass1),
	set_assumables(Ass1, !.SCtx^cx, NewCtx),
	!:SCtx = !.SCtx^cx := NewCtx,
	print(stderr_stream, "[done] add_assumable\n", !IO).

process_request(prove(L), !SCtx, !IO) :-
	print(stderr_stream, "[REQUEST] prove\n", !IO),
	list.map((pred(S::in, Q::out) is det :-
		vs(MProp, _VS) = det_string_to_vsmprop(S),
		Q = unsolved(MProp, not_assumable)
			), L, Qs),

	% TODO: merge all varsets
	%varset.merge_renaming(VS0, VSA, VS, Renaming),
	%PA = rename_vars_in_formula(Renaming, PA0),

	P0 = proof(vs([Qs], varset.init), []),
	is_ctx_proof(P0),

	print_ctx(stderr_stream, !.SCtx^cx, !IO),

	Proofs0 = set.to_sorted_list(solutions_set((pred((Cost-Gx)-P::out) is nondet :-
		prove(0.0, 100.0, P0, P, default_costs, !.SCtx^cx),
		Gx = last_goal(P),
		Cost = cost(!.SCtx^cx, P, default_costs)
			))),

	% examine derivations
	list.foldl((pred((Cost-Gy)-P::in, M0::in, M::out) is det :-
		(if map.search(M0, Cost-Gy, D0)
		then D1 = D0
		else D1 = set.init
		),
		set.insert(D1, P, D2),
		map.set(M0, Cost-Gy, D2, M)
			), Proofs0, map.init, DerivsMap),

	list.sort((pred((CA-_)-_::in, (CB-_)-_::in, Comp::out) is det :-
		float_compare(CA, CB, Comp)
			), map.to_assoc_list(DerivsMap), DerivsSorted),

	format(stderr_stream, "  %d proofs found.\n", [i(list.length(DerivsSorted))], !IO),

	list.foldl((pred((Cost-Gz)-Ds::in, !.IO::di, !:IO::uo) is det :-
		print(stderr_stream, "---------------------------------------------------------------------\n", !IO),
		format(stderr_stream, "proof cost = %f\n\n", [f(Cost)], !IO),
		print(stderr_stream, "proven goal:\n  " ++ goal_to_string(Gz) ++ "\n", !IO),
		nl(stderr_stream, !IO),

%		print(stderr_stream, "assumptions:\n", !IO),
%		print(stderr_stream, "  " ++ assumptions_to_string(!.Ctx, goal_assumptions(Gz)) ++ "\n", !IO),
%		nl(stderr_stream, !IO),

%		print(stderr_stream, "assertions:\n", !IO),
%		print(stderr_stream, "  " ++ assertions_to_string(!.Ctx, goal_assertions(Gz)) ++ "\n", !IO),
%		nl(stderr_stream, !IO),

		print(stderr_stream, string.from_int(set.count(Ds)) ++ " derivation(s).\n", !IO),

		print(stderr_stream, "- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n", !IO),

		set.fold((pred(Proof::in, !.IO::di, !:IO::uo) is det :-
			is_ctx_proof(Proof),
			print_proof_trace(stderr_stream, !.SCtx^cx, Proof, !IO),
			nl(stderr_stream, !IO)
				), Ds, !IO)

			), DerivsSorted, !IO),

	(if
		DerivsSorted = [(_Cost-G)-_Ds|_]
	then
		Response = "success",
		!:SCtx = !.SCtx^best_proof := yes(G)
	else
		Response = "failure",
		!:SCtx = !.SCtx^best_proof := no
	),

	print(Response ++ ".\n", !IO),
	flush_output(!IO),
	print(stderr_stream, "[done] prove\n", !IO).

process_request(get_best_proof, !SCtx, !IO) :-
	print(stderr_stream, "[REQUEST] get_best_proof\n", !IO),
	(
		!.SCtx^best_proof = yes(vs(Qs, VS)),
		format("%d\n", [i(list.length(Qs))], !IO),
		flush_output(!IO),
		list.foldl((pred(Q::in, !.IO::di, !:IO::uo) is det :-
			dissect_query(Q) = Marking-MProp,
			print(Marking ++ mprop_to_string(VS, MProp) ++ ".\n", !IO)
				), Qs, !IO),
		flush_output(!IO)
	;
		!.SCtx^best_proof = no,
		print("0\n", !IO),
		flush_output(!IO)
	),
	print(stderr_stream, "[done] get_best_proof\n", !IO).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func dissect_query(query(ctx_modality)) = pair(string, mprop(ctx_modality)).

dissect_query(proved(MProp)) = "P"-MProp.
dissect_query(unsolved(MProp, _)) = "U"-MProp.
dissect_query(assumed(MProp, _)) = "A"-MProp.
dissect_query(asserted(prop(MProp))) = "R"-MProp.
dissect_query(asserted(impl(_, MProp))) = "R"-MProp.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- func default_costs = costs.

default_costs = costs(1.0, 1.0, 0.1).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred is_ctx_proof(proof(ctx_modality)::in) is det.

is_ctx_proof(_).

/*
	io.command_line_arguments(CmdArgs, !IO),
	(if
		CmdArgs = [Goal, GoalAssumeCost],
		string.to_float(GoalAssumeCost, InitAssumeCost)
	then
		some [!Ctx] (
			!:Ctx = new_ctx,

			loading.load_stdin(Result, !Ctx, !IO),
			format("read result: %s\n", [s(string(Result))], !IO),

			vs(InitMProp, InitVarset) = det_string_to_vsmprop(Goal),

			P0 = proof(vs([[unsolved(InitMProp, const(InitAssumeCost))]], InitVarset), []),

			format("goal:\n  %s\n\n", [s(vsmprop_to_string(vs(InitMProp, InitVarset)))], !IO),

			print_ctx(!.Ctx, !IO),

			nl(!IO),

%			DC0 = new_d_ctx,

			Proofs0 = set.to_sorted_list(solutions_set((pred((Cost-G)-P::out) is nondet :-
				Costs = costs(1.0, 0.1, 0.1),
				prove(0.0, InitAssumeCost, P0, P, Costs, !.Ctx),
				G = last_goal(P),
				Cost = cost(!.Ctx, P, Costs)
					))),

			% TODO: derivations
			% deriv: map(proved_goal, set(list(steps)))

			list.foldl((pred((Cost-G)-P::in, M0::in, M::out) is det :-
				(if map.search(M0, Cost-G, D0)
				then D1 = D0
				else D1 = set.init
				),
				set.insert(D1, P, D2),
				map.set(M0, Cost-G, D2, M)
					), Proofs0, map.init, DerivsMap),

			list.sort((pred((CA-_)-_::in, (CB-_)-_::in, Comp::out) is det :-
				float_compare(CA, CB, Comp)
					), map.to_assoc_list(DerivsMap), DerivsSorted),


			format("found %d proofs.\n", [i(length(DerivsSorted))], !IO),

			list.foldl((pred((Cost-G)-Ds::in, !.IO::di, !:IO::uo) is det :-
				print("---------------------------------------------------------------------\n", !IO),
				format("proof cost = %f\n\n", [f(Cost)], !IO),
				print("proven goal:\n  " ++ goal_to_string(G) ++ "\n", !IO),
				nl(!IO),

				print("assumptions:\n", !IO),
				print("  " ++ assumptions_to_string(!.Ctx, goal_assumptions(G)) ++ "\n", !IO),
				nl(!IO),

				print("assertions:\n", !IO),
				print("  " ++ assertions_to_string(!.Ctx, goal_assertions(G)) ++ "\n", !IO),
				nl(!IO),

				print(string.from_int(set.count(Ds)) ++ " derivation" ++ plural_s(count(Ds)) ++ ".\n", !IO),

				print("- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -\n", !IO),

				set.fold((pred(Proof::in, !.IO::di, !:IO::uo) is det :-
					is_ctx_proof(Proof),
					print_proof_trace(!.Ctx, Proof, !IO),
					nl(!IO)
						), Ds, !IO)

					), DerivsSorted, !IO)
		)
	else
		io.progname("?", ProgName, !IO),
		format(stderr_stream, "Usage: %s GOAL GOAL_ASSUMPTION_COST < FILE\n", [s(ProgName)], !IO)
	).

%------------------------------------------------------------------------------%

:- func plural_s(int) = string.

plural_s(N) = S :-
	(if N > 1
	then S = "s"
	else S = ""
	).

%------------------------------------------------------------------------------%

:- pred term_to_assumable_function_def(term.term::in, assumable_function_def(M)::out) is semidet
		<= (modality(M), term_parsable(M)).

term_to_assumable_function_def(functor(atom("="), [FuncNameTerm, DefTerms], _), FuncDef) :-
	FuncNameTerm = functor(atom(FuncName), [], _),
	term_list(DefTerms, ListCostTerms),
	list.map((pred(AssignTerm::in, MGProp-Cost::out) is semidet :-
		AssignTerm = functor(atom("="), [MPropTerm, CostTerm], _),
		term_to_mprop(MPropTerm, m(Mod, Prop)),
		ground_formula(Prop, GProp),
		MGProp = m(Mod, GProp),
		CostTerm = functor(float(Cost), [], _)
			), ListCostTerms, Costs),
	FuncDef = FuncName-map.from_assoc_list(Costs).

:- pred term_list(term.term::in, list(term.term)::out) is semidet.

term_list(functor(atom("[]"), [], _), []).
term_list(functor(atom("[|]"), [HeadTerm, TailTerms], _), [HeadTerm | Tail]) :-
	term_list(TailTerms, Tail).

%------------------------------------------------------------------------------%

*/
