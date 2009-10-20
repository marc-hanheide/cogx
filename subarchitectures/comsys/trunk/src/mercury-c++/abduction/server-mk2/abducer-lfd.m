:- module 'abducer-lfd'.

:- interface.

:- import_module io.

:- pred main(io::di, io::uo) is det.

%------------------------------------------------------------------------------%

:- implementation.

:- import_module require, solutions.
:- import_module map, set, list, pair, assoc_list, string, float, int, bag, bool.
:- import_module utils.
:- import_module abduction, formula, context, costs.
:- import_module loading.

:- import_module ctx_modality, ctx_loadable, ctx_io, ctx_loadable_io.
:- import_module modality, stringable.

:- import_module parser, term_io, term, varset, formula_io, formula_ops, costs.
:- import_module gc.
:- import_module protocol.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

main(!IO) :-
	inner_loop(new_ctx, _, !IO).

:- pred inner_loop(ctx::in, ctx::out, io::di, io::uo) is det.

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

:- pred process_request(protocol.request::in, ctx::in, ctx::out, io::di, io::uo) is det.

process_request(init_ctx, !Ctx, !IO) :-
	print(stderr_stream, "[REQUEST] init_ctx\n", !IO),
	!:Ctx = new_ctx.

process_request(load_file(Filename), !Ctx, !IO) :-
	print(stderr_stream, "[REQUEST] load_file\n", !IO),
	loading.load_file(Filename, _Result, !Ctx, !IO),
	print("ok.\n", !IO),
	flush_output(!IO),
	print(stderr_stream, "[done] load_file\n", !IO).

process_request(clear_rules, !Ctx, !IO) :-
	print(stderr_stream, "[REQUEST] clear_rules\n", !IO),
	set_rules(set.init, !Ctx),
	print(stderr_stream, "[done] clear_rules\n", !IO).

process_request(clear_facts, !Ctx, !IO) :-
	print(stderr_stream, "[REQUEST] clear_facts\n", !IO),
	set_facts(set.init, !Ctx),
	print(stderr_stream, "[done] clear_rules\n", !IO).

process_request(clear_facts_by_modality(Modality), !Ctx, !IO) :-
	print(stderr_stream, "[REQUEST] clear_facts_by_modality\n", !IO).

process_request(clear_assumables, !Ctx, !IO) :-
	print(stderr_stream, "[REQUEST] clear_assumables\n", !IO),
	set_assumables(map.init, !Ctx),
	print(stderr_stream, "[done] clear_rules\n", !IO).

process_request(add_fact, !Ctx, !IO) :-
	print(stderr_stream, "[REQUEST] add_fact\n", !IO).

process_request(add_assumable, !Ctx, !IO) :-
	print(stderr_stream, "[REQUEST] add_assumable\n", !IO).

process_request(prove, !Ctx, !IO) :-
	print(stderr_stream, "[REQUEST] prove\n", !IO).

process_request(get_best_proof, !Ctx, !IO) :-
	print(stderr_stream, "[REQUEST] get_best_proof\n", !IO).


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

:- pred is_ctx_proof(proof(ctx_modality)::in) is det.

is_ctx_proof(_).

*/
