:- module world_model.

:- interface.
:- import_module string, int, map, set.
:- import_module lf, ontology.

:- type world_id(Index)
	--->	initial
	;	u(int)  % unnamed
	;	i(Index)
	.

:- type world_model(Index, Sort, Rel).

:- func worlds(world_model(I, S, R)) = map(I, S).
:- func reach(world_model(I, S, R)) = set({R, world_id(I), world_id(I)}).
:- func props(world_model(I, S, R)) = map(world_id(I), set(proposition)).

:- func unnamed_world_indices(world_model(I, S, R)) = set(int).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- type world_model == world_model(string, string, string).

:- func init = world_model(I, S, R).

	% add_lf(M0, LF, M)
	% True iff
	%   M0 \/ model-of(LF) = M
	%
	% i.e. add LF to M0 so that it is consistent, fail if not possible.
	%
:- pred add_lf(OS, world_model(I, S, R), lf(I, S, R), world_model(I, S, R)) <= isa_ontology(OS, S).
:- mode add_lf(in, in, in, out) is semidet.

:- pred union(OS, world_model(I, S, R), world_model(I, S, R), world_model(I, S, R)) <= isa_ontology(OS, S).
:- mode union(in, in, in, out) is semidet.

	% satisfies(M, LF)
	% True iff
	%   M |= LF
	%
:- pred satisfies(OS, world_model(I, S, R), lf(I, S, R)) <= isa_ontology(OS, S).
:- mode satisfies(in, in, in) is semidet.

	% Reduced model is a model for which it holds that for every reachability
	% relation R and worlds w1, w2, w3, it is true that
	%
	%   (w1, w2) \in R & (w1, w3) \in R -> w2 = w3
	%
	% i.e. every reachability relation is a (partial) function W -> W
	% rather than W -> pow(W)

	% reduced(M) = RM
	% True iff RM is a functionally reduced version of M.
	%
:- func reduced(OS::in, world_model(I, S, R)::in) = (world_model(I, S, R)::out) is semidet
		<= isa_ontology(OS, S).
:- func det_reduced(OS, world_model(I, S, R)) = world_model(I, S, R) <= isa_ontology(OS, S).


:- func lfs(world_model(I, S, R)) = set(lf(I, S, R)).

%------------------------------------------------------------------------------%

:- implementation.
:- import_module solutions, require.
:- import_module list, pair.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
	
:- type world_model(Index, Sort, Rel)
	--->	wm(
		worlds :: map(Index, Sort),  % sort
		reach :: set({Rel, world_id(Index), world_id(Index)}),  % reachability
		props :: map(world_id(Index), set(proposition)),  % valid propositions
		next_unnamed :: int  % lowest unused index for unnamed worlds
	).

%------------------------------------------------------------------------------%

init = wm(map.init, set.init, map.init, 0).

%init = wm(map.init, set.init, map.init, 0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

unnamed_world_indices(M) = Ints :-
	Ints = solutions_set((pred(Int::out) is nondet :-
		( member({_, u(Int), _}, M^reach)
		; member({_, _, u(Int)}, M^reach)
		; member(u(Int), keys(M^props))
		)
			)).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred new_unnamed_world(int::out, world_model(I, S, R)::in, world_model(I, S, R)::out) is det.

new_unnamed_world(Id, WM0, WM) :-
	Id = WM0^next_unnamed,
	WM = WM0^next_unnamed := Id + 1.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred rename_merge_world(OS::in, world_id(I)::in, world_id(I)::in, world_model(I, S, R)::in,
		world_model(I, S, R)::out) is det <= isa_ontology(OS, S).

rename_merge_world(Ont, Old, New, WM0, WM) :-
	Reach = set.map((func({Rel, OldIdA, OldIdB}) = {Rel, NewIdA, NewIdB} :-
		(OldIdA = Old -> NewIdA = New ; NewIdA = OldIdA),
		(OldIdB = Old -> NewIdB = New ; NewIdB = OldIdB)
			), WM0^reach),

	(if map.search(WM0^props, Old, OldsProps0)
	then OldsProps = OldsProps0
	else OldsProps = set.init
	),
	(if map.search(WM0^props, New, NewsProps0)
	then NewsProps = set.union(NewsProps0, OldsProps)
	else NewsProps = OldsProps
	),
	DelProps = map.delete(WM0^props, Old),
	(if NewsProps = set.init
	then Props = DelProps
	else Props = map.set(DelProps, New, NewsProps)
	),

	WM = wm(WM0^worlds, Reach, Props, WM0^next_unnamed).

%------------------------------------------------------------------------------%

add_lf(Ont, !.WM, LF, !:WM) :-
	add_lf0(Ont, initial, _, LF, !WM).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred add_lf0(OS::in, world_id(I)::in, world_id(I)::out, lf(I, S, R)::in,
		world_model(I, S, R)::in, world_model(I, S, R)::out) is semidet <= isa_ontology(OS, S).

add_lf0(Ont, Cur, Cur, at(of_sort(WName, Sort), LF), WM0, WM) :-
	% add the referenced world
	(if map.search(WM0^worlds, WName, OldSort)
	then
		% we've already been there
		NewSort = more_specific(Ont, OldSort, Sort)
	else
		% it's a new one
		NewSort = Sort
	),
	WM1 = WM0^worlds := map.set(WM0^worlds, WName, NewSort),
	add_lf0(Ont, i(WName), _, LF, WM1, WM).

add_lf0(Ont, Cur, i(WName), i(of_sort(WName, Sort)), WM0, WM) :-
	(
	 	Cur = initial,
		fail  % should we perhaps allow this?
	;
		Cur = i(WName),
		map.search(WM0^worlds, WName, OldSort),
		NewSort = more_specific(Ont, OldSort, Sort),
		WM = WM0^worlds := map.set(WM0^worlds, WName, NewSort)
	;
		Cur = u(Num),
		(if map.search(WM0^worlds, WName, OldSort)
		then NewSort = more_specific(Ont, OldSort, Sort)
		else NewSort = Sort
		),
		WM1 = WM0^worlds := map.set(WM0^worlds, WName, NewSort),
		rename_merge_world(Ont, u(Num), i(WName), WM1, WM)
	).

add_lf0(Ont, Cur, Cur, r(Rel, LF), WM0, WM) :-
	new_unnamed_world(Num, WM0, WM1),
	WM2 = WM1^reach := set.insert(WM0^reach, {Rel, Cur, u(Num)}),
	add_lf0(Ont, u(Num), _, LF, WM2, WM).

add_lf0(Ont, Cur, Cur, p(Prop), WM0, WM) :-
	(if OldProps = map.search(WM0^props, Cur)
	then NewProps = set.insert(OldProps, Prop)
	else NewProps = set.make_singleton_set(Prop)
	),
	WM = WM0^props := map.set(WM0^props, Cur, NewProps).
	
add_lf0(Ont, Cur0, Cur, and(LF1, LF2), WM0, WM) :-
	add_lf0(Ont, Cur0, Cur1, LF1, WM0, WM1),
	add_lf0(Ont, Cur1, Cur, LF2, WM1, WM).

%------------------------------------------------------------------------------%

:- pred map_merge_op(pred(V, V, V), map(K, V), map(K, V), map(K, V)).
:- mode map_merge_op(pred(in, in, out) is det, in, in, out) is det.
:- mode map_merge_op(pred(in, in, out) is semidet, in, in, out) is semidet.

map_merge_op(Pred, M1, M2, M) :-
	map.to_sorted_assoc_list(M1, L1),
	map.to_sorted_assoc_list(M2, L2),
	assoc_lists_merge_op(Pred, L1, L2, L),
	M = map.from_assoc_list(L).

:- pred assoc_lists_merge_op(pred(V, V, V), list(pair(K, V)), list(pair(K, V)), list(pair(K, V))).
:- mode assoc_lists_merge_op(pred(in, in, out) is det, in, in, out) is det.
:- mode assoc_lists_merge_op(pred(in, in, out) is semidet, in, in, out) is semidet.

assoc_lists_merge_op(_, [], [], []).
assoc_lists_merge_op(_, [H|T], [], [H|T]).
assoc_lists_merge_op(_, [], [H|T], [H|T]).
assoc_lists_merge_op(MergeProp, [K1-V1|T1], [K2-V2|T2], [K-V|T]) :-
	compare(Comp, K1, K2),
	(
		Comp = (=),
		K = K1,
		call(MergeProp, V1, V2, V),
		assoc_lists_merge_op(MergeProp, T1, T2, T)
	;
		Comp = (<),
		K = K1, V = V1,
		assoc_lists_merge_op(MergeProp, T1, [K2-V2|T2], T)
	;
		Comp = (>),
		K = K2, V = V2,
		assoc_lists_merge_op(MergeProp, [K1-V1|T1], T2, T)
	).

%------------------------------------------------------------------------------%

	% merge M2 into M1
union(Ont, M1, M2, M) :-
	ListUM2 = to_sorted_list(unnamed_world_indices(M2)),

		% rename unnamed worlds so that we don't have any name clashes
	list.foldr((pred(Int::in, Mx0::in, Mx::out) is det :-
		rename_merge_world(Ont, u(Int), u(Int + M1^next_unnamed), Mx0, Mx)
			), ListUM2, M2, M2R),

		% increase the generator counter correspondingly
	NextUnnamed = M1^next_unnamed + M2^next_unnamed,

		% merge worlds and sorts
	map_merge_op((pred(S1::in, S2::in, S::out) is semidet :-
		S = more_specific(Ont, S1, S2)
			), M1^worlds, M2R^worlds, Worlds),

		% merge reachability relations
	set.union(M1^reach, M2R^reach, Reach),

		% merge propositions
	map_merge_op((pred(Ps1::in, Ps2::in, Ps::out) is det :-
		set.union(Ps1, Ps2, Ps)
			), M1^props, M2R^props, Props),

	M = wm(Worlds, Reach, Props, NextUnnamed).

%------------------------------------------------------------------------------%

satisfies(Ont, M, LF) :-
	satisfies0(Ont, initial, M, LF).

:- pred satisfies0(OS::in, world_id(I)::in, world_model(I, S, R)::in, lf(I, S, R)::in) is semidet
		<= isa_ontology(OS, S).

satisfies0(Ont, _WCur, M, at(of_sort(Idx, Sort), LF)) :-
	satisfies0(Ont, i(Idx), M, i(of_sort(Idx, Sort))),
	satisfies0(Ont, i(Idx), M, LF).

satisfies0(Ont, i(Idx), M, i(of_sort(Idx, LFSort))) :-
	map.search(M^worlds, Idx, Sort),
	isa(Ont, Sort, LFSort).

satisfies0(Ont, WCur, M, r(Rel, LF)) :-
	set.member({Rel, WCur, WReach}, M^reach),
	satisfies0(Ont, WReach, M, LF).

satisfies0(Ont, WCur, M, p(Prop)) :-
	set.member(Prop, map.search(M^props, WCur)).

satisfies0(Ont, WCur, M, and(LF1, LF2)) :-
	satisfies0(Ont, WCur, M, LF1),
	satisfies0(Ont, WCur, M, LF2).

%------------------------------------------------------------------------------%
	
:- type mergable_result(I)
	--->	merge(world_id(I), world_id(I))  % merge 2nd into 1st
	;	none
	.

reduced(Ont, WM) = RWM :-
	first_mergable_worlds(set.to_sorted_list(WM^reach), Res),
	(
	 	Res = merge(W2, W3),
		rename_merge_world(Ont, W3, W2, WM, WM0),
		RWM = reduced(Ont, WM0)
	;
		Res = none,
		RWM = WM
	).

det_reduced(Ont, M) = RM :-
	(if RM0 = reduced(Ont, M)
	then RM = RM0
	else error("model irreducible in func det_reduced/1")
	).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

	% Fails when a reachability relation that cannot be reduced to a partial
	% function is found.
:- pred first_mergable_worlds(list({R, world_id(I), world_id(I)})::in, mergable_result(I)::out) is semidet.

first_mergable_worlds([], none).
first_mergable_worlds([{R, W1, W2} | Rest], Result) :-
	(if
		list.find_first_map((pred({Rx, W1x, W3x}::in, W3x::out) is semidet :-
			Rx = R, W1x = W1), Rest, W3)
	then
		(W2 = u(_), W3 = u(_) -> Result = merge(W2, W3) ;
		(W2 = i(_), W3 = u(_) -> Result = merge(W2, W3) ;
		(W2 = u(_), W3 = i(_) -> Result = merge(W3, W2) ;
		fail  % W2 and W3 are both indexed by nominals => non-reducible
		)))
	else
		first_mergable_worlds(Rest, Result)
	).

%------------------------------------------------------------------------------%

lfs(M) = LFs :-
	LFs = solutions_set((pred(LF::out) is nondet :-
		(
			map.member(M^props, W, SetProps),
			W = i(Idx), M^worlds^elem(Idx) = Sort,
			set.member(Prop, SetProps),
			LF = at(of_sort(Idx, Sort), p(Prop))
		;
			set.member({Rel, W1, W2}, M^reach),
			W1 = i(Idx1), M^worlds^elem(Idx1) = Sort1,
			W2 = i(Idx2), M^worlds^elem(Idx2) = Sort2,
			LF = at(of_sort(Idx1, Sort1), r(Rel, i(of_sort(Idx2, Sort2))))
		)
			)).
