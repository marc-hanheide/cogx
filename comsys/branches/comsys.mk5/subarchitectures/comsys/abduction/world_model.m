:- module world_model.

:- interface.
:- import_module string, int, map, set, pair.
:- import_module lf.

:- type world_id(Index)
	--->	this
	;	u(int)  % unnamed
	;	i(Index)
	.

:- type world_model(Index, Sort, Rel)
	--->	wm(
		names :: map(Index, Sort),  % sort
		reach :: set({Rel, world_id(Index), world_id(Index)}),  % reachability
		props :: map(world_id(Index), set(proposition)),  % valid propositions
		next_unnamed :: int  % lowest unused index for unnamed worlds
	).

:- type world_model == world_model(string, string, string).

:- func init = world_model.

:- pred add_lf(lf::in, world_model::in, world_model::out) is semidet.

	% Reduced model is a model for which it holds that for every reachability
	% relation R and worlds w1, w2, w3, it is true that
	%
	%   (w1, w2) \in R & (w1, w3) \in R -> w2 = w3
	%
	% i.e. every reachability relation is a (partial) function W -> W
	% rather than W -> pow(W)

:- func reduced(world_model(I, S, R)::in) = (world_model(I, S, R)::out) is semidet.

%------------------------------------------------------------------------------%

:- implementation.
:- import_module solutions, require.
:- import_module list.

init = wm(map.init, set.init, map.init, 0).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred new_unnamed_world(int::out, world_model::in, world_model::out) is det.

new_unnamed_world(Id, WM0, WM) :-
	Id = WM0^next_unnamed,
	WM = WM0^next_unnamed := Id + 1.

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred rename_merge_world(world_id(I)::in, world_id(I)::in, world_model(I, S, R)::in,
		world_model(I, S, R)::out) is det.

rename_merge_world(Old, New, WM0, WM) :-
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

	WM = wm(WM0^names, Reach, Props, WM0^next_unnamed).

%------------------------------------------------------------------------------%

add_lf(LF, !WM) :-
	add_lf0(this, _, LF, !WM).

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

:- pred add_lf0(world_id(string)::in, world_id(string)::out, lf::in,
		world_model::in, world_model::out) is semidet.

add_lf0(Cur, Cur, at(of_sort(WName, Sort), LF), WM0, WM) :-
	% add the referenced world
	(if map.search(WM0^names, WName, OldSort)
	then
		% we've already been there
		OldSort = Sort,  % conflict? => fail
		WM1 = WM0
	else
		% it's a new one
		WM1 = WM0^names := map.det_insert(WM0^names, WName, Sort)
	),
	add_lf0(i(WName), _, LF, WM1, WM).

add_lf0(Cur, i(WName), i(of_sort(WName, Sort)), WM0, WM) :-
	(
	 	Cur = this,
		fail  % should we perhaps allow this?
	;
		Cur = i(WName),
		map.search(WM0^names, WName, Sort),
		WM = WM0
	;
		Cur = u(Num),
		(if map.search(WM0^names, WName, OldSort)
		then OldSort = Sort, WM1 = WM0
		else WM1 = WM0^names := map.det_insert(WM0^names, WName, Sort)
		),
		rename_merge_world(u(Num), i(WName), WM1, WM)
	).

add_lf0(Cur, Cur, r(Rel, LF), WM0, WM) :-
	new_unnamed_world(Num, WM0, WM1),
	WM2 = WM1^reach := set.insert(WM0^reach, {Rel, Cur, u(Num)}),
	add_lf0(u(Num), _, LF, WM2, WM).

add_lf0(Cur, Cur, p(Prop), WM0, WM) :-
	(if OldProps = map.search(WM0^props, Cur)
	then NewProps = set.insert(OldProps, Prop)
	else NewProps = set.make_singleton_set(Prop)
	),
	WM = WM0^props := map.set(WM0^props, Cur, NewProps).
	
add_lf0(Cur0, Cur, and(LF1, LF2), WM0, WM) :-
	add_lf0(Cur0, Cur1, LF1, WM0, WM1),
	add_lf0(Cur1, Cur, LF2, WM1, WM).

%------------------------------------------------------------------------------%
	
:- type mergable_result(I)
	--->	merge(world_id(I), world_id(I))  % merge 2nd into 1st
	;	none
	.

reduced(WM) = RWM :-
	first_mergable_worlds(set.to_sorted_list(WM^reach), Res),
	(
	 	Res = merge(W2, W3),
		rename_merge_world(W3, W2, WM, WM0),
		RWM = reduced(WM0)
	;
		Res = none,
		RWM = WM
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
