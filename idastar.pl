/* -----------------------------------------------------------------------
(SWI Prolog Version)

Iterative deepening A* search with Path checking

idastar(Start, Successors, GoalFn, HvalFn, Answer, StateEqFn).


True if "Answer" is a path from Start to a state for which
GoalFn is true, where Answer is the path found using an IDA*
search with costs and neighbours of a state returned by Successors and
the heuristic value of a state returned by Hvalfn.

More specifically. 
Start---a node in the state space using whatever representation
        the user designs. The design of the state space
        representation will of course influence how the other functions 
        like GoalFn operate. Note that the state represenation
        might include other information besides the raw state,
        e.g., the move that was used to generate this state. 

Successors---A function of the form
       Successors(State, ListOfNeighboursWithCosts)
       Returns a list of neighbours of a state, the elements of
       this list must be in the form of pairs
       (Cost,Newstate)
       where Cost is the cost of getting to the neighbour and
       Newstate is the neighbour. 

GoalFn-A predicate of the form
       GoalFn(State1)
       which is true iff State1 is a goal state.

HvalFn-A function of the form
       HvalFn(State,Hval)
       which sets Hval to be the heuristic value of the state
       (estimated distance to a goal)

Answer this is a list of states from the initial state to the final
       goal satisfying state. This is the path to the goal. 

NOTE the signature of these functions is important. The routine
generates particular types of function calls assuming these
signatures.

Note you must execute
consult(basics).

Before idastar will work.

----------------------------------------------------------------------- */


/* ----------------------------------------------------------
   IDA*

Idastar works much like astar. Except that it puts the new states at the front
of the frontier rather than in sorted order.
Also the internal search routine carries around three extra parameters
Start---the state from which to restart the search.
Bound---the f-value bound. No node with f-value greater than this is expanded.
BestBeyondBound---the min f-value of a node that lies beyond the f-value horizon.
 this node was not expanded but it was placed on the frontier at 
 some point.

 We start of with BestBeyondBound equal to the constant noneSeen.
 If we end the search with BestBeyondBound equal to noneSeen, then
 this means that the search never encountered a node beyond the
 current F-value horizon. so we do not need to start the search again.
 Otherwise BestBeyondBound becomes the new f-value bound to use.
---------------------------------------------------------- */

:- ensure_loaded('astarcommon.pl').

idastar(Start, Successors, GoalFn, HvalFn, Answer, StateEqFn) :-
	%% call an internal start point with Start initialized as a
	%% path. Each path also contains a pair (g-val, h-val) 
	%% for the final node of the path.
	callHvalFn(HvalFn, Start, Val),!,
        idAstarRecursive(Start, Val, noneSeen, [[(0,Val), Start]],
			 Successors, StateEqFn, GoalFn, HvalFn, Answer, 0). 


idAstarRecursive(_, Bound, _, [ [(Gval,_), FinalState | Path ]| _],
		 _, _, GoalFn, _, Answer, NumExpanded) :-
	Gval =< Bound,
	%%found goal within bound?
	callGoalFn(GoalFn,FinalState), !,  % cut is to abort this clause if not at goal state
	reverse([FinalState | Path], Answer),
	N is NumExpanded +1,
	writeln(['Search Successful, states expanded =', N]).

idAstarRecursive(Start, Bound, BestBeyondBound,
		 [ [(Gval,Hval), FinalState | Path ]| OtherPaths],
		 Successors, StateEqFn, GoalFn, HvalFn, Answer, NumExpanded) :- 
	Fval is Gval+Hval,
	Fval =< Bound, !,
	%% Expand node within horizon .
	%%writeln(['Expanding: ',  [FinalState|Path]]),
	%%nl,
	callSuccessors(Successors,FinalState,Neighbours), 
	expand_idastar(Gval, FinalState, Path, Neighbours, StateEqFn, HvalFn, NewPaths),
	append(NewPaths, OtherPaths, NewFrontier),
	N is NumExpanded +1,!,
        %% Here you can place a node expansion bound. 
	(N =< 20000 ->
                idAstarRecursive(Start,Bound,BestBeyondBound,NewFrontier, Successors, StateEqFn, GoalFn, HvalFn, Answer,N);
                writeln(['Node Expansion Limit Reached', N]), nl, fail).

idAstarRecursive(Start, Bound, BestBeyondBound,
		 [ [(Gval,Hval), _ | _ ]| OtherPaths], Successors,
		 StateEqFn, GoalFn, HvalFn, Answer, NumExpanded) :-
	Fval is Gval+Hval,
	Fval > Bound,!,
	%% Found node beyond horizon .
	%%writeln(['Bound: ',  Fval]),
	%nl,
	idastarMin(Fval,BestBeyondBound,NewBest),
	idAstarRecursive(Start, Bound, NewBest, OtherPaths,
			 Successors, StateEqFn, GoalFn, HvalFn, Answer, NumExpanded).

idAstarRecursive(Start, _, BestBeyondBound, [], Successors, StateEqFn,
		 GoalFn, HvalFn, Answer, NumExpanded) :-
	%% exhausted the search start again if we have seen other nodes beyond the frontier.
	%% set min of the f-values seen beyond the frontier as the new bound.
	writeln(['Restarting Search.']),
	nl,
	BestBeyondBound \= noneSeen ->
	   (callHvalFn(HvalFn,Start,Val), !,
            idAstarRecursive(Start, BestBeyondBound, noneSeen,
			     [[(0,Val), Start]], Successors,
			     StateEqFn, GoalFn, HvalFn, Answer,
			     NumExpanded)). 

idastarMin(X,noneSeen,X) :- !.
idastarMin(X,Y,X) :- X =< Y.
idastarMin(X,Y,Y) :- Y < X.

/* ****************************************************** */

expand_idastar(Gval,  State , Path,
       [(Cost,NewState) | RestNeigh], StateEqFn, HvalFn,
       [ [(NGval, NHval), NewState, State | Path] | RestNewPaths]) :-
	%%The form of the new paths is the G and H vals in a pair followed by
	%%the new state followed by the old states. Newstate is a
	%%legit extension of the old path if the pair
	%%(Cost,NewState) is in neighbours, if Newstate is not equal to
	%%a previous state on the path, and the new Gval is the old
	%%one plus the cost. And the new Hvalue is the hvalue f NewState.

	cyclecheck(NewState, [State | Path], StateEqFn), !, %cycle checking!
	NGval is Gval + Cost,
	callHvalFn(HvalFn, NewState, NHval),
	expand_idastar(Gval, State, Path, RestNeigh, StateEqFn, HvalFn, RestNewPaths).

expand_idastar(Gval,  State , Path,
	[(_,_)| RestNeigh], StateEqFn, HvalFn, RestNewPaths) :-
	%% For newstates that fail the cyclecheck
	expand_idastar(Gval,State,Path, RestNeigh, StateEqFn, HvalFn, RestNewPaths).

expand_idastar(_, _, _, [],_,_, []). 


