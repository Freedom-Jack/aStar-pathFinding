/* -----------------------------------------------------------------------
(SWI Prolog Version)

A* search algorithm + Cycle Checking

      Identical to A* except must past an equality test for states
      so that duplicate states can be detected. 

astarCC(Start, NbFn, GoalFn, HvalFn, Answer, StateEqFn).


"Answer" is the path found using an A* search with transition costs
and neighbours of a state returned by Successors, and the heuristic
value of a state returned by Hvalfn. 

More specifically. 
More specifically. 
Start---a node in the state space. This is passed in using whatever
        representation the user designs. The design of the state space
        representation will of course influence how the other functions
        like GoalTest, Successors, HvalFn operate. Note that the state
        represenation could include other information besides the
        raw state, e.g., the move that was used to generate this
        state. It can have whatever is convenient. All you have to
        do is ensure that the other functions "GoalTest",
        "Succesors" etc. operate properly with your state your data
        structure, perhaps ignoring parts of the structure not
        relevant to them.

Successors---A function of the form
       Successors(State, ListOfNeighboursWithCosts)
       Returns a list of neighbours of a state, the elements of
       this list must be in the form of pairs
       (Cost, Newstate). Where Cost is the cost of getting to the
       neighbour and Newstate is the neighbour.

GoalTest-A predicate of the form
       GoalTest(State1)
       which is true iff State1 is a goal state.

HvalFn-A function of the form
       HvalFn(State, Hval)
       which sets Hval to be the heuristic value of the state
       (estimated distance to a goal)

Answer this is a list of states from the initial state to the final
       goal satisfying state. This is the path to the goal. 

StateEqFn(State1, State2)
       this predicate is true of two states that are equal. Note
       that if one is including extra information in the state
       representation, e.g., the action that yielded the state,
       then the equality function will have to ignore the
       irrelevant parts of the state representation. 

NOTE the signature of these functions is important. The routine
generates particular types of function calls assuming these
signatures.

NOTE you must also consult "basics" prior to using.

How Cycle checking works. There are different ways of getting CC to
work. Here we use a technique that will ensure that CC works even if
the heuristic is not monotone. The price we pay is that we might
repeat parts of the search. In particular, we store the old states
expanded as well as the g-value (the cost of the path we found to that
state). If later on we find the same state again we only reject it if
its g-value (the cost of the new path we have found) is higher than
or equal to the cost of the old path found. 

Now to the mechanics. Again there are different ways. The technique we
use is that when a state is to be expanded we first check to see if it
was expanded before with a lower cost path. If so, we reject it and move
on to the next state. When we expand a state we reject all of its
successors to which we have already expanded with cheaper
paths. Then we place all of the rest on the frontier. This
technique allows the possiblity of having duplicate paths on the
frontier: we might not have seen node n, we expand two nodes n1 and
n2 both have n as a successor, since n has never been expanded
before, the two copies of n both end up on the frontier since we
never check the states on the frontier for duplicates, only the set
of states already expanded are checked. It is more effort to
check the frontier and is not worth while unless space is very
tight---but then we would probably use IDA* instead. Because we don't
check the frontier, and might place duplicates on it, we need to
recheck a node prior to expanding it. 

Finally to implementation. Here we use a trick of utilizing the prolog
database to store the previously visited states and the cost of the
path to them---we use "assert".

----------------------------------------------------------------------- */

:- ensure_loaded('astarcommon.pl').


astarCC(Start, Successors,  GoalFn, HvalFn, Answer, StateEqFn) :-
	%% call an internal start point with Start initialized as a
	%% path. Each path also contains a pair (g-val, h-val) 
	%% stored at the front of the path. This cost information is
	%% used internally by astar, the user need not worry about the
	%% this data.
	%%
	%% Get heuristic value of start state, initialize the
	%% predicate used to store duplicate state then recursively search
	%% for a solution.
        callHvalFn(HvalFn, Start, Val), !,
        astarRecursiveCC([[(0,Val), Start]], Successors, GoalFn, HvalFn,
	                 StateEqFn, [], Answer, 0).


/* ----------------------------------------------------------
   astarRecursiveCC (internal A*)
   (3 clauses)
---------------------------------------------------------- */

%1 We found a goal.
astarRecursiveCC([ [_, FinalState | Path ] | _], _, GoalFn, _,
	_, _, Answer, NumExpanded) :-
	callGoalFn(GoalFn, FinalState), !,
	reverse([FinalState | Path], Answer),
	N is NumExpanded +1,
	writeln(['Search Successful, states expanded =', N]).

%2. We expand a previously unseen or more expensively seen node.
astarRecursiveCC([ [(Gval,_),  FinalState | Path ]| OtherPaths], Successors, 
	GoalFn, HvalFn, StateEqFn, OldStates, Answer, NumExpanded) :-
	%% Before expanding we check to see if we have visited the
	%% node with a cheaper path.
	ccOk(FinalState, Gval, OldStates, StateEqFn),
	%% Expand and search.
	%%writeln(['Expanding node: ', NumExpanded,  FinalState]),
	%%nl,
	ccRemember(FinalState, Gval, OldStates, NewOldStates),
	callSuccessors(Successors,FinalState,Neighbours), 
	expand_astarCC(Gval, FinalState , Path, Neighbours,  HvalFn, NewPaths),
	ccPrune(NewPaths,PrunedPaths, OldStates, StateEqFn),	
	ourmerge(PrunedPaths, OtherPaths, NewFrontier),
	N is NumExpanded +1,!,
	(N =< 10000 ->
	        astarRecursiveCC(NewFrontier, Successors,  GoalFn,
		                 HvalFn, StateEqFn, NewOldStates, Answer,N) ;
                writeln(['Node Expansion Limit Reached', N]), nl, !, fail).

%3 The front of the frontier was seen before via an equally cheap path.
astarRecursiveCC([ _ | OtherPaths], Successors,  GoalFn, HvalFn,
                 StateEqFn, OldStates, Answer,NumExpanded) :-
	!,
	astarRecursiveCC(OtherPaths, Successors, GoalFn, HvalFn,
                         StateEqFn, OldStates, Answer,NumExpanded).

expand_astarCC(_, _, _, [], _, []).

expand_astarCC(Gval, State, Path,
	[(Cost,NewState) | RestNeigh],  HvalFn,
	[ [(NGval, NHval), NewState, State | Path] | RestNewPaths]) :-
	%%This function massages the successor states into the proper
	%%representation of paths that are on A*'s frontier.
	%%The form of the new paths is the G and H vals in a pair followed by
	%%the new state followed by the old states. Newstate is a
	%%legit extension of the old path if the pair
	%%(Cost,NewState) is in neighbours and the new Gval is the old
	%%one plus the cost. And the new Hvalue is the hvalue of NewState.

	NGval is Gval + Cost,
	callHvalFn(HvalFn, NewState, NHval),
	expand_astarCC(Gval, State, Path, RestNeigh,  HvalFn, RestNewPaths).


/* ----------------------------------------------------------
   Cycle checking
---------------------------------------------------------- */

ccOk(_, _, [], _).
%%New path must be cheaper than ALL other paths to the same state.

ccOk(State, Gval, [(OldState,OldGval) | Rest], StateEqFn) :-
	callEqFn(StateEqFn, State, OldState), Gval < OldGval, !,
	ccOk(State, Gval, Rest, StateEqFn).

%% clause for nonmatching state.
ccOk(State, Gval, [(OldState, _) | Rest], StateEqFn) :-
	not(callEqFn(StateEqFn, State, OldState)), !,
	ccOk(State, Gval, Rest, StateEqFn).

ccRemember(State, Gval, OldStates, [(State,Gval)|OldStates]).

ccPrune([ NewPath | RestNPaths], PrunedPaths, OldStates, StateEqFn) :-
	NewPath = [(Gval, _), FinalState | _],
	ccOk(FinalState, Gval, OldStates, StateEqFn), !,
	ccPrune(RestNPaths, RestPruned, OldStates, StateEqFn),
	PrunedPaths = [NewPath | RestPruned].

ccPrune([ _ | RestNPaths], PrunedPaths, OldStates, StateEqFn) :-
	ccPrune(RestNPaths, PrunedPaths, OldStates, StateEqFn).

ccPrune([], [], _, _).



