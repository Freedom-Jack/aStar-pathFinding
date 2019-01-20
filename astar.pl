/* -----------------------------------------------------------------------
(SWI Prolog Version)

A* search algorithm


astar(Start, Successors, GoalTest, HvalFn, Answer).

True if "Answer" is a path from "Start" to a state on which
GoalTest succeeds.


"Answer" is the path found using an A* search with transition costs
and neighbours of a state returned by Successors, and the heuristic
value of a state returned by Hvalfn. 

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

NOTE the signature of these functions is important. The routine
generates particular types of function calls assuming these
signatures.

NOTE you must execute
 consult(basics).

Before astar will work.

----------------------------------------------------------------------- */

:- ensure_loaded('astarcommon.pl').


astar(Start, Successors, GoalTest, HvalFn, Answer, StateEqFn) :-
	%% call an internal start point with Start initialized as a
	%% path. Each path also contains the pair (g-val, h-val),
	%% stored at the front of the path. This cost information is
	%% used internally by astar, the user need not worry about the
	%% this data.
	%%
	%% Get heuristic value of statt state, then recursively search
	%% for a solution.
	callHvalFn(HvalFn, Start, Val), !,
        astarRecursive([[(0,Val), Start]], Successors, StateEqFn, GoalTest, HvalFn, Answer, 0).


astarRecursive([ ], _, _, _, _, 'unsolvable', NumExpanded) :- !,
	writeln(['Frontier empty, problem shown unsolveable, states expanded =', NumExpanded]),
	!, fail.

astarRecursive([ [_, FinalState | Path ] | _], _, _, GoalTest, _, Answer, NumExpanded) :-
	%%found goal
	callGoalFn(GoalTest,FinalState), !,  % cut is to abort this clause if not at goal state
	reverse([FinalState | Path], Answer),
	N is NumExpanded + 1,
	writeln(['Search Successful, states expanded =', N]).

astarRecursive([ [(Gval,_), FinalState | Path ]| OtherPaths],
	       Successors, StateEqFn, GoalTest, HvalFn, Answer, NumExpanded) :-
	%% Expand and search.
	%%writeln(['Expanding: ',  [FinalState|Path]]),
	%%nl,
	callSuccessors(Successors,FinalState,Neighbours),
	expand_astar(Gval, FinalState, Path, Neighbours,  StateEqFn, HvalFn, NewPaths),
	ourmerge(NewPaths, OtherPaths, NewFrontier),
	N is NumExpanded + 1,!,
	%%%Here you can place a node expansion bound
	(N =< 10000 ->
	        astarRecursive(NewFrontier, Successors, StateEqFn, GoalTest,
			       HvalFn, Answer, N) ;
                writeln(['Node Expansion Limit Reached', N]), nl, !, fail).


expand_astar(Gval,  State , Path,
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
	expand_astar(Gval, State, Path, RestNeigh, StateEqFn, HvalFn, RestNewPaths).

expand_astar(Gval,  State , Path,
	[(_,_)| RestNeigh], StateEqFn, HvalFn, RestNewPaths) :-
	%% For newstates that fail the cyclecheck
	expand_astar(Gval,State,Path, RestNeigh, StateEqFn, HvalFn, RestNewPaths).

expand_astar(_, _, _, [],_,_, []). 


