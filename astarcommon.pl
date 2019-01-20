callHvalFn(HvalFn, State, X) :-
	%% create a call to HvalFn on State return value X.
	H =.. [HvalFn, State, X], H.

callGoalFn(GoalTest,State) :-
	%%true if GoalTest applied to state is true.
	G =.. [GoalTest, State], G.

callSuccessors(Successors, State, Neighbours) :-
	%% create call to Successors on State return value Neighbours
	N =.. [Successors, State, Neighbours], N.

%%% Test if two states are equal
callEqFn(StateEqFn, S1, S2) :-
	%% create a call to HvalFn on State return value X.
	E =.. [StateEqFn, S1, S2], !, E.

%% Cycle check a single path. 
cyclecheck(Nstate, [State| OtherStates], StateEqFn) :- 
	not(callEqFn(StateEqFn,Nstate,State)),
	cyclecheck(Nstate, OtherStates, StateEqFn).

cyclecheck(_, [], _).

%%ourmerge---keep the frontier in order sorted by f-value

%% Old part of the frontier is already sorted.
%% but New part is not.

ourmerge([NewState|RestNewStates], OldStates, NewFrontier) :-
	insert(NewState,OldStates,New),
	ourmerge(RestNewStates, New, NewFrontier).

ourmerge([],AllAdded,AllAdded) :- !.

/* ----------------------------------------------------------
  insert_list
  keep the frontier in order sorted by f-value
---------------------------------------------------------- */

%% Old part of the frontier is already sorted.
%% but New part is not.

insert_list([NewState|RestNewStates], OldStates, NewFrontier) :-
	insert(NewState,OldStates,New),
	insert_list(RestNewStates, New, NewFrontier).

insert_list([],AllAdded,AllAdded) :- !.

%%. Insert either is a merge insert of old into new. 
insert(NewState, [OldState | RestOld], 
	[NewState, OldState | RestOld]) :-
	lowerOrEqualFvalue(NewState, OldState), !.

insert(NewState, [OldState | RestOld], 
	[OldState | InsertedIntoRest]) :-
	greaterFvalue(NewState,OldState),
	insert(NewState, RestOld, InsertedIntoRest), !.

insert(NewState, [], [NewState]).

lowerOrEqualFvalue([(G1,H1) | _], [(G2, H2) | _]) :-
	X is G1+H1, Y is G2+H2, X =< Y, !.

greaterFvalue([(G1,H1) | _], [(G2, H2) | _]) :-
	X is G1+H1, Y is G2+H2, X > Y, !.
