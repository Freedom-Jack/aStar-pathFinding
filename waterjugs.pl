%% Waterjugs
%% States are pairs of positive integers x/y representing the number
%% of gallons in the 3 gallon jug and the number of gallons in the 4
%% gallon jug. 

% 1. Successor state function, returns list of successors of a state,
%    annotated by cost.  
%    There are 6 actions, (1) pour the 3 gallon jug into the 4 gallon
%    jug, (2) pour the 3 gallon jug onto the ground, (3) fill the 3
%    gallon jug, (4) pour the 4 gallon jug into the 3 gallon jug, (5)
%    pour the 4 gallon jug  onto the ground and (6) fill the 4 gallon
%    jug. 
%
%    Note that for actions 1 and 3, the pouring stops when either the
%    destination jug is full or the source jug is empty.
%    Actions are considered to be unavailable if they lead back to the
%    same state. 
%
%
%    Note structure of neighbours list, a list of pair of the form
%    (Cost,State), where Cost is the cost of getting to State from the
%    current state. 

%% try the actions in order. All actions have cost 1.
min(Z,X,Y) :- X < Y, !, Z=X.
min(Z,X,Y) :- X >= Y, !, Z=Y.

threeToFour(X/Y, [(1, threeto4-NX/NY)]) :-
	Space is 4 - Y, min(TransferAmount, X, Space),
	TransferAmount > 0,
	NX is X - TransferAmount, NY is Y + TransferAmount.
threeToFour(X/Y, []) :-
	Space is 4 - Y, min(TransferAmount, X, Space),
	TransferAmount =:= 0.

threeToGround(X/Y, [(1, threetoG-0/Y)]) :- X > 0.
threeToGround(X/_, []) :- X =:= 0.

fillThree(X/Y, [(1, fill3-3/Y)]) :- X < 3.
fillThree(X/_, []) :- X =:= 3.

fourToThree(X/Y, [(1, fourto3-NX/NY)]) :-
	Space is 3 - X, min(TransferAmount, Y, Space), 
	TransferAmount > 0,
	NX is X + TransferAmount, NY is Y - TransferAmount.
fourToThree(X/Y, []) :-
	Space is 3 - X, min(TransferAmount, Y, Space),
	TransferAmount =:= 0.

fourToGround(X/Y, [(1, fourtoG-X/0)]) :- Y > 0.
fourToGround(_/Y, []) :- Y =:= 0.

fillFour(X/Y, [(1, fill4-X/4)]) :- Y < 4.
fillFour(_/Y, []) :- Y =:= 4.

successors(_-X/Y, Successors) :-
	threeToFour(X/Y, N1), 
	threeToGround(X/Y, N2),
	fillThree(X/Y,N3),
	fourToThree(X/Y, N4),
	fourToGround(X/Y, N5),
	fillFour(X/Y,N6),
	append(N1, N2, P1),
	append(N3, P1, P2),
	append(N4, P2, P3),
	append(N5, P3, P4),
	append(N6, P4, Successors).
	

% 2. Goal test function: 2 gallons in the four gallon jug.

goal(_-_/2).

%% We simply pass astar "goal", and use setGoalState to dynamically
%% alter the current goal.

%3. Heuristic functions.

hfnUniform(_,0).              % causes breadth first search.

%% Equality ignores the aciton. 

stateEqFn(_-X,_-Y) :- X=Y.

%%Sample Searches

% ?- astar(nop-0/0, successors, goal, hfnUniform, Path, stateEqFn).

% ?- astarCC(nop-0/0, successors, goal, hfnUniform, Path, stateEqFn).

% ?- idastar(nop-0/0, successors, goal, breadthfn, Path, stateEqFn).
