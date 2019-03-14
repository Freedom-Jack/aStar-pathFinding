/* load the three search algorithms */
:- ensure_loaded('astar.pl').
:- ensure_loaded('astarCC.pl').
:- ensure_loaded('idastar.pl').

/* ------------------------------------------------------- */

/* successors( +State, -Neighbours)

   Neighbors is a list of elements (Cost, NewState) where
   NewState is a state reachable from State by one action and
   Cost is the cost for that corresponding action (=1 in our
   case)
*/

/*----------------------------------------------------------
	successors( State, Succs ) :-
		leftNeighbours(State, LResult) -> member([1,LResult], Succs),
		rightNeighbours(State, RResult) -> member([1,RResult], Succs),
		upNeighbours(State, UResult) -> member([1,UResult], Succs),
		downNeighbours(State, DResult) -> member([1,DResult], Succs).
 ------------------------------------------------------- */

% Find all the matching elements base on the move methods, since all costs are the same, we simply use findall built-in
	successors( State, Succs ) :- findall((1,OneThing), checker(State, OneThing), Succs).

	checker(State, GoodThing) :-
		leftNeighbours(State, GoodThing);
		rightNeighbours(State, GoodThing);
		upNeighbours(State, GoodThing);
		downNeighbours(State, GoodThing).



%	Getting the neighbours from performing a move(up, down, left, right) of the blank
	leftNeighbours(S, LResult) :- nth0(Index, S, 0), length(S, L), perfectSquare(L, 0), leftOK(Index, sqrt(L), 0), !, moveLeft(S, LResult, Index).
	rightNeighbours(S, RResult) :- nth0(Index, S, 0), length(S, L), perfectSquare(L, 0), rightOK(Index, sqrt(L), 0), !, moveRight(S, RResult, Index).
	upNeighbours(S, UResult) :- nth0(Index, S, 0), length(S, L), perfectSquare(L, 0), upOK(Index, sqrt(L)), !, moveUp(S, UResult, Index, round(sqrt(L))).
	downNeighbours(S, DResult) :- nth0(Index, S, 0), length(S, L), perfectSquare(L, 0), downOK(Index, sqrt(L)), !, moveDown(S, DResult, Index, round(sqrt(L))).



%	Check if the index of 0 is at the left most column(kn), if yes, fail
% I: index; SIDE: length of the side of the matrix; C: counter; NC: New counter.
	leftOK(I, SIDE, C) :- I is (C * SIDE), !, fail.
	leftOK(I, SIDE, C) :- I > (C * SIDE), NC is C + 1, leftOK(I, SIDE, NC).
	leftOK(I, SIDE, C) :- I < (C * SIDE).

% Check if the index of 0 is at the right most column(kn - 1), if yes, fail
  rightOK(I, SIDE, C) :- I is (C * SIDE - 1), !, fail.
  rightOK(I, SIDE, C) :- I > (C * SIDE - 1), NC is C + 1, rightOK(I, SIDE, NC).
  rightOK(I, SIDE, C) :- I < (C * SIDE - 1).

% Check if the index of 0 is at the first row(0 <= index <= n - 1), if yes, fail
  upOK(I, SIDE) :- I > SIDE - 1.

% Check if the index of 0 is at the last row((n-1)*n <= index <= (n^2) - 1), if yes, fail
  downOK(I, SIDE) :- I < ((SIDE - 1) * SIDE).



% Move methods, for generating the corresponding neighbours according to the move
% moveLeft, move the 0 to its left, which means moving it to (index - 1) spot
  moveLeft(State, Answer, ZeroIndex) :- Temp is (ZeroIndex - 1), swapEle(State, ZeroIndex, Temp, Answer).

% moveRight, move the 0 to its right, which means moving it to (index + 1) spot
  moveRight(State, Answer, ZeroIndex) :- Temp is (ZeroIndex + 1), swapEle(State, ZeroIndex, Temp, Answer).

% moveUp, move the 0 to its top, which means moving it to (index - Side) spot
  moveUp(State, Answer, ZeroIndex, SideLength) :- Temp is (ZeroIndex - SideLength), swapEle(State, ZeroIndex, Temp, Answer).

% moveDown, move the 0 to its below, which means moving it to (index + Side) spot
  moveDown(State, Answer, ZeroIndex, SideLength) :- Temp is (ZeroIndex + SideLength), swapEle(State, ZeroIndex, Temp, Answer).



% Swap two elements(I,J) in the list, according to the index(starting at 0), output in RESULT, adapted from online code
  swapEle(S,I,J,RESULT) :-
  	same_length(S,RESULT),
  	length(BeforeI, I),
  	length(BeforeJ, J),
  	append(BeforeI, [EI|AfterI], S),
  	append(BeforeI, [EJ|AfterI], Temp),
  	append(BeforeJ, [EJ|AfterJ], Temp),
	append(BeforeJ, [EI|AfterJ], RESULT).

/* ------------------------------------------------------- */


/* equality(+S1, +S2)

   holds if and only S1 and S2 describe the same state
*/

  equality(State1, State2) :- equalityHelper(State1, State2).

% Helper method: recursively check if the first elements are the same, base case is when both lists are empty(Implys that they have the same size)
  equalityHelper([],[]).
  equalityHelper([H1|T1],[H2|T2]) :-
  	H1 is H2,
  	equalityHelper(T1, T2).

/* ------------------------------------------------------- */

/* hfn_null( +State, -V)

   V is the null heuristic for State (=0 irrelevant of the state)
*/
	hfn_null(_State, 0).



/* hfn_misplaced( +State, -V)

   V is the number of misplaced tiles in State
*/

%  Misplaced heuristic function, call the helper methods.
	hfn_misplaced( State, V ) :- misplacedHelper(State, V, 0, 1).

%  Helper method: check if the element is same as its index(starting at 1), count all and return when it finishs
	misplacedHelper([], R, R, _).
	misplacedHelper([X|T], Return, Counts, Actual) :-
		(X =\= 0, X =\= Actual) ->
    (NewCount is (Counts + 1), Temp is (Actual + 1), misplacedHelper(T, Return, NewCount, Temp));
    Temp is (Actual + 1), misplacedHelper(T, Return, Counts, Temp).



/* hfn_manhattan( +State, -V)

   V is the sum over the manhattan distances between the current
   and the designated position of each tile
*/
% hfn_manhattan( State, C ) :-  ...

% Manhattan distance heuristic function, calculate with the assist from a helper method
  hfn_manhattan( State, C ) :- length(State, L), Side is round(sqrt(L)), manhattanHelper(State, C, 0, 0, Side).

% Helper method: using divmod to find the corresponding row and column: the quotient is the row it is in, remainder is the column
  manhattanHelper([], Return, Return, _, _).
  manhattanHelper([X|T], Return, Count, Pos, Side) :-
  		(Num = (Pos + 1), X =\= 0, X =\= Num) ->
      (CoresPos is X - 1, divmod(CoresPos, Side, ItsRow, ItsCol), divmod(Pos, Side, AcRow, AcCol), RowD is abs(ItsRow - AcRow), ColD is abs(ItsCol - AcCol),
      Temp is (RowD + ColD), NewCount is Count + Temp, NewPos is Pos + 1, manhattanHelper(T, Return, NewCount, NewPos, Side));
      NewPos is Pos + 1, manhattanHelper(T, Return, Count, NewPos, Side).

/* ------------------------------------------------------- */


/* init( +Name, -State)

   State is the initial state for problem Name
*/
% Using a list to represent the states, going from top left, from left to right, from top to bottom
	init(a,	[1,2,3,4,8,5,0,7,6]).

	init(b,	[8,2,6,4,1,5,0,7,3]).

	init(c,	[0,2,6,4,1,5,8,7,3]).

	init(d,	[1,2,3,4,5,6,7,8,9,10,0,15,13,12,11,14]).


/* ------------------------------------------------------- */

/* goal( +State )

   holds if and only if State is a goal state

   check if the first element is 1,
   then check if the size is a perfect square,
   at last check if the difference between 2 consective elements is 1, and end with 0
*/

%	Goal: First element is 1, size is perfect square, each consective elements is off by 1, and end with 0
	goal(S) :- first(1, S), length(S, L), perfectSquare(L, 0), goalHelper(S).

%	Helper methods for checking the above conditions: first is 1, size is perfect square, offsets are 1
	first(M, [F|_]) :- M is F.

	perfectSquare(N, I) :- N =:= I * I.
	perfectSquare(N, I) :- N > I * I, J is I + 1, perfectSquare(N, J).
	perfectSquare(N, I) :- N < I * I, fail.

	goalHelper([_,0]).
	goalHelper([X,Y|T]) :- Y is X + 1, goalHelper([Y|T]).

/* ------------------------------------------------------- */






/** ---------------------------------------------------------
  calling the search algorithms
  ---------------------------------------------------------- */

go(ProblemName, HFN) :-
	init(ProblemName, Init),
	astar(Init, successors, goal, HFN, Path, equality),
	writeln(Path).

goCC(ProblemName, HFN) :-
	init(ProblemName, Init),
	astarCC(Init, successors, goal, HFN, Path, equality),
	writeln(Path).

goIDA(ProblemName, HFN) :-
	init(ProblemName, Init),
	idastar(Init, successors, goal, HFN, Path, equality),
	writeln(Path).
