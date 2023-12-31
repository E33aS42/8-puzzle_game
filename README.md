# 8-puzzle_game
A 8-puzzle game solver applying 2 possible search tree techniques: Depth-first search (DFS)  and Breadth-first search (BFS)

# Introduction

An instance of the N-puzzle game consists of a board holding $`N = m^2 − 1   (m = 3, 4, 5, ...)`$ distinct movable tiles, plus an empty space. The tiles are numbers from the set $`{1, …, m^2 − 1}`$. 
For any such board, the empty space may be swapped with any tile horizontally or vertically adjacent to it. The blank space is represented with the number 0 and focus on the m = 3 case (8-puzzle).
Given an initial state of the board, the combinatorial search problem is to find a sequence of moves that transitions this state to the goal state; that is, the configuration with all tiles arranged in ascending order $`⟨0, 1, …, m^2 − 1⟩`$. 
The search space is the set of all possible states reachable from the initial state.
The blank space may be swapped with a component in one of the four directions {‘Up’, ‘Down’, ‘Left’, ‘Right’}, one move at a time. The cost of moving from one configuration of the board to another is the same and equal to one. Thus, the total cost of path is equal to the number of moves made from the initial state to the goal state.
