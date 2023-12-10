#!/usr/bin/env python3
import sys
import time

# Launching the goal with the following command line: 
# python driver.py [choose dfs, bfs or ast] [initial board] 
# ex: > python driver.py bfs 8,7,6,5,0,4,3,2,1

start_time = time.process_time()


cases = [['Down', 'Right'], ['Down', 'Left', 'Right'], ['Down', 'Left'],
			['Up', 'Down', 'Right'], ['Up', 'Down', 'Left', 'Right'], ['Up', 'Down', 'Left'],
			['Up', 'Right'], ['Up', 'Left', 'Right'], ['Up', 'Left']]

goal = '012345678'


class State:
	# get possible children from current state
	def __init__(self):
		pass

	def get_children(self, parent):
		ind0 = -1
		children = []
		directions = []
		# get directions from parent
		for i in range(len(parent)):
			if parent[i] == '0' and ind0 == -1:
				ind0 = i
				directions = cases[ind0]
			elif parent[i] == '0' and not ind0 == -1:
				print('Error: More than one cell has a 0')
				break
				# raise exception if more than one 0

		# get children from directions
		for d in directions:
			# raise exception if ind0 = -1
			if ind0 == -1:
				print('Error: No cell has a 0')
				break
			i = ind0
			child = parent
			if d == 'Up':
				child = child[:i-3] + '0' + child[i-2:i] + parent[i - 3] + child[i+1:]
				children.append(child)
			elif d == 'Down':
				child = child[:i] + parent[i + 3] + child[i+1:i+3] + '0' + child[i+4:]
				children.append(child)
			elif d == 'Left':
				child = child[:i-1] + '0'+ parent[i - 1]  + child[i+1:]
				children.append(child)
			elif d == 'Right':
				child = child[:i] + parent[i + 1] + '0' + child[i+2:]
				children.append(child)

		current_state = (parent, children, directions)
		return current_state

class Path:
	def __init__(self):
		pass

	def find_path(self, tree, init):
		path_to_goal = []
		current_node = goal

		for i in range(len(tree)-1,-1, -1):
			eltern = tree[i][0]
			kind = tree[i][1]
			richtung = tree[i][2]
			if kind == current_node:
				path_to_goal.append(richtung)
				current_node = eltern[:]
				if eltern == init:
					break

		return path_to_goal


class Heuristic:
	def __init__(self, board):
		self.board = board
		self.count = ((0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (1, 2), (2, 0), (2, 1), (2, 2))

	def get_heuristic(self):
		h = 0
		for i in range(len(self.board)):
			b = int(self.board[i])
			g = int(goal[i])
			if b == 0:
				continue
			h += abs(self.count[g][0]-self.count[b][0])+abs(self.count[g][1]-self.count[b][1])
		return h


class Output:

	def __init__(self, path, cost, nodes, depth, max_depth, zeit, ram):
		self.path = path
		self.cost = cost
		self.nodes = nodes
		self.depth = depth
		self.max_depth = max_depth
		self.zeit = zeit
		self.ram = ram

	def print_output(self):
		out = open('output.txt', 'w')
		out.write('path_to_goal: %s \n' % self.path)
		out.write('cost_of_path: %s \n' % self.cost)
		out.write('nodes_expanded: %s \n' % self.nodes)
		out.write('search_depth: %s \n' % self.depth)
		out.write('max_search_depth: %s \n' % self.max_depth)
		out.write('running_time: %s \n' % self.zeit)
		out.write('max_ram_usage: %s \n' % self.ram)
		out.close()


class Method:
	init_board = list(sys.argv[2])
	for _ in range(init_board.count(',')): init_board.remove(',')  # remove all ',' instances from the board list
	init_board = map(int, init_board)  # convert list of strings to list of integers
	init_board = ''.join(str(i) for i in init_board)

	def __init__(self):
		pass


### Breadth-First search method ###
	def bfs(self, init_board):
		# initial state
		frontier =[]
		explored = set()        # explored states
		tree = []
		nodes = 0
		parent = init_board
		frontier.append(parent)

		# loop
		while frontier:    # check if frontier is not empty
			explored.add(parent)
			if parent == goal:
				print("Success!")
				break

			else:
				current_state = State().get_children(parent)  # current_state = [parent, children, directions]
				kinder = current_state[1]
				richtungen = current_state[2]
				i = 0
				k = 0
				for kind in kinder:
					k = 1
					if kind not in explored and kind not in frontier:
						frontier.append(kind)
						tree.append((parent, kind, richtungen[i]))
					i += 1
				nodes += k
				frontier.pop(0)
				parent = frontier[0]

		# Find path
		path_to_goal = Path().find_path(tree, init_board)[::-1]
		cost_of_path = len(path_to_goal)
		nodes_expanded = nodes
		search_depth = cost_of_path
		max_search_depth = search_depth + 1
		running_time = time.process_time() - start_time
		ram = 'tbd'

		print(' running_time: %s s' % running_time)
		print(' nodes: %s' % nodes)

		print(' path_to_goal: %s' %path_to_goal)

		Output(path_to_goal, cost_of_path, nodes_expanded, search_depth, max_search_depth, running_time, ram).\
			print_output()


# Depth First search method
	def dfs(self, init_board):
		# initial state
		stack =[]
		explored = set()        # explored states
		tree = []
		nodes = 0
		parent = init_board
		stack.append(parent)


		while stack:  # check if stack is not empty
			explored.add(parent)
			if parent == goal:
				print("Success!")
				break

			else:
				stack.pop()
				current_state = State().get_children(parent)  # current_state = [parent, children, directions]
				kinder = current_state[1][::-1]
				richtungen = current_state[2][::-1]
				i = 0
				k = 0
				for kind in kinder:
					k = 1
					if kind not in explored and kind not in stack:
						stack.append(kind)
						tree.append((parent, kind, richtungen[i]))
					i += 1
				nodes += k
				parent = stack[-1]

		# Find path
		path_to_goal = Path().find_path(tree, init_board)[::-1]
		cost_of_path = len(path_to_goal)
		nodes_expanded = nodes
		search_depth = cost_of_path
		max_search_depth = search_depth
		running_time = time.process_time() - start_time
		ram = 'tbd'

		print(' running_time: %s s' % running_time)
		print('cost_of_path: %s' % cost_of_path)
		print(' nodes: %s' % nodes)

		Output(path_to_goal, cost_of_path, nodes_expanded, search_depth, max_search_depth, running_time, ram). \
			print_output()


# A-star search
	def ast(self, init_board):
		# initial state
		heap = {}
		fringe = []
		explored = set()  # explored states
		tree = []
		nodes = 0
		parent = init_board
		heur = Heuristic(parent).get_heuristic()
		fringe.append(parent)
		heap[parent] = heur
		gFunc = 0

		while heap:  # check if stack is not empty
			explored.add(parent)

			if parent == goal:
				print("Success!")
				break

			else:
				current_state = State().get_children(parent)
				kinder = current_state[1]
				richtungen = current_state[2]
				i = 0
				k = 0
				gFunc += 1
				for kind in kinder:
					k = 1
					if kind not in explored and kind not in heap.keys():
						heur = Heuristic(kind).get_heuristic()
						fFunc = heur + gFunc
						heap[kind] = fFunc
						k = 1
						tree.append((parent, kind, richtungen[i], fFunc))
					i += 1
				nodes += k
				mini = min(heap.values())
				result = [key for key, value in heap.items() if value == mini][::-1]
				parent = result[0]
				del heap[result[0]]



				# Find path
		path_to_goal = Path().find_path(tree, init_board)[::-1]
		cost_of_path = len(path_to_goal)
		nodes_expanded = nodes
		search_depth = cost_of_path
		max_search_depth = search_depth
		running_time = time.process_time() - start_time
		ram = 'tbd'

		print(' running_time: %s s' % running_time)
		print(' nodes: %s' % nodes)

		print(' path_to_goal: %s' % path_to_goal)

		Output(path_to_goal, cost_of_path, nodes_expanded, search_depth, max_search_depth, running_time, ram). \
			print_output()

	method_names = {'bfs': bfs, 'dfs': dfs, 'ast': ast}
	meth = method_names[sys.argv[1]]

if __name__ == "__main__":
	res = Method().meth(Method.init_board)
