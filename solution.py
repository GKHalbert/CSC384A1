#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are stored.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    result = 0
    for box in state.boxes:
      shortest = float('inf')
      if box not in state.storage:
        for storage in state.storage:
          distance = abs(box[0]-storage[0]) + abs(box[1]-storage[1])
          if distance < shortest:
            shortest = distance
        result += shortest
    return result

#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count

def check_wall(state,coord):
  return (coord[0] < 0) or (coord[0] >= state.width) or (coord[1] < 0) or (coord[1] >= state.height)

def check_invalid(state,coord):
  '''A helper function checks if given coordinate is out of bound or obstacles'''
  return coord in state.obstacles or check_wall(state,coord)
def check_box(state, coord):
  '''A helper function checks if given coordinate is a box'''
  return coord in state.boxes

def check_deadlock(state, box, storage_x, storage_y):
  '''A helper function checks if a box cannot reach any storage'''
  top = (box[0], box[1] + 1)
  bottom = (box[0], box[1] - 1)
  left = (box[0] - 1, box[1])
  right = (box[0] + 1, box[1])
  top_invalid = check_invalid(state, top)
  bottom_invalid = check_invalid(state, bottom)
  left_invalid = check_invalid(state, left)
  right_invalid = check_invalid(state, right)

  if (top_invalid or bottom_invalid) and (left_invalid or right_invalid):
    return True

  # check if there are consective boxes against wall or obstacles
  if check_box(state, top):
    top_left = (top[0] - 1, top[1])
    top_right =(top[0] + 1, top[1])
    if (left_invalid and check_invalid(state, top_left)) or (right_invalid and check_invalid(state, top_right)):
      return True

  if check_box(state, bottom):
    bottom_left = (bottom[0] - 1, bottom[1])
    bottom_right = (bottom[0] + 1, bottom[1])
    if (left_invalid and check_invalid(state, bottom_left)) or (right_invalid and check_invalid(state, bottom_right)):
      return True

  if check_box(state, left):
    left_top = (left[0], left[1] + 1)
    left_bottom =(left[0], left[1] - 1)
    if (top_invalid and check_invalid(state, left_top)) or (bottom_invalid and check_invalid(state, left_bottom)):
      return True

  if check_box(state, right):
    right_top = (right[0], right[1] + 1)
    right_bottom =(right[0], right[1] - 1)
    if (top_invalid and check_invalid(state, right_top)) or (bottom_invalid and check_invalid(state, right_bottom)):
      return True

  # check if a box is against the wall and there is no storage point along the wall
  if check_wall(state, top) or check_wall(state, bottom):
    if box[1] not in storage_y:
      return True

  if check_wall(state, left) or check_wall(state, right):
    if box[0] not in storage_x:
      return True

  
  return False

prevdis = None
def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    global prevdis
    result = 0

    if (state.parent is not None) and state.parent.boxes == state.boxes:
      if prevdis == float('inf'):
        return float('inf')
      result = prevdis
      for box in state.boxes:
        if box not in state.storage:
          robot_distance = [abs(box[0]-robot[0]) + abs(box[1]-robot[1]) for robot in state.robots]
          result += min(robot_distance)
      return result

    prevdis = 0
    storage_x = [storage[0] for storage in state.storage] 
    storage_y = [storage[1] for storage in state.storage]
    for box in state.boxes:
      if box not in state.storage:     
        if check_deadlock(state, box, storage_x, storage_y): #box is trapped in deadlock
          prevdis = float('inf')
          return float('inf')

        storage_distance = [abs(box[0]-storage[0]) + abs(box[1]-storage[1]) for storage in state.storage]
        result += min(storage_distance)
        prevdis += min(storage_distance)
       
        robot_distance = [abs(box[0]-robot[0]) + abs(box[1]-robot[1]) for robot in state.robots]
        result += min(robot_distance)

    return result
 
def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
  
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  # weight = 3
  wrapped_fval_function = (lambda sN: fval_function(sN, weight))
  se = SearchEngine('custom', 'default')
  se.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)

  final_result = False

  costbound = (float('inf'),float('inf'),float('inf'))

  time = os.times()[0]
  endtime = time + timebound
  while time < endtime:
    search_result = se.search(endtime-time, costbound)

    if search_result:
      final_result = search_result
      costbound = (float('inf'),float('inf'),search_result.gval + heur_fn(search_result))
      time = os.times()[0]
    else:
      return final_result
    if weight > 1:
      weight -= 0.1

  return final_result

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''

  se = SearchEngine('best_first', 'default')
  se.init_search(initial_state, sokoban_goal_state, heur_fn)

  final_result = False

  costbound = (float('inf'),float('inf'),float('inf'))

  time = os.times()[0]
  endtime = time + timebound
  while time < endtime:
    search_result = se.search(endtime-time, costbound)

    if search_result:
      final_result = search_result
      costbound = (search_result.gval,float('inf'), float('inf'))
      time = os.times()[0]
    else:
      return final_result

  return final_result
