import itertools
import math

""" ---- Keep only permutations which begin with a certain condition and end with another condition ---- """
def filter_permutations(all_permutations, num_doors):
    conserved_permutations = []
    for permutation in all_permutations:
      if permutation[0] == 0 and permutation[-1] == num_doors-1: # Has to begin with door0 = (0,0) and end with door_end = (0,0)
        take = True # Condition to conserve a permutation : if take == False, permutation is not conserved

        for i in range(len(permutation)-1): # Testing if the same door appears twice (for example 1 and 2 are the same door but with opposite theta angles)
          for j in range(i, len(permutation)-1):
            if (permutation[i] - permutation[j] == 1 and permutation[i]%2==0):
              take = False
            elif(permutation[i] - permutation[j] == -1 and permutation[i]%2==1):
              take = False

        if take == True :
          conserved_permutations.append(permutation)
    return conserved_permutations


""" ---- Overall time of a given path ---- """
def global_time(OPC_time_table, permutation):
    global_time = 0
    for i in range(len(permutation)-1):
        global_time += OPC_time_table[permutation[i]][permutation[i+1]]
    return global_time


""" ---- Main algorithm for the TSP ---- """
def traveling_salesman_problem(doors, OPC_time_table):
    num_doors = len(doors)
    all_permutations = itertools.permutations(range(num_doors), int(num_doors/2)+1 ) # Generate all possible distances

    keeped_permutations =  filter_permutations(all_permutations, num_doors) # Keep only those which begin and end with initial position (0,0)
    min_time = float('inf')
    best_path = None

    for permutation in keeped_permutations: # Testing all possibilities and take the best one
        current_time = global_time(OPC_time_table, permutation)
        if current_time < min_time:
          min_time = current_time
          best_path = permutation

    return best_path, min_time