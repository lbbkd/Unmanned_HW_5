from itertools import permutations
import numpy as np
start = (0,0)
cost_list = []
perms = list()
perm = list(permutations([(0,0),(2,2),(5,3),(3,4),(6,4)]))
#compute distance
for i in perm:
    if i[0] == (0,0):
        perms.append(i)
#print("number of combinations is", len(list(perms)))
#for i in list(perms)
c = 0
for i in perms: 
    cost = 0
    for k in range(4):
        cost = cost + np.sqrt((i[k][0]-i[k+1][0])**2 + (i[k][1] + i[k+1][1])^2)
    cost_list.append(cost)
    c = c + 1    
    print(cost)
chosen_one = cost_list.index(min(cost_list))
chosen_perm = perms[chosen_one]
chosen_cost = cost_list[chosen_one]
print('the fastest route is',chosen_perm, 'with a cost of',chosen_cost)