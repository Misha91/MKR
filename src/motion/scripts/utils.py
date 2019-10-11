#!/usr/bin/env python

"""The function for inserting a node into sorted list"""
def insort(list, node, lo = 0, hi= None):
    if lo < 0:
        raise Exception("Negative value of lower bound")
    if hi == None:
        hi = len(list)
    while (lo<hi):
        mid = (lo + hi)//2
        if list[mid][0] > node[0]:
            hi = mid
        else: lo = mid+1
    list.insert(lo, node)
    return list

def manhattan_dist(start, goal):
    dist = abs(goal[0]-start[0])+abs(goal[1]-start[1])
    return dist

#test insort
"""
list = [1,2,3,5,6,7]
insort(list,6.5)
print(list)
"""
# test data sort
"""
node1 =[5,3.0,0.0]
node2 = [6,1.0,1.0]
node3 = [2, 1.5,2.5]

data = [node1, node2, node3]
print(data)
data.sort(key = lambda r:r[0])
print(data)
"""
"""
list = []
insort(list,1)
print(list)
"""
