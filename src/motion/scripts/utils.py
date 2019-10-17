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

def obtain_rank(lst):
    sorted_data = [(value,i) for i,value in enumerate(lst)]
    sorted_data = sorted(sorted_data)
    ranked_list = [0]*len(lst)
    for i,(_,ind) in enumerate(sorted_data):
        ranked_list[ind] = i
    return ranked_list
