#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Dec  6 13:41:29 2018

@author: fabiangnegel
"""

import random
import math
def writeData(file_name,TWs,distM,direcotry_name ="SSPInstances"):
    
    print "reading "+file_name

    file = open(direcotry_name+"/"+file_name, "w")

    
    file.write("%d\n"% len(TWs))
    for i in distM:
        myStr = ""
        for c,j in enumerate(i):
            if c != len(TWs)-1:
                myStr += str(j)+ " "
            else:
                myStr += str(j)
        file.write(myStr+"\n")
    for j in TWs:
        file.write("%d %d\n" % (j[0],j[1]))
    file.close()

def quickSort(alist):
   quickSortHelper(alist,0,len(alist)-1)

def quickSortHelper(alist,first,last):
   if first<last:

       splitpoint = partition(alist,first,last)

       quickSortHelper(alist,first,splitpoint-1)
       quickSortHelper(alist,splitpoint+1,last)


def partition(alist,first,last):
   pivotvalue = alist[first][1]

   leftmark = first+1
   rightmark = last

   done = False
   while not done:

       while leftmark <= rightmark and alist[leftmark][1] <= pivotvalue:
           leftmark = leftmark + 1

       while alist[rightmark][1] >= pivotvalue and rightmark >= leftmark:
           rightmark = rightmark -1

       if rightmark < leftmark:
           done = True
       else:
           temp = alist[leftmark]
           alist[leftmark] = alist[rightmark]
           alist[rightmark] = temp

   temp = alist[first]
   alist[first] = alist[rightmark]
   alist[rightmark] = temp


   return rightmark

def dist(point1,point2):
    if point1[0] > point2[0] and point1[1] > point2[1]:
        return 1000
    return math.sqrt((point1[0]-point2[0])*(point1[0]-point2[0])+(point1[1]-point2[1])*(point1[1]-point2[1]))
def dist2(point1,point2):
    if point1[0] < point2[0] and point1[1] < point2[1]:
        return 1000
    return math.sqrt((point1[0]-point2[0])*(point1[0]-point2[0])+(point1[1]-point2[1])*(point1[1]-point2[1]))

def generateInstance(fileName):
    points = [(0,0)]
    
    for i in range(798):
        points.append((300*random.random(),300*random.random()))
    points.append((1000,1000))
    distList = [[(j,dist(i,points[j])) for j in range(len(points))] for i in points]
    distList2 = [[(j,dist2(i,points[j])) for j in range(len(points))] for i in points]
    for l in distList:
        quickSort(l)
    for l in distList2:
        quickSort(l)
    successors=[[] for i in points]
    for j in range(len(points)):
        for i in range(random.randint(4,16)):
            if j !=len(points)-1:
                chosenOne = distList[j][random.randint(2,10)][0]
                if chosenOne not in successors[j]:
                    successors[j].append(chosenOne)
            if j != 0:
                chosenOne = distList2[j][random.randint(2,10)][0]
                if j not in successors[chosenOne] and chosenOne != len(points)-1:
                    successors[chosenOne].append(j)
    distMatrix = [[0 for i in points] for j in points]
    for i in range(len(points)):
        for j in successors[i]:
            distMatrix[i][j]=int(dist(points[i],points[j]))
    return distMatrix,points
             
distM,points = generateInstance("")  

TWs = []
for i,p in enumerate(points):
    if i == 0 or i == len (points)-1:
        TWs.append((0,10000))
    else:
        TWlb=int(dist((0,0),p)/5)+random.randint(0,int(dist((0,0),p)/5)+500)
        TWub=TWlb+random.randint(10,200)
        TWs.append((TWlb,TWub))
writeData("test5",TWs,distM,"SSPInstances")