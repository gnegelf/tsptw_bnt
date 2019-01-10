#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Dec  6 13:41:29 2018

@author: fabiangnegel
"""
import sys
import random
import math
def writeData(file_name,TWs,distM,costM,direcotry_name ="SSPInstances"):
    
    print "Writing "+file_name

    file = open(direcotry_name+"/"+file_name, "w")

    
    file.write("%d\n"% len(TWs))
    for row,rowDict in distM.iteritems():
        myStr = ""
        for col,val in rowDict.iteritems():
                myStr += str(col)+ " " + str(val) + " "
        if len(myStr)>0:
            myStr = myStr[:-1]
        else:
            myStr = " "
        file.write(myStr+"\n")
    for row,rowDict in costM.iteritems():
        myStr = ""
        for col,val in rowDict.iteritems():
                myStr += str(col)+ " " + str(val) + " "
        if len(myStr)>0:
            myStr = myStr[:-1]
        else:
            myStr = " "
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

def generateInstance(fileName,levels=10,gatePoints=2,pointsPerLevel=100,length = 30):
    TWs = []
    points = [(0,0)]
    adder = 0
    successors = []
    
    for k in range(levels):
        corner1 = (k*length,k*length)
        corner2 = ((k+1)*length,(k+1)*length)
        
        corner = corner2[0]
        for i in range(pointsPerLevel-gatePoints+(adder-1)):
            diagVal = corner1[0]+corner*random.random()
            diagVal2=random.random()*min(20,corner-diagVal)-min(20,corner-diagVal)/2
            points.append((diagVal+diagVal2,diagVal-diagVal2))
        
        for j in range(gatePoints):
            diagVal2=random.random()*corner-corner/2
            points.append((corner+diagVal2,corner-diagVal2))
        distList = [[(j,dist(points[i],points[j])) for j in range(pointsPerLevel*k-gatePoints*adder,(k+1)*pointsPerLevel)]
                for i in range(pointsPerLevel*k-gatePoints*adder,(k+1)*pointsPerLevel)]
        distList2 = [[(j,dist2(points[i],points[j])) for j in range(pointsPerLevel*k-gatePoints*adder,(k+1)*pointsPerLevel)]
                for i in range(pointsPerLevel*k-gatePoints*adder,(k+1)*pointsPerLevel)]
        for l in distList:
            quickSort(l)
        for l in distList2:
            quickSort(l)
        successors +=[[] for i in range(pointsPerLevel*k-gatePoints*adder,(k+1)*pointsPerLevel)]
        for j in range(pointsPerLevel*k-gatePoints*adder,(k+1)*pointsPerLevel):
            for i in range(random.randint(4,18)):
                if j not in range(pointsPerLevel*(k+1)-gatePoints,pointsPerLevel*(k+1)):
                    chosenOne = distList[j-(pointsPerLevel*k-gatePoints*adder)][random.randint(1,13)][0]
                    if chosenOne not in successors[j]:
                        successors[j].append(chosenOne)
                if j not in range(pointsPerLevel*k-gatePoints*adder,pointsPerLevel*k+(1-adder)):
                    chosenOne = distList2[j-(pointsPerLevel*k-gatePoints*adder)][random.randint(gatePoints,gatePoints+8)][0]
                    if j not in successors[chosenOne] and chosenOne != len(points)-1:
                        successors[chosenOne].append(j)
        adder=1
    distMatrix = {j:{} for j,p in enumerate(points)}
    costMatrix = {j:{} for j,p in enumerate(points)}
    for i in range(len(points)):
        for j in successors[i]:
            distMatrix[i][j] = 1+int(dist(points[i],points[j]))
            costMatrix[i][j] = 1+int((0.5+random.random()/2.0)*distMatrix[i][j])
            costMatrix[i][j] = distMatrix[i][j]
    gateIndices = [p for k in range(levels) for p in range((k+1)*pointsPerLevel-gatePoints,(k+1)*pointsPerLevel)]
    twlbadder = 0
    oldcorner=(0,0)
    for i,p in enumerate(points):
        if i == 0 or i == len (points)-1:
            TWs.append([0,7000])
        else:
            if i not in gateIndices:
                TWlb=twlbadder+random.randint(0,int(dist(oldcorner,p)*1.5))+random.randint(0,length)
                TWub=TWlb+random.randint(10,length+30)
                TWs.append([TWlb,TWub])
            else:
                TWlb=int(dist((0,0),p)*1.5)+random.randint(0,30)
                TWub=TWlb+random.randint(10,length+60)
                TWs.append([TWlb,TWub])
                twlbadder=int(dist((0,0),p)*1.5)
                oldcorner=p
    return distMatrix,points,successors,costMatrix,distList,TWs

sys.setrecursionlimit(1500)     
for I in range (0,5):      
    distM,points,successors,costM,distList,TWs = generateInstance("")  
    processed = [0]
    toBeUpdated = [i for i in successors[0]]
    while toBeUpdated != []:
        for i in toBeUpdated:
            minAtime=100000
            for j in processed:
                if i in successors[j]:
                    if TWs[j][0]+distM[j][i] <minAtime:
                        minAtime=TWs[j][0]+distM[j][i]
            if minAtime > TWs[i][0]:
                TWs[i][0] = minAtime
                if minAtime >= TWs[i][1]:
                    TWs[i][1] = minAtime+random.randint(10,150)
        processed += toBeUpdated
        NewtoBeUpdated = []
        for i in toBeUpdated:
            for j in successors[i]:
                if not j in processed and not j in NewtoBeUpdated:
                    NewtoBeUpdated.append(j)
        toBeUpdated = NewtoBeUpdated
        #print "%d items processed, %d are open" % (len(processed),len(toBeUpdated))
           
    #TWs[-1][0]=0
    writeData("newInst_1000_%d" % I,TWs,distM,costM,"SSPInstances")