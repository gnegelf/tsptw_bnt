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
    return math.sqrt((point1[0]-point2[0])*(point1[0]-point2[0])+(point1[1]-point2[1])*(point1[1]-point2[1]))
def generateInstance(fileName,levels=200,pointsPerLevel=10,width=5.0,length = 100.0,arcsPerPoint=10):
    if pointsPerLevel < arcsPerPoint:
        print "points per level should be greater than the number of arcs per point"
        return 0
    TWs = []
    points = [(0,0)]
    shifter = 1
    successors = []
    length=float(length)
    for k in range(1,levels):
        yVal=k*length/levels
        points.append((0,yVal))
        for i in range(pointsPerLevel-1):
            xVal = width*random.random()
            points.append((xVal,yVal))
        if shifter > 1:
            distList = [[(j,dist(points[i],points[j])) for j in range(shifter,shifter+pointsPerLevel)]
                    for i in range(shifter-pointsPerLevel,shifter)]
            for l in distList:
                quickSort(l)
            successors +=[[] for i in range(pointsPerLevel)]
            for j in range(shifter-pointsPerLevel,shifter):
                for i in range(arcsPerPoint):
                        chosenOne = distList[j+pointsPerLevel-shifter][i][0]
                        successors[j].append(chosenOne)
        else:
            successors.append([i for i in range(shifter,shifter+pointsPerLevel)])
        shifter += pointsPerLevel
    points.append((0,length))
    successors +=[[] for i in range(pointsPerLevel)]
    for j in range(shifter-pointsPerLevel,shifter):
        for i in range(arcsPerPoint):
            successors[j].append(len(points)-1)
    successors.append([])
    distMatrix = {j:{} for j,p in enumerate(points)}
    costMatrix = {j:{} for j,p in enumerate(points)}
    for i in range(len(points)):
        for j in successors[i]:
            distMatrix[i][j] = int(dist(points[i],points[j]))+1
            costMatrix[i][j] = int(100*(dist((0,0),(width,length/levels))-dist(points[i],points[j])))/50
            if costMatrix[i][j] == 0:
                costMatrix[i][j] = 1
    TWs=[]
    for i,p in enumerate(points):
        if i == 0:    
            TWs.append([0,int(length)])
        else:
            TWs.append([int(levels*points[i][1]/length+0.0001),int(levels*width+levels)])
    return distMatrix,points,successors,costMatrix,distList,TWs

sys.setrecursionlimit(1500) 
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
for i in TWs:
    if i[1]>480:
        i[1]-=480
for I in range (0,10):
    writeData("newInst_1000_%d" % I,TWs,distM,costM,"SSPInstances")
    for i in TWs:
        if i[1]>4:
            i[1]-=4