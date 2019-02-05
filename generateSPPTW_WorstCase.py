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
def myPow(a,b):
    if b<0:
        return 0
    else:
        return pow(a,b)
def dist(point1,point2):
    if point1[0] > point2[0] and point1[1] > point2[1]:
        return 1000
    return math.sqrt((point1[0]-point2[0])*(point1[0]-point2[0])+(point1[1]-point2[1])*(point1[1]-point2[1]))
def dist2(point1,point2):
    if point1[0] < point2[0] and point1[1] < point2[1]:
        return 1000
    return math.sqrt((point1[0]-point2[0])*(point1[0]-point2[0])+(point1[1]-point2[1])*(point1[1]-point2[1]))

def generateInstance(fileName,n):
    costM = {j:{i:myPow(2,j-1) for i in range(j+1,n)} for j in range(0,n)}
    costM[n-1]={n:1}
    distM = {j:{i:i-j+sum([myPow(2,k) for k in range(j,i-1)]) for i in range(j+1,n)} for j in range(0,n)}
    distM[n-1]={n:1}
    TWs = [[0,i+sum([myPow(2,k) for k in range(0,i-1)])] for i in range(0,n)]+[[0,n]]
    costM[n]={}
    distM[n]={}
    return distM,costM,TWs

for n in range(3,13):
    distM,costM,TWs = generateInstance("",n)  
    
    writeData("worstCase_%d" % n,TWs,distM,costM,"SPPTW_worst_case_instances")