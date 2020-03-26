#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 10:16:06 2020

@author: fabiangnegel
"""
import functools
import time
import re
import cplex

def floatEqual(f1,f2,tol=0.001):
    if abs(f1-f2)<tol:
        return 1
    else:
        return 0

def nodeIdEq(tup1,tup2):
    if tup1[0] == tup2[0] and floatEqual(tup1[1],tup2[1]) and floatEqual(tup1[2],tup2[2]):
        return 1
    else:
        return 0

class Analyser():
    def __init__(self):
        self.functionTimes = {}
        self.functionCalls = {}
        self.errorMessages = []
        self.warningMessages = []
    def recordError(self,msg):
        print(msg)
        self.errorMessages.append(msg)
    def recordWarning(self,msg):
        print(msg)
        self.warningMessages.append(msg)
myAnalyser = Analyser()
eps = 0.0001
bigNum = 1000000


def registerTime(analyser=Analyser()):
    def registerTimeDecorator(func):
        @functools.wraps(func)
        def wrapper_decorator(*args, **kwargs):
            t0 = time.time()
            value = func(*args, **kwargs)
            if func.__name__ in analyser.functionTimes:
                analyser.functionTimes[func.__name__] += time.time()-t0
            else:
                analyser.functionTimes[func.__name__] = time.time()-t0
            return value
        return wrapper_decorator
    return registerTimeDecorator

def registerCalls(analyser=Analyser()):
    def registerCallsDecorator(func):
        @functools.wraps(func)
        def wrapper_decorator(*args, **kwargs):
            value = func(*args, **kwargs)
            if func.__name__ in analyser.functionCalls:
                analyser.functionCalls[func.__name__] += 1
            else:
                analyser.functionCalls[func.__name__] = 1
            return value
        return wrapper_decorator
    return registerCallsDecorator

def arcLength(time,batteryLevel,headTimeWindow,headBatteryWindow,travelTime,batteryDelta1,batteryDelta2,loadFactor,batteryCapacity):
    tr_bat = batteryDelta1 + batteryDelta2
    if  batteryDelta1+batteryLevel < -2*eps:
        print( "Error battery level too low to return to depot")
    forcedLoadTime = max(headBatteryWindow[0]-(batteryLevel+tr_bat),0)/loadFactor
    forcedWaitTime = max(headTimeWindow[0]-(time+travelTime),0)
    loadTime=max(forcedLoadTime,forcedWaitTime)
    timeDelta = loadTime+travelTime
    arrivalBattery = min(batteryCapacity,batteryLevel+batteryDelta1+loadTime*loadFactor)+batteryDelta2
    arrivalTime=time+timeDelta
    cost = (max(0,arrivalTime-headTimeWindow[1]))**2
    
    return arrivalTime,arrivalBattery,cost


def readData(fileName,directoryName ="BatVRP"):
    
    print("reading "+fileName)

    file = open(directoryName+"/"+fileName, "r")
    vrp_data = file.read()
    file.close()
    
    entries = re.split("\n+", vrp_data)
    for i in range(len(entries)):
        if ' \r' in entries[i]:
            entries[i]=entries[i].replace(' \r','')
        if '\r' in entries[i]:
            entries[i]=entries[i].replace('\r','')

    depotDists = re.split(" +", entries[0])
    TWlbs = re.split(" +", entries[1])
    TWubs = re.split(" +",entries[2])
    profits = re.split(" +",entries[3])
    unwantedChars = ['\r','']
    listsToClean=[depotDists,TWlbs,TWubs,profits]
    for cleanMe in listsToClean:
        for charac in unwantedChars:
            if charac in cleanMe:
                cleanMe.remove(charac)
    #ToDo: Dieses korrigieren
    timeWindows = [(float(TWlbs[i]),float(TWubs[i])) for i in range(len(TWlbs))]
    depotDists = [float(i) for i in depotDists]
    profits = [float(i) for i in profits]
    return depotDists,timeWindows,profits

class VRPmodelHandler():
    def __init__(self,vrpGraph,adaptDuals):
        self.VRPgraph = vrpGraph
        self.vrp = self.VRPgraph
        self.model = cplex.Cplex()
        self.adjMatrix = vrpGraph.fullAdjMatrix
        self.arcDictionary = vrpGraph.arcDictionary
        self.adaptDuals = adaptDuals
    def fillModel(self,varType = 'C'):
        self.constNum = 0
        self.constName2idx={}
        self.idx2name = {}
        self.name2idx = {}
        self.yVarName2Arc = {}
        self.xNames = []
        self.yNames = []
        self.xNum = 0
        self.model.set_problem_type(0)
        self.model.parameters.lpmethod.set(4)
        self.model.set_results_stream(None)
        self.model.set_log_stream(None)
        #self.model.parameters.advance.set(1)
        self.model.set_warning_stream(None)
        xNames = ["x_%d_%d" %(i,j) for i in range(self.vrp.n) for j in self.vrp.fullAdjMatrix[i]]
        xObj = [0 for i in range(self.vrp.n) for j in self.vrp.fullAdjMatrix[i]]
        #yNames = [arc.getCurrentVarName()
        #                for key,arc_list in self.vrp.arcDictionary.items() for arc in arc_list]
        #yObj = [arc.cost for key,arc_list in self.vrp.arcDictionary.items() for arc in arc_list]
        self.addVariables(xNames,[varType]*len(xNames),xObj,[0.0]*len(xNames),[1.0]*len(xNames),'x')
        self.xNum=len(xNames)
        for key,arc_list in self.vrp.arcDictionary.items():
            for arc in arc_list:
                arc.createVarInModel(self)
        #self.addVariables(yNames,['C']*len(yNames),yObj,[0.0]*len(yNames),[1.0]*len(yNames),'y')
        allvars = []
        allrhs = []
        allsenses = []
        allNames = []
        
        for node in self.vrp.nodes:
            if node.name!=self.vrp.goal and node.name != self.vrp.depot:
                thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.vrp.fullAdjMatrix[node.name]]
                thecoefs = [1 for node2 in self.vrp.fullAdjMatrix[node.name]]
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("E")
                allrhs.append(1.0)
                allNames.append("visit_once_%d" % node.name)
            if node.name == self.vrp.depot:
                thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.vrp.fullAdjMatrix[node.name]]
                thecoefs = [1]*len(thevars)
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("L")
                allrhs.append(self.vrp.vehicleAmount)
                allNames.append("VehicleAmount")
                    

        for node in self.vrp.nodes:
            for interval_node in node.timeNodes:
                for bat_node in interval_node.batteryNodes:
                    if node.name != self.vrp.goal and node.name !=self.vrp.depot:
                        thevars = [arc.getCurrentVarName()
                                    for arc in bat_node.outgoingArcs]
                        
                        thecoefs = [-1]*len(bat_node.outgoingArcs)
                        
                            
                        #allvars.append(cplex.SparsePair(thevars,thecoefs))
                        #allsenses.append("E")
                        #allrhs.append(0.0)
                        #allNames.append("goout_%d_%d" %(node.name,interval_node.id))
                        
                        thevars += [arc.getCurrentVarName()
                                    for arc in bat_node.ingoingArcs]
                        
                        thecoefs += [1]*len(bat_node.ingoingArcs)
        
                            
                        allvars.append(cplex.SparsePair(thevars,thecoefs))
                        allsenses.append("E")
                        allrhs.append(0.0)
                        allNames.append("flow_" + bat_node.toStr())
        
        for i,j in self.vrp.arcDictionary:
            thevars = [arc.getCurrentVarName()
                       for arc in self.vrp.arcDictionary[i,j]]
            thecoefs = [1]*len(thevars)
            thevars += ["x_%d_%d" %(i,j)]
            thecoefs += [-1]
            allvars.append(cplex.SparsePair(thevars,thecoefs))
            allsenses.append("E")
            allrhs.append(0.0)
            allNames.append("arcuse_%d_%d" %(i,j))
        
        self.addConstraints(allNames,allvars,allsenses,allrhs)
        if varType == 'C':
            self.model.set_problem_type(0)
            self.model.parameters.lpmethod.set(4)
    def addVariables(self,names,types,objs,lbs,ubs,varType):
        oldInds=self.model.variables.get_num()
        self.model.variables.add(names = names,
                            types=types,obj=objs,
                            lb = lbs,ub = ubs)
        if varType=='x':
            self.xNames += names
        if varType=='y':
            self.yNames += names
        self.name2idx.update({name:oldInds+j for j,name in enumerate(names)})
        self.idx2name.update({oldInds+j:name for j,name in enumerate(names)})
        self.varNum = len(self.name2idx)
    def addConstraints(self,names,thevars,senses,rhs):
        oldInds=self.constNum
        self.model.linear_constraints.add(names=names,lin_expr = thevars, 
                                                     senses = senses, rhs = rhs)
        self.constName2idx.update({name:oldInds+j for j,name in enumerate(names)})
        self.constNum += len(names)
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def setName(self,idx,varName):
        self.model.variables.set_names(idx,varName)
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def changeVariableName(self,oldVarName,newVarName):
        self.model.variables.set_names(self.name2idx[oldVarName],newVarName)
        #self.setName(self.name2idx[oldVarName],newVarName)
        if newVarName[0] == 'y':
            idx = self.name2idx.pop(oldVarName)
            self.yNames[idx-len(self.xNames)]=newVarName
            self.idx2name[idx] = newVarName
            self.name2idx[newVarName] = idx
        #else:
        #    print ("Changing names of non y-variables is currently not supported :(")
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def changeConstraintCoefficient(self,constName,varName,coeff):
        self.model.linear_constraints.set_coefficients([(self.constName2idx[constName],self.name2idx[varName],coeff)]) 
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def solveLpRelaxation(self,node,tol=0.00001):
        vrp = self.vrp
        model = self.model
        oldLen = len(node.branches)
        
        model.linear_constraints.add(names = node.branchNames,lin_expr = node.branchLinExprs,senses = node.branchSenses,rhs = node.branchRhs)
        if vrp.modelHandler.adaptDuals and node.dualValuesModel != 0:
            #model.parameters.advance.set(1)
            model.start.set_start(col_status=[],row_status=[],
                                  row_dual=node.dualValuesModel+node.dualValuesBranches,col_primal=[],row_primal=[],col_dual=[])
        self.model.set_problem_type(0)
        self.model.parameters.lpmethod.set(4)
        feas = vrp.modelHandler.solveModel()
        
        #model.parameters.preprocessing.presolve.set(0)
        
        node.updatedLpRelaxation = 1
        #model.parameters.advance.set(0)
        if not feas:
            node.feasible = 0
            node.lowerBound = 1000000
            model.linear_constraints.delete(range(model.linear_constraints.get_num()-oldLen,model.linear_constraints.get_num()))
            return 0
        else:
            solution = model.solution
            if vrp.modelHandler.adaptDuals:
                dualValues = solution.get_dual_values()
            else:
                dualValues = []
            node.dualValuesModel = dualValues[:self.constNum]
            node.dualValuesBranches = dualValues[self.constNum:]
            node.primalValues = solution.get_values()
            node.lowerBound = solution.get_objective_value()
            
            #node.primalXValues = {name:node.primalValues[self.name2idx[name]] for name in self.xNames 
            #                        if node.primalValues[self.name2idx[name]]>tol}
            node.primalXValues = { name:val for name,val in zip(self.xNames,node.primalValues[0:self.xNum])
                        if node.primalValues[self.name2idx[name]]>tol }
            node.fractionals = {key:val for key,val in node.primalXValues.items() if abs(0.5-val)<0.5-tol}
            #if len(node.fractionals) == 0:
            node.primalYValues = {name:node.primalValues[self.name2idx[name]] for name in self.yNames
                        if node.primalValues[self.name2idx[name]]>tol}
            #else:
            #    node.primalYValues = {}
        model.linear_constraints.delete(range(model.linear_constraints.get_num()-oldLen,model.linear_constraints.get_num()))
        node.feasible = 1
        #self.tree.total_relaxation_time += time.time() - t0
        return 1
    def solveModel(self):
        self.model.solve()
        return self.model.solution.is_primal_feasible()
    def createMIPmodel(self):
        self.MIPmodel = cplex.Cplex()
        model = self.MIPmodel
        model.parameters.timelimit.set(1800)
        xNames = ["x_%d_%d" %(i,j) for i in range(self.vrp.n) for j in self.vrp.fullAdjMatrix[i]]
        xObj = [0 for i in range(self.vrp.n) for j in self.vrp.fullAdjMatrix[i]]
        
        gammaNames = ["gamma_%d" % i for i in range(self.vrp.n)]
        gammaObj = [0 for i in range(self.vrp.n)]
        
        cNames = ["c_%d_%d" %(i,j) for i in range(self.vrp.n) for j in self.vrp.fullAdjMatrix[i]]
        cObj = [0 for i in range(self.vrp.n) for j in self.vrp.fullAdjMatrix[i]]
        
        tNames = ["t_%d" % i for i in range(self.vrp.n)]
        tObj = [0 for i in range(self.vrp.n)]
        
        bNames = ["b_%d" % i for i in range(self.vrp.n)]
        bObj = [0 for i in range(self.vrp.n)]
        

        # model.variables.add(names=["m"],obj=[1],lb=[0],ub=[self.vrp.timeHorizon*self.vrp.n],types=["C"])
        model.variables.add(names=xNames,types=['B']*len(xNames),obj=xObj,lb=[0.0]*len(xNames),ub=[1.0]*len(xNames))
        
        model.variables.add(names=gammaNames,types=['C']*len(gammaNames),obj=gammaObj,
                            lb=[0.0]*len(gammaNames),ub=[self.vrp.timeHorizon]*len(gammaNames))
        
        model.variables.add(names=cNames,types=['C']*len(cNames),obj=cObj,lb=[0.0]*len(cNames),ub=
                            [self.vrp.batteryCapacity/self.vrp.loadFactor]*len(cNames))
        
        model.variables.add(names=tNames,types=['C']*len(tNames),obj=tObj,
                            lb=[node.timeWindow[0] for node in self.vrp.nodes],ub=[self.vrp.timeHorizon]*len(tNames))
        
        model.variables.add(names=bNames,types=['C']*len(bNames),obj=bObj,lb=[node.batteryInterval[0] for node in self.vrp.nodes],
                                                          ub=[node.batteryInterval[1] for node in self.vrp.nodes])
        
        allvars = []
        allrhs = []
        allsenses = []
        allNames = []
        
        for node in self.vrp.nodes:
            if node.name!=self.vrp.goal and node.name != self.vrp.depot:
                thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.vrp.fullAdjMatrix[node.name]]
                thecoefs = [1 for node2 in self.vrp.fullAdjMatrix[node.name]]
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("E")
                allrhs.append(1.0)
                allNames.append("visit_once_%d" % node.name)
                thevars = ["x_%d_%d" %(node2,node.name) for node2 in self.vrp.fullAdjMatrixTransposed[node.name]]
                thecoefs = [1 for node2 in self.vrp.fullAdjMatrixTransposed[node.name]]
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("E")
                allrhs.append(1.0)
                allNames.append("visit_once_%d" % node.name)
            if node.name == self.vrp.depot:
                thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.vrp.fullAdjMatrix[node.name]]
                thecoefs = [1]*len(thevars)
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("L")
                allrhs.append(self.vrp.vehicleAmount)
                allNames.append("VehicleAmount")
                    
        """
        for node in self.vrp.nodes:
            if node.name != self.vrp.goal and node.name != self.vrp.depot:
                thevars = ["x_%d_%d" % (node.name,nodeHead) for nodeHead in self.vrp.fullAdjMatrix[node.name]]+[
                            "x_%d_%d" % (nodeTail,node.name) for nodeTail in self.vrp.fullAdjMatrixTransposed[node.name]]
                
                thecoefs = [1 for nodeHead in self.vrp.fullAdjMatrix[node.name]]+[
                            -1 for nodeTail in self.vrp.fullAdjMatrixTransposed[node.name]]
                
                    
                #allvars.append(cplex.SparsePair(thevars,thecoefs))
                #allsenses.append("E")
                #allrhs.append(0.0)
                #allNames.append("goout_%d_%d" %(node.name,interval_node.id))
                


                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("E")
                allrhs.append(0.0)
                allNames.append("flow_%d" %node.name)
        """
        for nodeTail in self.vrp.nodes:
            for nodeHeadName in self.vrp.fullAdjMatrix[nodeTail.name]:
                thevars = ["t_%d" %nodeTail.name,"c_%d_%d" % (nodeTail.name,nodeHeadName),
                            "t_%d" %nodeHeadName,"x_%d_%d" % (nodeTail.name,nodeHeadName)]
                thecoefs = [1,1,-1,self.vrp.timeHorizon]
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("L")
                allrhs.append(self.vrp.timeHorizon-self.vrp.fullAdjMatrix[nodeTail.name][nodeHeadName])
                allNames.append("time_%d_%d" % (nodeTail.name,nodeHeadName))
                thevars = ["b_%d" %nodeTail.name,"c_%d_%d" % (nodeTail.name,nodeHeadName),
                            "b_%d" %nodeHeadName,"x_%d_%d" % (nodeTail.name,nodeHeadName)]
                thecoefs = [1,self.vrp.loadFactor,-1,-self.vrp.batteryCapacity]
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("G")
                allrhs.append(-self.vrp.batteryCapacity+self.vrp.depotDistances[nodeTail.name]+self.vrp.depotDistances[nodeHeadName])
                allNames.append("battery_%d_%d" % (nodeTail.name,nodeHeadName))
        for node in self.vrp.nodes:
            thevars = ["t_%d" %node.name,"gamma_%d" % node.name]
            thecoefs = [1,-1]
            allvars.append(cplex.SparsePair(thevars,thecoefs))
            allsenses.append("L")
            allrhs.append(node.timeWindow[1])
            allNames.append("upperTimeBound_%d" % (node.name))
        # for node in self.vrp.nodes:
        #     thevars = ["m"]+["gamma_%d"%node.name]
        #     thecoefs = [-1]+[1]
        #     allvars.append(cplex.SparsePair(thevars,thecoefs))
        #     allsenses.append("L")
        #     allrhs.append(0)
        #     allNames.append("calcmax_%d"%node.name)
        model.linear_constraints.add(lin_expr=allvars,senses=allsenses,rhs=allrhs,names=allNames)
        model.objective.set_quadratic_coefficients([("gamma_%d" % node.name, "gamma_%d"% node.name,2.0) for node in self.vrp.nodes])
    def createMIPmodel1Norm(self):
        self.MIPmodel = cplex.Cplex()
        model = self.MIPmodel
        model.parameters.timelimit.set(1800)
        xNames = ["x_%d_%d" %(i,j) for i in range(self.vrp.n) for j in self.vrp.fullAdjMatrix[i]]
        xObj = [0 for i in range(self.vrp.n) for j in self.vrp.fullAdjMatrix[i]]
        
        gammaNames = ["gamma_%d" % i for i in range(self.vrp.n)]
        gammaObj = [1 for i in range(self.vrp.n)]
        
        cNames = ["c_%d_%d" %(i,j) for i in range(self.vrp.n) for j in self.vrp.fullAdjMatrix[i]]
        cObj = [0 for i in range(self.vrp.n) for j in self.vrp.fullAdjMatrix[i]]
        
        tNames = ["t_%d" % i for i in range(self.vrp.n)]
        tObj = [0 for i in range(self.vrp.n)]
        
        bNames = ["b_%d" % i for i in range(self.vrp.n)]
        bObj = [0 for i in range(self.vrp.n)]
        

        # model.variables.add(names=["m"],obj=[1],lb=[0],ub=[self.vrp.timeHorizon*self.vrp.n],types=["C"])
        model.variables.add(names=xNames,types=['B']*len(xNames),obj=xObj,lb=[0.0]*len(xNames),ub=[1.0]*len(xNames))
        
        model.variables.add(names=gammaNames,types=['C']*len(gammaNames),obj=gammaObj,
                            lb=[0.0]*len(gammaNames),ub=[self.vrp.timeHorizon]*len(gammaNames))
        
        model.variables.add(names=cNames,types=['C']*len(cNames),obj=cObj,lb=[0.0]*len(cNames),ub=
                            [self.vrp.batteryCapacity/self.vrp.loadFactor]*len(cNames))
        
        model.variables.add(names=tNames,types=['C']*len(tNames),obj=tObj,
                            lb=[node.timeWindow[0] for node in self.vrp.nodes],ub=[self.vrp.timeHorizon]*len(tNames))
        
        model.variables.add(names=bNames,types=['C']*len(bNames),obj=bObj,lb=[node.batteryInterval[0] for node in self.vrp.nodes],
                                                          ub=[node.batteryInterval[1] for node in self.vrp.nodes])
        
        allvars = []
        allrhs = []
        allsenses = []
        allNames = []
        
        for node in self.vrp.nodes:
            if node.name!=self.vrp.goal and node.name != self.vrp.depot:
                thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.vrp.fullAdjMatrix[node.name]]
                thecoefs = [1 for node2 in self.vrp.fullAdjMatrix[node.name]]
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("E")
                allrhs.append(1.0)
                allNames.append("visit_once_%d" % node.name)
                thevars = ["x_%d_%d" %(node2,node.name) for node2 in self.vrp.fullAdjMatrixTransposed[node.name]]
                thecoefs = [1 for node2 in self.vrp.fullAdjMatrixTransposed[node.name]]
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("E")
                allrhs.append(1.0)
                allNames.append("visit_once_%d" % node.name)
            if node.name == self.vrp.depot:
                thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.vrp.fullAdjMatrix[node.name]]
                thecoefs = [1]*len(thevars)
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("L")
                allrhs.append(self.vrp.vehicleAmount)
                allNames.append("VehicleAmount")
                    
        """
        for node in self.vrp.nodes:
            if node.name != self.vrp.goal and node.name != self.vrp.depot:
                thevars = ["x_%d_%d" % (node.name,nodeHead) for nodeHead in self.vrp.fullAdjMatrix[node.name]]+[
                            "x_%d_%d" % (nodeTail,node.name) for nodeTail in self.vrp.fullAdjMatrixTransposed[node.name]]
                
                thecoefs = [1 for nodeHead in self.vrp.fullAdjMatrix[node.name]]+[
                            -1 for nodeTail in self.vrp.fullAdjMatrixTransposed[node.name]]
                
                    
                #allvars.append(cplex.SparsePair(thevars,thecoefs))
                #allsenses.append("E")
                #allrhs.append(0.0)
                #allNames.append("goout_%d_%d" %(node.name,interval_node.id))
                


                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("E")
                allrhs.append(0.0)
                allNames.append("flow_%d" %node.name)
        """
        for nodeTail in self.vrp.nodes:
            for nodeHeadName in self.vrp.fullAdjMatrix[nodeTail.name]:
                thevars = ["t_%d" %nodeTail.name,"c_%d_%d" % (nodeTail.name,nodeHeadName),
                            "t_%d" %nodeHeadName,"x_%d_%d" % (nodeTail.name,nodeHeadName)]
                thecoefs = [1,1,-1,self.vrp.timeHorizon]
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("L")
                allrhs.append(self.vrp.timeHorizon-self.vrp.fullAdjMatrix[nodeTail.name][nodeHeadName])
                allNames.append("time_%d_%d" % (nodeTail.name,nodeHeadName))
                thevars = ["b_%d" %nodeTail.name,"c_%d_%d" % (nodeTail.name,nodeHeadName),
                            "b_%d" %nodeHeadName,"x_%d_%d" % (nodeTail.name,nodeHeadName)]
                thecoefs = [1,self.vrp.loadFactor,-1,-self.vrp.batteryCapacity]
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("G")
                allrhs.append(-self.vrp.batteryCapacity+self.vrp.depotDistances[nodeTail.name]+self.vrp.depotDistances[nodeHeadName])
                allNames.append("battery_%d_%d" % (nodeTail.name,nodeHeadName))
        for node in self.vrp.nodes:
            thevars = ["t_%d" %node.name,"gamma_%d" % node.name]
            thecoefs = [1,-1]
            allvars.append(cplex.SparsePair(thevars,thecoefs))
            allsenses.append("L")
            allrhs.append(node.timeWindow[1])
            allNames.append("upperTimeBound_%d" % (node.name))
        # for node in self.vrp.nodes:
        #     thevars = ["m"]+["gamma_%d"%node.name]
        #     thecoefs = [-1]+[1]
        #     allvars.append(cplex.SparsePair(thevars,thecoefs))
        #     allsenses.append("L")
        #     allrhs.append(0)
        #     allNames.append("calcmax_%d"%node.name)
        model.linear_constraints.add(lin_expr=allvars,senses=allsenses,rhs=allrhs,names=allNames)
        model.objective.set_quadratic_coefficients([("gamma_%d" % node.name, "gamma_%d"% node.name,2.0) for node in self.vrp.nodes])
   

class VRPgraph():
    necessaryInstanceParameterList = ['batteryCapacity','loadFactor','timeHorizon','serviceTime','vehicleAmount']
    def __init__(self,depotDistances,timeWindows,profits,instanceParameters,controlParameters=[]):
        if not self.checkParameters(instanceParameters):return
        self.registerInstanceParameters(instanceParameters)
        if len(depotDistances) != len(timeWindows):
            myAnalyser.recordError("Error in VRPgraph creation: Size of given time windows and depot distances is not the same")
            return
        self.n = len(depotDistances)+2
        if len(profits)+2!=self.n:
            myAnalyser.recordError("Error in VRPgraph creation: Size of profits is wrong")
            return
        self.depotDistances = [0]+list(depotDistances)+[0]
        self.timeWindows = [(0.0,0.0)]+[(TW[0]+self.serviceTime,TW[1]+self.serviceTime) for TW in timeWindows]+[(self.timeHorizon,self.timeHorizon)]
        self.profits = [0.0] + profits + [0.0]
        self.batteryIntervals = [(dist,self.batteryCapacity-dist) for dist in self.depotDistances]
        self.fullAdjMatrix = {i:{j:self.depotDistances[i]+self.serviceTime+self.depotDistances[j] for j in range(1,self.n) if j!=i} for i in range(self.n-1)}
        self.fullAdjMatrix[self.n-1]={}
        self.fullAdjMatrix[0].pop(self.n-1)
        self.fullAdjMatrixTransposed = {i:{j:self.depotDistances[i]+self.serviceTime+self.depotDistances[j] 
                                           for j in range(self.n) if i in self.fullAdjMatrix[j]} for i in range(self.n)}
        self.vrpNodes = [VRPnode(self,i, self.timeWindows[i], self.batteryIntervals[i]) for i in range(self.n)]
        self.nodes = self.vrpNodes
        self.arcDictionary = {(i,j):[] for i in self.fullAdjMatrix for j in self.fullAdjMatrix[i] if i!=j}
        for vrpNode in self.vrpNodes[:self.n-1]:
            for timeNode in vrpNode.timeNodes:
                for tailNode in timeNode.batteryNodes:
                    for headName in self.fullAdjMatrix[vrpNode.name]:
                        if headName != vrpNode.name:
                            arrivalTime,arrivalBattery,cost = arcLength(timeNode.interval[0],
                                                                         tailNode.interval[1], 
                                                                         self.nodes[headName].timeWindow, 
                                                                         self.nodes[headName].batteryInterval, 
                                                                         self.fullAdjMatrix[vrpNode.name][headName], 
                                                                         -self.depotDistances[vrpNode.name], 
                                                                         -self.depotDistances[headName], 
                                                                         self.loadFactor, 
                                                                         self.batteryCapacity)
                            timeNodeIndex,headTimeNode = self.vrpNodes[headName].findLowestReachableTimeNode(arrivalTime)
                            batNodeIndex,headNode = headTimeNode.findLargestReachableBatteryNode(arrivalBattery)

                            Arc(tailNode,headNode,arrivalTime,arrivalBattery,cost)
        self.depot = self.vrpNodes[0].name
        self.goal = self.vrpNodes[-1].name
        self.modelHandler = VRPmodelHandler(self,1)
        self.modelHandler.fillModel('C')
    def registerInstanceParameters(self,instanceParameters):
        for paramName,paramVal in instanceParameters.items():
            self.__setattr__(paramName,paramVal)
    def checkParameters(self,instanceParameters):
        retVal = True
        for paramName in VRPgraph.necessaryInstanceParameterList:
            if paramName not in instanceParameters:
                myAnalyser.recordError(f"Error in VRPgraph creation: Parameter '{paramName}' was not included in instance parameter dictionary")
                retVal=False
        return retVal
    def findSplitPoints(self,primalYValues):#heuristic for additional split points 
        paths,shortCycles = self.solToCycles(primalYValues)
        splitPointList = []
        newSplitPointList = []
        
        if len(shortCycles) == 0:
            for path in paths:
                pathTB ,objective = self.findTimedBatteryPath(path)
                splitPointList += pathTB
            for splitPoint in splitPointList:
                lbExists,timeNode = self.nodes[splitPoint[0]].findLowerBound(splitPoint[1])
                if lbExists:
                    ubExists,batteryNode = timeNode.findUpperBound(splitPoint[2])
                    if not ubExists:
                        newSplitPointList.append(splitPoint)
                else:
                    newSplitPointList.append(splitPoint)
        else:
            for cycle in shortCycles:
                nextI=0
                for curI in range(0,len(cycle)-1):
                    nextI = curI+1
                    arrivalTime,arrivalBattery,cost = arcLength(cycle[curI][1],
                                                      cycle[curI][2],
                                                      self.nodes[cycle[nextI][0]].timeWindow,
                                                      self.nodes[cycle[nextI][0]].batteryInterval,
                                                      self.fullAdjMatrix[cycle[curI][0]][cycle[nextI][0]],
                                                      -self.depotDistances[cycle[curI][0]],
                                                      -self.depotDistances[cycle[nextI][0]],
                                                      self.loadFactor,
                                                      self.batteryCapacity)
                    
                    newSplitPointList.append((cycle[nextI][0],arrivalTime,arrivalBattery))
        newSplitPointList = [sp for sp in newSplitPointList if sp[0]!=self.goal and sp[0]!=self.depot]
        return newSplitPointList
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def solToArcCorrections(self,yStringList):
        newSplitPoints = []
        for varName in yStringList:
            arc = self.modelHandler.yVarName2Arc[varName]
            if not arc.hasCorrectLength():
                if arc.head.name != self.goal:
                    newSplitPoints.append((arc.head.name,arc.arrivalTime, arc.arrivalBattery))
        for splitPoint in newSplitPoints:
            lbExists,timeNode = self.nodes[splitPoint[0]].findLowerBound(splitPoint[1])
            if lbExists:
                ubExists,batteryNode = timeNode.findUpperBound(splitPoint[2])
                if ubExists:
                    print("ERROR arc is assumed to not have correct length, but should already exist")
        return newSplitPoints
    def findSplitPointsToBreakCycle(self,cycle):
        splitPoints = []
        if cycle[0]==cycle[-1]:
            cycle.pop(-1)
        #print( cycle)
        if cycle not in self.brokenCycleList:
            self.brokenCycleList.append(tuple(cycle))
            for startIndex in range(len(cycle)):
                arrivalBattery = self.nodes[cycle[startIndex]].batteryInterval[1]
                arrivalTime = self.nodes[cycle[startIndex]].timeWindow[0]
                curIndex = startIndex
                while (arrivalTime <= self.nodes[cycle[curIndex]].timeWindowl[1]+0.005 and arrivalBattery 
                    >= self.nodes[cycle[curIndex]].batteryInterval[0]-0.005):
                    splitPoints.append((cycle[curIndex],arrivalTime,arrivalBattery))
                    nextIndex = (curIndex+1) % (len(cycle))
                    #print cycle
                    arrivalTime,arrivalBattery,cost = arcLength(arrivalTime,
                                                      arrivalBattery,
                                                      self.nodes[cycle[nextIndex]].timeWindow,
                                                      self.nodes[cycle[nextIndex]].batteryInterval,
                                                      self.fullAdjMatrix[cycle[curIndex]][cycle[nextIndex]],
                                                      -self.depotDistances[cycle[curIndex]],
                                                      -self.depotDistances[cycle[nextIndex]],
                                                      self.loadFactor,
                                                      self.batteryCapacity)
                    curIndex = nextIndex
        else:
            "We have to correct the arcs"
        return splitPoints
    def solToCycles(self,yStringList,tol=0.001):
        arcListFloat = []
        for string in yStringList:
            arcListFloat.append([float(s) for s in re.split("[_]",string)[1:]])
        oS = [[(int(arc[0]),arc[1],arc[2]),(int(arc[3]),arc[4],arc[5])] for arc in arcListFloat]
        S = [[(self.nodes[0].name,self.nodes[0].timeNodes[0].interval[0],self.nodes[0].timeNodes[0].batteryNodes[-1].interval[1]),
              ] for i in range(self.vehicleAmount)
            ]
        insertPos=[1 for i in range(self.vehicleAmount)]
        #tailPos=[1 for i in range(self.vehicle_amount)]
        appendedArcL = 0
        insertedCustomers = [set() for i in range(self.vehicleAmount)]
        while appendedArcL<len(oS):
            arcL = oS.pop(0)
            tail = arcL[0]
            head = arcL[-1]
            added = 0
            for i in range(self.vehicleAmount):
                if nodeIdEq(tail,S[i][insertPos[i]-1]):
                    arcL.pop(0)
                    S[i][insertPos[i]:insertPos[i]] = arcL
                    appendedArcL = 0
                    insertPos[i] += len(arcL)
                    for tup in arcL:
                        insertedCustomers[i].add(tup[0])
                    added = 1
                    break
            if not added:
                for i in range(len(oS)):
                    if nodeIdEq(tail,oS[i][-1]):
                        arcL.pop(0)
                        oS[i].extend(arcL)
                        appendedArcL = 0
                        added = 1
                        break 
                    elif nodeIdEq(head,oS[i][0]):
                        arcL.pop(-1)
                        oS[i][0:0] = arcL
                        appendedArcL = 0
                        added = 1
                        break
            if not added:
                oS.append(arcL)
                appendedArcL += 1
        for k in range(self.vehicleAmount):
            if len(insertedCustomers[k]) < len(S[k])-2:
                path = S[k]
                cycleFound = 0
                for i,node in enumerate(path):
                    for j in range(i+1,len(path)):
                        if path[j][0] == node[0]:
                            cycleFound = 1
                            low=i
                            high=j
                            break
                    if cycleFound:
                        break
                if cycleFound:
                    oS.append(path[low:high+1])
                    S[k] = path[0:low]+path[high+1:len(path)]
                else:
                    myAnalyser.recordError("Error customer tracking predicted cycle, but no cycle found")
        return S,oS
    def findTimedBatteryPath(self,path):
        pathTimeBattery = []
        arrivalBattery = self.batteryCapacity
        arrivalTime = 0.0
        objective = 0.0
        for i in range(len(path)-1):
            arrivalTime,arrivalBattery,cost = arcLength(arrivalTime,
                                                  arrivalBattery,
                                                  self.nodes[path[i+1][0]].timeWindow,
                                                  self.nodes[path[i+1][0]].batteryInterval,
                                                  self.fullAdjMatrix[path[i][0]][path[i+1][0]],
                                                  -self.depotDistances[path[i][0]],
                                                  -self.depotDistances[path[i+1][0]],
                                                  self.loadFactor,
                                                  self.batteryCapacity)
            objective += cost
            pathTimeBattery.append((path[i+1][0],arrivalTime,arrivalBattery))
        return pathTimeBattery,objective

class VRPnode():
    def __init__(self,vrp,name,timeWindow,batteryInterval):
        self.vrp = vrp
        self.name = name
        self.timeWindow = tuple(timeWindow)
        self.batteryInterval = tuple(batteryInterval)
        self.timeNodes = [TimeNode(self,timeWindow,True,True,batteryInterval=batteryInterval,firstPredecessor=0)]
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def findLowestReachableTimeNode(self,arrivalTime):
        if arrivalTime < self.timeWindow[0]-eps:
            myAnalyser.recordWarning("Warning while looking for time node: Arrival time was less than time window")
            return 0,self.timeNodes[0]
        for i,timeNode in enumerate(reversed(self.timeNodes)):
            if timeNode.isContained(arrivalTime):
                return len(self.timeNodes)-1-i,timeNode
        return -1,self.timeNodes[-1]
    def findLowerBound(self,time):
        for timeNode in self.timeNodes:
            if timeNode.isLowerBound(time): return True,timeNode
        return False,0
    #first return value is the position of the timenode the splitpoint was contained in, the second is the shift to the newly added timenode
    # So if the second value is 0 nothing was changed
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def splitTimeNodes(self,splitPoint,isLb=True,isUb=False):
        if isLb and isUb:
            myAnalyser.recordError(f"Error while splitting nodes: '{splitPoint}' Splitpoint cannot be both lower and upper bound")
            return
        if splitPoint < self.timeWindow[0]-eps:
            myAnalyser.recordError(f"Error while splitting nodes: '{splitPoint}' Splitpoint cannot bew lower than left hand side of interval")
            return
        oldIdx,oldTimeNode = self.findLowestReachableTimeNode(splitPoint)
        if oldTimeNode.includesLb and abs(splitPoint-oldTimeNode.interval[0])<eps:
            #myAnalyser.recordError(f"Error while splitting nodes: '{splitPoint}' was already used as lower bound")
            return oldIdx,0
        if oldIdx == -1:
            oldIdx = len(self.timeNodes)-1
            if splitPoint < self.timeWindow[1]+eps or 1:
                inheritedBatteryNodes= [node for node in oldTimeNode.batteryNodes 
                                        if (not abs(node.interval[1] - self.batteryInterval[1])<eps)]
            else:
                inheritedBatteryNodes= [oldTimeNode.batteryNodes[0]]
            self.timeNodes.append(TimeNode(self, (splitPoint,splitPoint),1,1,
                                   batteryInterval = self.batteryInterval,
                                   inheritedBatteryNodes=inheritedBatteryNodes,firstPredecessor=oldTimeNode.batteryNodes[-1]))
            """
            self.timeNodes.append(TimeNode(self, (oldTimeNode.interval[1],splitPoint),1,1,
                                   batteryInterval = self.batteryInterval,
                                   inheritedBatteryNodes=inheritedBatteryNodes,firstPredecessor=oldTimeNode.batteryNodes[-1]))
            """
            oldTimeNode.interval[1] = splitPoint
            oldTimeNode.includesUb = 0
            return oldIdx,1
        if isLb:
            if splitPoint < self.timeWindow[1]+eps or 1:
                inheritedBatteryNodes= [node for node in oldTimeNode.batteryNodes 
                                        if (not abs(node.interval[1] - self.batteryInterval[1])<eps)]
            else:
                inheritedBatteryNodes= [oldTimeNode.batteryNodes[0]]
            newTimeNode = TimeNode(self, (splitPoint,oldTimeNode.interval[1]),isLb,oldTimeNode.includesUb,
                                   batteryInterval = self.batteryInterval,
                                   inheritedBatteryNodes=inheritedBatteryNodes,firstPredecessor=oldTimeNode.batteryNodes[-1])
            oldTimeNode.includesUb = 0
            oldTimeNode.interval[1] = splitPoint
            self.timeNodes.insert(oldIdx+1,newTimeNode)
            return oldIdx,1
        if isUb:
            if splitPoint < self.timeWindow[1]+eps or 1:
                inheritedBatteryNodes= [node for node in oldTimeNode.batteryNodes 
                                        if (not abs(node.interval[1] - self.batteryInterval[1])<eps)]
            else:
                inheritedBatteryNodes= [oldTimeNode.batteryNodes[0]]
            newTimeNode = TimeNode(self, (oldTimeNode.interval[0],splitPoint),oldTimeNode.includesLb,isUb,
                                    batteryInterval=self.batteryInterval,
                                   inheritedBatteryNodes=inheritedBatteryNodes,firstPredecessor=oldTimeNode.batteryNodes[-1])
            oldTimeNode.includesLb = 0
            oldTimeNode.interval[0] = splitPoint
            self.timeNodes.insert(oldIdx,newTimeNode)
            oldIdx += 1
            return oldIdx,-1
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def incorporateSplitPoint(self,timeSplitPoint,batterySplitPoint,modelHandler=0,dualValueLocationList=[]):
        timeSplitPoint = float(timeSplitPoint)
        batterySplitPoint = float(batterySplitPoint)
        if timeSplitPoint > self.timeWindow[0]-eps:
            oldTimeIndex,timeShift = self.splitTimeNodes(timeSplitPoint)
        else:
            myAnalyser.recordError(f"Error while splitting nodes: '{timeSplitPoint}' Splitpoint cannot be lower than left hand side of interval")
            return
        newTimeIndex = oldTimeIndex + timeShift
        oldBatIndex,predBatteryNode = self.timeNodes[oldTimeIndex].findLargestReachableBatteryNode(batterySplitPoint)
        oldBatIndex,batShift = self.timeNodes[newTimeIndex].splitBatteryNodes(batterySplitPoint,
                                                    predecessor=predBatteryNode)
        newBatIndex = oldBatIndex + batShift
        if batShift == 0 and timeShift == 0:
            return 0
        if timeShift == 0:
            oldBatteryNodeList = [self.timeNodes[oldTimeIndex].batteryNodes[oldBatIndex]]
        else:
            oldBatteryNodeList = [oldBNode for oldBNode in self.timeNodes[oldTimeIndex].batteryNodes]
        if timeShift == 0:
            newBatteryNodeList = [self.timeNodes[oldTimeIndex].batteryNodes[newBatIndex]]
        else:
            newBatteryNodeList = [newBNode for newBNode in self.timeNodes[newTimeIndex].batteryNodes]
        
        for newBatteryNode in newBatteryNodeList:
            if modelHandler != 0:
                modelHandler.addConstraints(['flow_'+newBatteryNode.toStr()],[],['E'],[0.0])
                if newBatteryNode.predecessor == 0:
                    myAnalyser.recordError(f"""Error: No predecessor of battery node 
                                           {newBatteryNode.toStr()} set, so dual value location could not be set.""")
                    dualValueLocationList.append(0)
                else:
                    #print(newBatteryNode.predecessor)
                    dualValueLocationList.append(modelHandler.constName2idx['flow_' + newBatteryNode.predecessor.toStr()])
            newBatteryNode.createOutgoingArcs(modelHandler)
        for oldBatteryNode in oldBatteryNodeList:
            removedIndices = []
            for i,arc in enumerate(oldBatteryNode.ingoingArcs):
                actuallyChanged = arc.findNewHead(modelHandler)
                if actuallyChanged: removedIndices.append(i)
            oldBatteryNode.ingoingArcs = [arc for j,arc in enumerate(oldBatteryNode.ingoingArcs) if j not in removedIndices]
        if newTimeIndex < len(self.timeNodes)-1 and batShift!=0:
            self.incorporateSplitPoint(self.timeNodes[newTimeIndex+1].interval[0],batterySplitPoint,modelHandler,dualValueLocationList)#ToDo auch fuer andere Dateien einführen
        return 1
    def __str__(self):
        retstr = f"VrpNode with index {self.name}, Time Window: {self.timeWindow}, Battery Interval: {self.batteryInterval}"
        for timeNode in self.timeNodes:
            retstr += "\n"+timeNode.__str__()
        return retstr
    def __repr__(self):
        return self.__str__()
        
        

class TimeNode():
    def __init__(self,vrpNode,interval,includesLb=True,includesUb=False,batteryInterval=[],inheritedBatteryNodes=[],firstPredecessor=0):
        self.vrpNode = vrpNode
        self.name = vrpNode.name
        self.interval = list(interval)
        self.includesLb = includesLb
        self.includesUb = includesUb
        if batteryInterval != []:
            self.batteryNodes = [BatteryNode(self,batteryInterval,True,True,predecessor = firstPredecessor)]
        for batteryNode in reversed(inheritedBatteryNodes):
            self.splitBatteryNodes(batteryNode.interval[1],predecessor=batteryNode)
    def isContained(self,time):
        return self.isGreaterThanLb(time) and self.isSmallerThanUb(time)
    def isGreaterThanLb(self,time):
        return (time > self.interval[0]+eps-2*eps*self.includesLb)
    def isSmallerThanUb(self,time):
        return (time < self.interval[1]-eps+2*eps*self.includesUb)
    def isLowerBound(self,time):
        return (time>self.interval[0]-eps and time< self.interval[0]+eps)
    def isUpperBound(self,time):
        return (time>self.interval[1]-eps and time < self.interval[1]+eps)
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def findLargestReachableBatteryNode(self,arrivalBattery):
        if 'batteryNodes' in self.__dict__:
            for i,batteryNode in enumerate(self.batteryNodes):
                if batteryNode.isContained(arrivalBattery):
                    return i,batteryNode
        return -1,0
    #first return value is the position of the  battery node the splitpoint was contained in, the second is the shift to the newly added timenode
    # So if the second value is 0 nothing was changed
    def splitBatteryNodes(self,splitPoint,isLb=False,isUb=True,predecessor=0):
        if isLb and isUb:
            myAnalyser.recordError(f"Error while splitting nodes: '{splitPoint}' Splitpoint cannot be both lower and upper bound")
            return
        oldIdx,oldBatteryNode = self.findLargestReachableBatteryNode(splitPoint)
        if oldIdx ==-1:
            myAnalyser.recordError(f"""Error while splitting nodes: '{splitPoint}' Splitpoint was not contained 
                                   in the battery intervals of the battery nodes""")
            return
        if oldBatteryNode.includesUb  and abs(splitPoint-oldBatteryNode.interval[1])<eps:
            #myAnalyser.recordError(f"Error while splitting nodes: '{splitPoint}' was already used as Upper bound")
            return oldIdx,0
        if isUb:
            newBatteryNode = BatteryNode(self, (oldBatteryNode.interval[0],splitPoint),oldBatteryNode.includesLb,isUb,predecessor)
            oldBatteryNode.includesLb = 0
            oldBatteryNode.interval[0] = splitPoint
            self.batteryNodes.insert(oldIdx,newBatteryNode)
            oldIdx += 1
            return oldIdx,-1
        if isLb:
            newBatteryNode = BatteryNode(self, (splitPoint,oldBatteryNode.interval[1]),isLb,oldBatteryNode.iincludesUb,predecessor)
            oldBatteryNode.includesUb = 0
            oldBatteryNode.interval[1] = splitPoint
            self.batteryNodes.insert(oldIdx+1,newBatteryNode)
            return oldIdx,1   
    def findUpperBound(self,battery):
        for batteryNode in self.batteryNodes:
            if batteryNode.isUpperBound(battery):return True,batteryNode
        return False,0
    def __str__(self):
        if self.includesLb:
            istring = "["
        else:
            istring = "("
        istring += f"{self.interval[0]},{self.interval[1]}"
        if self.includesUb:
            istring += "]"
        else:
            istring += ")"
        
        retstr = f"Timenode belonging to node {self.name}, Interval = " +istring
        for batNode in self.batteryNodes:
            retstr += "\n"+batNode.__str__()
        return retstr

class BatteryNode(TimeNode):
    def __init__(self,timeNode,interval,includesLb=False,includesUb=True,predecessor=0):
        self.timeNode = timeNode
        self.name = timeNode.name
        self.interval = list(interval)
        self.includesLb = includesLb
        self.includesUb = includesUb
        self.outgoingArcs = []
        self.ingoingArcs = []
        self.predecessor = predecessor
    def createOutgoingArcs(self,modelHandler=0):
        vrp = self.timeNode.vrpNode.vrp
        for j in vrp.fullAdjMatrix[self.name]:
            arrivalTime,arrivalBattery,cost = arcLength(self.timeNode.interval[0],self.interval[1],
                                    vrp.nodes[j].timeWindow,vrp.nodes[j].batteryInterval,vrp.fullAdjMatrix[self.name][j],
                                    -vrp.depotDistances[self.name],-vrp.depotDistances[j],vrp.loadFactor,vrp.batteryCapacity
                                    )
            timeNodeIndex,timeNodeHead = vrp.nodes[j].findLowestReachableTimeNode(arrivalTime)
            batNodeIndex,batNodeHead = timeNodeHead.findLargestReachableBatteryNode(arrivalBattery)
            newArc = Arc(self,batNodeHead,arrivalTime,arrivalBattery,cost)
            if modelHandler != 0:
                newArc.registerToModel(modelHandler)
    def toStr(self):
        return "%d_%.3f_%.3f" % self.getId()
    def getId(self):
        id1=self.name
        if self.timeNode.includesLb:
            id2 = self.timeNode.interval[0]
        else:
            id2 = self.timeNode.interval[1]
        if self.includesUb:
            id3 = self.interval[1]
        else:
            id3 = self.interval[0]
        return (id1,id2,id3)
    def __str__(self):
        if self.includesLb:
            istring = "["
        else:
            istring = "("
        istring += f"{self.interval[0]},{self.interval[1]}"
        if self.includesUb:
            istring += "]"
        else:
            istring += ")"
            
        return f"Batterynode of node {self.name}, Interval = " +istring
        
class Arc():
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def __init__(self,tail,head,arrivalTime,arrivalBattery,cost):
        self.head = head         
        self.tail = tail
        self.arrivalTime = arrivalTime
        self.arrivalBattery = arrivalBattery
        self.cost = cost
        self.head.ingoingArcs.append(self)
        self.tail.outgoingArcs.append(self)
        self.head.timeNode.vrpNode.vrp.arcDictionary[tail.name,head.name].append(self)
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def findNewHead(self,modelHandler=0):
        if self.hasCorrectLength():
            return False
        oldVarName = self.getCurrentVarName()
        oldHead = self.head
        headVrpNode = self.head.timeNode.vrpNode
        timeNodeindex , timeNodeHead = headVrpNode.findLowestReachableTimeNode(self.arrivalTime)
        batNodeIndex,self.head = timeNodeHead.findLargestReachableBatteryNode(self.arrivalBattery)
        if modelHandler != 0:
            newVarName = self.getCurrentVarName()
            modelHandler.changeConstraintCoefficient("flow_" + oldHead.toStr(),oldVarName,0)
            modelHandler.changeVariableName(oldVarName,newVarName)
            modelHandler.changeConstraintCoefficient("flow_" + self.head.toStr(),newVarName,1.0)
            modelHandler.yVarName2Arc.pop(oldVarName)
            modelHandler.yVarName2Arc[newVarName] = self
        if oldHead.toStr() != self.head.toStr():
            self.head.ingoingArcs.append(self)
            return True
        else:
            return False
    def createVarInModel(self,modelHandler):
        modelHandler.yVarName2Arc[self.getCurrentVarName()] = self
        modelHandler.addVariables([self.getCurrentVarName()],['C'],[self.cost],[0.0],[1.0],'y')
    def registerToModel(self,modelHandler):
        modelHandler.addVariables([self.getCurrentVarName()],['C'],[self.cost],[0.0],[1.0],'y')
        modelHandler.yVarName2Arc[self.getCurrentVarName()] = self
        modelHandler.changeConstraintCoefficient('arcuse_%d_%d'%(self.tail.name,self.head.name),self.getCurrentVarName(), 1.0)
        if self.tail.name not in [self.head.timeNode.vrpNode.vrp.goal,self.tail.timeNode.vrpNode.vrp.depot]:
            modelHandler.changeConstraintCoefficient('flow_'+self.tail.toStr(),self.getCurrentVarName(), -1.0)
        if self.head.name not in [self.head.timeNode.vrpNode.vrp.goal,self.tail.timeNode.vrpNode.vrp.depot] :  
            modelHandler.changeConstraintCoefficient('flow_'+self.head.toStr(),self.getCurrentVarName(), 1.0)
    def toStr(self):
        return self.tail.toStr()+'_'+self.head.toStr()
    def getCurrentVarName(self):
        return 'y_'+self.toStr()
    def hasCorrectLength(self):
        return floatEqual(self.arrivalTime, self.head.timeNode.interval[0]) and floatEqual(
                self.arrivalBattery,self.head.interval[1])



class TreeNode():
    def __init__(self,tree,branches,dualValuesModel=0,dualValuesBranches=0,primalValues = 0,slacks = 0,redCosts = 0):
        self.tree = tree
        self.branches = branches
        self.branchNames = [branch.name for branch in self.branches]
        self.branchLinExprs = [branch.linExpr for branch in self.branches]
        self.branchSenses = [branch.sense for branch in self.branches]
        self.branchRhs = [branch.rhs for branch in self.branches]
        self.dualValuesModel = dualValuesModel
        self.dualValuesBranches = dualValuesBranches
        self.primalValues = primalValues
        self.slacks = slacks
        self.redCosts = redCosts
        self.lowerBound = 0
        self.fractionals = {}
        if self.tree.vrp.modelHandler.solveLpRelaxation(self) and self.lowerBound < self.tree.ub:
            self.feasible = 1
        else:
            self.feasible = 0
    def getOneBranches(self):
        retval = 0
        for rhs in self.branchRhs:
            if abs(rhs-1) < eps:
                retval += 1
        return retval   
    def updateDualValues(self,dualValueLocations,extraValues=[]):
        for ind in dualValueLocations:
            self.dualValuesModel.append(self.dualValuesModel[ind])
        if extraValues != []:
            self.dualValuesModel += extraValues
    def chooseBranchVar(self):
        #return self.branchVar, self.branchVal
        #t0 = time.time()
        maxScore = -1000
        chosenVar = -1
        chosenVal = -1
        scores = []
        for varName,varVal in self.fractionals.items():
            #return varName,varVal
            score,has_history = self.calcScore(varName)
            if has_history:
                scores.append(score)
            if score > maxScore:
                maxScore = score
                chosenVar = varName
                chosenVal = varVal
        if len (scores)>0:
            self.tree.psiAvg = sum(scores)/len(scores)
        #self.tree.branchVariable_selection_time += time.time()-t0
        if chosenVar == -1:
            print (self.fractionals)
            print ("Error")
        return chosenVar, chosenVal
    def calcScore(self,varName):
        branchHistory = self.tree.branchHistory
        if len(branchHistory[varName])>0:
            psi_0 = 0.0
            psi_1 = 0.0
            for tup in branchHistory[varName]:
                psi_0 += tup[0]
                psi_1 += tup[1]
            psi_0 = psi_0 /len(branchHistory[varName])
            psi_1 = psi_1 /len(branchHistory[varName])
            return (5.0/6)*min(psi_0,psi_1)+1.0/6*max(psi_0,psi_1),1
        else:
            return self.tree.psiAvg,0.0



class Branch():
    def __init__(self,var,sense,rhs):
        self.var = var
        self.name= var + str(rhs)
        self.sense = sense
        self.rhs = rhs
        self.linExpr = cplex.SparsePair([var],[1])  
    def __repr__(self):
        return self.var +" sense: " +(self.sense)
    def __str__(self):
        return self.var +" sense: " +(self.sense)


class Tree():
    necessaryInstanceParameterList = ['timeLimit']
    def __init__(self,vrp,controlParameters):
        if not self.checkParameters(controlParameters):return
        self.vrp = vrp
        self.registerInstanceParameters(controlParameters)
        self.ub = bigNum
        self.lb = -bigNum
        self.printInterval = 6
        self.count = 0
        self.psiAvg = 0.0
        self.root = TreeNode(self,[])
        self.openNodes = [self.root]
        self.branchHistory = {key:[] for key in self.vrp.modelHandler.xNames}
    def registerInstanceParameters(self,controlParameters):
        for paramName,paramVal in controlParameters.items():
            self.__setattr__(paramName,paramVal)
    def checkParameters(self,controlParameters):
        retVal = True
        for paramName in Tree.necessaryInstanceParameterList:
            if paramName not in controlParameters:
                myAnalyser.recordError(f"Error in Treecreation: Parameter '{paramName}' was not included in control parameter dictionary")
                retVal=False
        return retVal
    def conditionalPrint(self,string):
        if self.count % self.printInterval == 0:
            print (string)
    #@registerTime
    #@registerCalls
    def chooseNode(self,selection=2,fac=100):
        if selection == 1:
            minInd = 0
            minVal= 100000000
            for i,node in enumerate(self.openNodes):
                #if (len(node.fractionals)==0):
                #    return self.openNodes.pop(i)
                if node.lowerBound < minVal:
                    minInd = i
                    minVal = node.lowerBound
            if  self.lb < minVal:
                self.lb=minVal
        if selection == 2:
            minInd = 0
            minVal= 100000000
            minLb = 100000000
            for i,node in enumerate(self.openNodes):
                if node.lowerBound - fac*node.getOneBranches() < minVal:
                    minInd = i
                    minVal = node.lowerBound - fac*node.getOneBranches() 
                if node.lowerBound < minLb:
                    minLb = node.lowerBound
            if self.lb < minLb:
                self.lb = minLb
        return self.openNodes.pop(minInd)
    #@registerTime
    #@registerCalls
    def solveMIPformulation(self):
        self.vrp.modelHandler.createMIPmodel()
    def branch(self,node):
        branchVar,branchVal = node.chooseBranchVar()
        #print "branching"
        f_1 = 1.0-branchVal
        f_0 = branchVal
        if self.vrp.modelHandler.adaptDuals:
            new_node_list = [TreeNode(node.tree,node.branches+[Branch(branchVar,'E',0.0)],
                                                                node.dualValuesModel,
                                                                node.dualValuesBranches+[0.0]),
                             TreeNode(node.tree,node.branches+[Branch(branchVar,'E',1.0)],
                                                                node.dualValuesModel,
                                                                node.dualValuesBranches+[0.0])]
        else:
            new_node_list = [TreeNode(node.tree,node.branches+[Branch(branchVar,'E',0.0)]),
                             TreeNode(node.tree,node.branches+[Branch(branchVar,'E',1.0)])]  
        if new_node_list[0].feasible:
            c_0 = (new_node_list[0].lowerBound-node.lowerBound)/f_0
        else:
            c_0 = self.psiAvg/f_0
        if new_node_list[1].feasible:
            c_1 = (new_node_list[1].lowerBound-node.lowerBound)/f_1
        else:
            c_1 = self.psiAvg/f_1
        self.branchHistory[branchVar].append((c_0,c_1))
        for new_node1 in new_node_list:
            if new_node1.feasible:
                if new_node1.lowerBound < self.ub-eps:
                    self.openNodes.append(new_node1)
    #@registerTime
    #@registerCalls
    def removeShortCycles(self,shortCycles):
        splitPoints = []
        #print "shortCycles before"
        #print shortCycles
        for S in shortCycles:
            cy= [s[0] for s in S]
            #print "extracted cycle"
            #print cy
            splitPoints += self.vrp.findSplitPointsToBreakCycle(cy)
            #self.vrp.add_ste_cut(cy)
        self.adaptVrpGraph(splitPoints)
    def cleanUpAndAdaptDuals(self,dualValueLocations,extraValues):
        popIndices=[]
        for i,node2 in enumerate(self.openNodes):
            if self.vrp.modelHandler.adaptDuals:
                node2.updateDualValues(dualValueLocations,extraValues)
            if len(extraValues)+len(dualValueLocations)>0:
                node2.updatedLpRelaxation = 0
            if not node2.feasible or node2.lowerBound >= self.ub-eps:#TODO: Adapt this to non integer objective
                popIndices.append(i)
        while (len(popIndices)>0):
            self.openNodes.pop(popIndices.pop(-1))
    def adaptVrpGraph(self,splitPoints):
        dualValueLocations=[]
        actuallySplit = 0
        if self.vrp.modelHandler.adaptDuals:
            for i,t,b in splitPoints:
                bat = b
                time = t
                actuallySplit += self.vrp.nodes[i].incorporateSplitPoint(time,bat,self.vrp.modelHandler,dualValueLocations)
        else:
            for i,t,b in splitPoints:
                bat = b
                time = t
                actuallySplit += self.vrp.nodes[i].incorporateSplitPoint(time,bat,modelHandler=self.vrp.modelHandler)
        return dualValueLocations,actuallySplit
    def splitAtRoot(self):
        prevLb = -1000
        lb = -1
        count = 0
        #dualValueLocations,splitSuccessfull = self.adaptVrpGraph([(node.name,node.timeWindow[1],node.batteryInterval[0]) 
        #                                            for node in self.vrp.nodes if node.name != self.vrp.goal and node.name != self.vrp.depot])
        #self.cleanUpAndAdaptDuals(dualValueLocations, [])
        #self.vrp.modelHandler.solveLpRelaxation(self.root)
        splitPointList = self.vrp.solToArcCorrections(self.root.primalYValues)
        #splitPointList = self.vrp.findSplitPoints(self.root.primalYValues)

        while (abs(prevLb-lb)> 10 or count < 10) and len(splitPointList)>0:
            dualValueLocations,splitSuccessfull = self.adaptVrpGraph(splitPointList)
            prevLb = lb
            self.cleanUpAndAdaptDuals(dualValueLocations, [])
            self.vrp.modelHandler.solveLpRelaxation(self.root)
            #self.openNodes=[self.root]
            lb = self.root.lowerBound
            count += 1
            #splitPointList = self.vrp.findSplitPoints(self.root.primalYValues)
            splitPointList = self.vrp.solToArcCorrections(self.root.primalYValues)
            print(f"Current root node has a lower bound of {self.root.lowerBound}")
        print(f"Solution rounds at root: {count}")
    @registerTime(myAnalyser)
    @registerCalls(myAnalyser)
    def branchAndRefine(self):
        self.count=0
        prevRootLb = self.root.lowerBound
        #return
        t0 = time.time()
        self.splitAtRoot()
        while ( len(self.openNodes)>eps) and (time.time()-t0 < self.timeLimit):
            self.count+=1
            self.conditionalPrint("Current lower bound: %f" % (self.lb))
            self.conditionalPrint("Current best upper Bound: %f " % (self.ub))
            self.conditionalPrint("Number of open nodes: %d" % len(self.openNodes))
            node = self.chooseNode()
            if node.updatedLpRelaxation == 0:
                self.vrp.modelHandler.solveLpRelaxation(node)
            if not node.feasible or node.lowerBound >= self.ub-eps:
                continue
            if len(node.fractionals)>0:
                self.branch(node)
                continue
            #this part should only be reached for integer solutions
            splitPointList = self.vrp.solToArcCorrections(node.primalYValues)
            #splitPointList = self.vrp.findSplitPoints(node.primalYValues)
            #Segment to split nodes, if nodes are split loop is continued
            #splitPoints = self.vrp.findSplitPointsInfPathRef(sol)#splitPoints pruft, ob die Wege in
            if len(splitPointList)>0:
                print( "Splitting nodes at ", len(splitPointList), "points")
                paths,shortCycles = self.vrp.solToCycles(node.primalYValues)
                if len(shortCycles) == 0:
                    objective = 0.1
                    timedBatteryPaths = []
                    for path in paths:
                        tbp,cost = self.vrp.findTimedBatteryPath(path)
                        objective += cost
                        timedBatteryPaths.append(tbp)
                    if objective < self.ub:
                        print(f"New uperbound found: {objective}")
                        self.ub = objective
                        self.heuristicSol = timedBatteryPaths
                        self.heuristicUb = objective-1.0
                self.latestInfeasibleNode = node
                self.openNodes.append(node)
                dualValueLocations,splitSuccessfull = self.adaptVrpGraph(splitPointList)
                self.cleanUpAndAdaptDuals(dualValueLocations,[])
                if splitSuccessfull == 0:
                    myAnalyser.recordError("Error no node split lb: " + str(node.lowerBound))
                    print (splitPointList)
                    print(node.primalYValues)
                    print(self.vrp.solToCycles(node.primalYValues))
                    time.sleep(10)
                newRoot = TreeNode(self,[])
                print ("current root lb = %.2f" % newRoot.lowerBound)
                if self.lb-newRoot.lowerBound<0.1:
                    print ("Root node has same lower bound as branch tree, restarting from root")
                    self.openNodes=[newRoot]
                    self.lb = newRoot.lowerBound
                    prevRootLb = newRoot.lowerBound
                continue
            #segment if no cut was added and nodes were not split
            if node.lowerBound < self.ub-eps:
                #self.solution_path = self.vrp.solToPaths(sol)
                print( "Integer feasible solution found, objective: %f" %node.lowerBound)
                self.ub = node.lowerBound
                self.solutionNode = node
                self.cleanUpAndAdaptDuals([],[])

instanceNames = {
"SimCologne_C_25_twLength_30_i_1_equalProfits.txt":-22,
"SimCologne_C_25_twLength_30_i_2_equalProfits.txt":-21,
"SimCologne_C_25_twLength_30_i_3_equalProfits.txt":-26,
"SimCologne_C_25_twLength_30_i_4_equalProfits.txt":-23,
"SimCologne_C_25_twLength_30_i_5_equalProfits.txt":-23,
"SimCologne_C_25_twLength_30_i_6_equalProfits.txt":-24,
"SimCologne_C_25_twLength_30_i_7_equalProfits.txt":-23,
"SimCologne_C_25_twLength_30_i_8_equalProfits.txt":-26,
"SimCologne_C_25_twLength_30_i_9_equalProfits.txt":-24,
"SimCologne_C_25_twLength_30_i_10_equalProfits.txt":-23,
"SimCologne_C_25_twLength_30_i_11_equalProfits.txt":-22,
"SimCologne_C_25_twLength_30_i_12_equalProfits.txt":-22,
"SimCologne_C_25_twLength_30_i_13_equalProfits.txt":-24,
"SimCologne_C_25_twLength_30_i_14_equalProfits.txt":-24,
"SimCologne_C_25_twLength_30_i_15_equalProfits.txt":-21,             
                  }
instanceParameters = {'batteryCapacity':120.0,
                      'loadFactor':2.5, 
                      'timeHorizon':900.0,
                      'serviceTime':5.0,
                      'vehicleAmount':4
    }
def boolToAnswer(boo):
    if boo:
        return "yes"
    else:
        return "no"

saveFileName = "Results2Norm"
mip=0
file = open(saveFileName, "a")
file.write("_____________________________\n")
file.close()
solverParameters = {'timeLimit':7200}
for instanceName in sorted(instanceNames.keys()):
    t0= time.time()
    depotDistances,timeWindows,profits = readData(instanceName)
    vrp = VRPgraph(depotDistances,timeWindows,profits,instanceParameters)
    if mip:
        vrp.modelHandler.createMIPmodel()
        vrp.modelHandler.MIPmodel.solve()
        objective = vrp.modelHandler.MIPmodel.solution.get_objective_value()
        print( vrp.modelHandler.MIPmodel.solution.get_objective_value())
        nameVal = zip(vrp.modelHandler.MIPmodel.variables.get_names(),vrp.modelHandler.MIPmodel.solution.get_values())
        for name,val in nameVal:
            if val>0 and name[0]=="g":
                print(name,val)
    else:
        tree = Tree(vrp, solverParameters)
        
        tree.branchAndRefine()
        objective = tree.solutionNode.lowerBound
        tours = tree.vrp.solToCycles(tree.solutionNode.primalYValues)[0]
        for tour in tours:
            for node in tour:
                if node[1]>vrp.nodes[node[0]].timeWindow[1]:
                    print(f"Node {node[0]}, Time-window violation of {node[1]-vrp.nodes[node[0]].timeWindow[1]}")
                if node[2]<vrp.nodes[node[0]].batteryInterval[0]-eps:
                    print("ERROR battery window violated")
    file = open(saveFileName, "a")
    file.write(f"{instanceName}, MIP formulation: {boolToAnswer(mip)}, number of vehicles: {vrp.vehicleAmount}\n time: {time.time()-t0}, obj: {objective} ")
    file.write("\n\n")
    file.close()
#print(time.time()-t0)
print(myAnalyser.functionCalls)
print(myAnalyser.functionTimes)
