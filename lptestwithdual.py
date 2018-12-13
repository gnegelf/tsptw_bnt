import cplex
import random
import cplex
import re
import time
import math

def solToPaths(solutionStringList):
    pathList=[]
    for string in solutionStringList:
        pathList.append([(int(re.split("[_]",string)[1]),int(re.split("[_]",string)[3]))
           ,(int(re.split("[_]",string)[2]),int(re.split("[_]",string)[4]))])
    path = pathList.pop(0)
    count = 0
    while len(pathList) > 0 and count < len(solutionStringList)*5:
        count += 1
        adder = 0
        for i in range(len(pathList)):
            if pathList[i-adder][1][0]==path[0][0]:
                path.insert(0,pathList[i-adder][0])
                pathList.pop(i-adder)
                adder += 1
            else:
                if pathList[i-adder][0][0]==path[-1][0]:
                    path.append(pathList[i-adder][1])
                    pathList.pop(i-adder)
                    adder+=1
    if count == len(solutionStringList)*5:
        print "ERROR"
        print pathList
        print path
    return path


def readData(file_name,direcotry_name ="SSPInstances"):
    
    print "reading "+file_name

    file = open(direcotry_name+"/"+file_name, "r")
    graph_data = file.read()
    file.close()
    
    entries = re.split("\n+", graph_data)
    
    vertNum = int(entries.pop(0))
    adj_matrix=[]
    for i in range(vertNum):
        lineStr = entries.pop(0)
        lineStrList = re.split(" +", lineStr)
        #print lineStrList
        adj_matrix.append([int(val) for val in lineStrList])
    TWs=[]
    for i in range(vertNum):
        lineStr = entries.pop(0)
        lineStrList = re.split(" +", lineStr)
        #lineStrList.pop(-1)
        TWs.append([int(val) for val in lineStrList])
    for i in range(len(TWs)):
        TWs[i] = (TWs[i][0],int(TWs[i][1]*1.3))
    nodes = [[i,TWs[i][0],TWs[i][1]] for i in range(vertNum)]
    return vertNum,TWs,adj_matrix,nodes

 
class Graph():
    def __init__(self,nodes,adj_matrix,origin,destination):
        self.nodes = [Graph_node(self,node[0],node[1],node[2]) for node in nodes]
        self.adj_matrix = adj_matrix
        self.arc_list = [Arc(jj,ii,adj_matrix[i.name][j.name]) 
            for i in self.nodes for ii in i.interval_nodes for j in self.nodes
                for jj in j.interval_nodes if i.name!= j.name and adj_matrix[i.name][j.name] > 0.5 and 
                jj.is_reachable(ii.interval,adj_matrix[i.name][j.name])]
        self.origin = origin
        self.destination = destination
        self.dual_values = []
    def create_model(self):
        self.model = cplex.Cplex()
        model = self.model
        if not self.print_log:
            model.parameters.simplex.display.set(0)
            model.set_results_stream(None)
            model.set_log_stream(None)
        self.x_names = ["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                arc.tail.interval[0],arc.head.interval[0])
                        for node in self.nodes 
                        for interval_node in node.interval_nodes
                        for arc in interval_node.outgoing_arcs]
        all_names = ["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                arc.tail.interval[0],arc.head.interval[0])
                        for node in self.nodes 
                        for interval_node in node.interval_nodes
                        for arc in interval_node.outgoing_arcs]
        all_obj = [self.adj_matrix[arc.tail.name][arc.head.name] 
                            for node in self.nodes 
                        for interval_node in node.interval_nodes
                        for arc in interval_node.outgoing_arcs
                        ]
        all_lb = [0.0]*len(all_names)
        all_ub = [1.0]*len(all_names)
        model.variables.add(names = all_names,
                            types=['C']*len(all_names),obj=all_obj,
                            lb = all_lb,ub = all_ub)
        allvars = []
        allrhs = []
        allsenses = []
        
        
        for node in self.nodes:
            for interval_node in node.interval_nodes:
                thevars = (["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                               arc.tail.interval[0],arc.head.interval[0])
                            for arc in interval_node.outgoing_arcs]+
                            ["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                arc.tail.interval[0],arc.head.interval[0])
                            for arc in interval_node.ingoing_arcs]
                            )
                
                thecoefs = [-1]*len(interval_node.outgoing_arcs)+[1]*len(interval_node.ingoing_arcs)

                    
                allvars.append(cplex.SparsePair(thevars,thecoefs))
                allsenses.append("E")
                if node.name == self.origin:
                    allrhs.append(-1.0)
                else:
                    if node.name == self.destination:
                        allrhs.append(1.0)
                    else:
                        allrhs.append(0.0)
        
        model.linear_constraints.add(lin_expr = allvars, 
                                                     senses = allsenses, rhs = allrhs)
        self.const_num=len(allvars)
        
        self.names =  { n : j for j, n in enumerate(model.variables.get_names()) }
        self.var_num = len(self.names)
        self.model.set_problem_type(0)
        self.model.parameters.lpmethod.set(2)
        self.idx2name = { j : n for j, n in enumerate(model.variables.get_names()) }
        self.name2idx = { n : j for j, n in enumerate(model.variables.get_names()) }
        if self.dual_values != [] and self.use_start and 0:
            model.start.set_start(col_status=[],
                     row_status=[],
                     row_dual=self.dual_values,
                     col_primal=[],
                     row_primal=[],
                     col_dual=[],
                     #row_dual=[]
                     )
    def solve_model(self):
        model = self.model
        model.solve()
        #t0 = time.time()
        solution = model.solution
        #self.feasible = solution.is_primal_feasible()
        self.set_vars = {}
        self.dual_values = solution.get_dual_values()
        self.objective = solution.get_objective_value()
        self.primal_values = dict(zip(self.x_names,solution.get_values()))
        
        for key,val in self.primal_values.iteritems():
            if val >0.9999:
                self.set_vars[key] = 1
            if val > 0.0001 and val < 0.9999:
                print "Error fractional value detected"

    def split_at(self,pathWithTime):
        path = [pat[0] for pat in pathWithTime]
        arrival_time = self.nodes[path[0]].tw_lb
        split_at = -1
        for i in range(0,len(path)-1):
            earliest_arrival = max(self.nodes[path[i+1]].tw_lb,
                                   arrival_time+self.adj_matrix[path[i]][path[i+1]])
            #print earliest_arrival
            if earliest_arrival-0.0001 < self.nodes[path[i+1]].tw_ub:
                arrival_time = earliest_arrival
            else:
                split_at = 1
                break
        
        split_point = 0
        if split_at != -1:
            splitList = []
            for ind in range(len(pathWithTime)-2):
                tail = pathWithTime[ind][0]
                head = pathWithTime[ind+1][0]
                #this way of determining lbs is bad
                lb1 = float(pathWithTime[ind][1])
                lb2 = float(pathWithTime[ind+1][1])
                if lb1+self.adj_matrix[tail][head] > lb2+0.0001:
                    splitList.append((head,lb1+self.adj_matrix[tail][head]))
                    #split_at = head
                    #split_point = lb1+self.adj_matrix[tail][head]
                    #break
        else:
            return -1
        return splitList
        return split_at,split_point

class Graph_node():
    def __init__(self,graph,name,tw_lb,tw_ub):
        self.graph = graph
        self.name = name
        self.tw_lb = tw_lb
        self.tw_ub = tw_ub
        self.interval_nodes = [Interval_node(name,[tw_lb,tw_ub],tw_lb,tw_ub)]
    def split_node(self,split_point):
        idx = self.find_index(split_point)
        self.split_index = idx
        old_ub = self.interval_nodes[idx].interval[1] 
        old_lb = self.interval_nodes[idx].interval[0] 
        if (old_lb >= split_point 
                or split_point >= old_ub):
            print("Error: split point %f outside of node interval[%f,%f]" % (split_point,old_lb,old_ub) )
            return 0
        self.interval_nodes.insert(idx+1,Interval_node(self.name,[split_point,old_ub],self.tw_lb,self.tw_ub))
        self.interval_nodes[idx].interval[1] = split_point
        pop_indices = []
        arc_index = 0
        
        for arc in self.interval_nodes[idx].ingoing_arcs:
            if not arc.head.is_lowest_reachable(arc.tail.interval,arc.length,arc.tail.is_tw_ub()):
                arc.head = self.interval_nodes[idx+1]
                self.interval_nodes[idx+1].ingoing_arcs.append(arc)
                pop_indices.insert(0,arc_index)
            else:
                if self.interval_nodes[idx+1].is_lowest_reachable(arc.tail.interval,arc.length,
                                      arc.tail.is_tw_ub()):
                    new_arc = Arc(self.interval_nodes[idx+1],arc.tail,arc.length)
                    self.graph.arc_list.append(new_arc)
            arc_index += 1
        for i in pop_indices:
            self.interval_nodes[idx].ingoing_arcs.pop(i)
        pop_indices = []
        arc_index = 0
        for arc in self.interval_nodes[idx].outgoing_arcs:
            if not arc.head.is_lowest_reachable(arc.tail.interval,arc.length,arc.tail.is_tw_ub()):
                arc.tail = self.interval_nodes[idx+1]
                self.interval_nodes[idx+1].outgoing_arcs.append(arc)
                pop_indices.insert(0,arc_index)
            else:
                if arc.head.is_lowest_reachable(self.interval_nodes[idx+1].interval,
                                                arc.length,self.interval_nodes[idx+1].is_tw_ub()):
                    new_arc = Arc(arc.head,self.interval_nodes[idx+1],arc.length)
                    self.graph.arc_list.append(new_arc)
                else:
                    is_reachable,new_head = self.graph.nodes[arc.head.name].find_lowest_reachable(self.interval_nodes[idx+1].interval,
                                                arc.length,self.interval_nodes[idx+1].is_tw_ub())
                    if is_reachable:
                        new_arc = Arc(new_head,self.interval_nodes[idx+1],arc.length)
                        self.graph.arc_list.append(new_arc)
                    
            arc_index += 1
        for i in pop_indices:
            self.interval_nodes[idx].outgoing_arcs.pop(i)
    def find_index(self,split_point):
        if split_point < self.tw_lb:
            print ("Error: split point outside of time window (less)")
            return 0
        else:
            idx = 0
            while self.interval_nodes[idx].interval[0] < split_point:
                if idx >= len(self.interval_nodes)-1:
                    idx += 1
                    break
                idx += 1
            if split_point > self.tw_ub:
                print ("Error: split point outside of time window (larger)")
            return idx-1
    def find_lowest_reachable(self,interval,step,ub_included=0):
        for interval_node in self.interval_nodes:
            if interval_node.is_lowest_reachable(interval,step,ub_included):
                return 1,interval_node
        return 0,-1        
            
class Interval_node():
    def __init__(self,name,interval,tw_lb,tw_ub):
        self.name = name
        self.interval = interval
        self.tw_lb = tw_lb
        self.tw_ub = tw_ub
        self.ingoing_arcs = []
        self.outgoing_arcs = []
    def is_reachable(self,interval,step,ub_included=0):
        if not ub_included:
            if (interval[0]+step < self.interval[1] and (self.is_tw_lb() or interval[1]+step > self.interval[0])):
                return 1
        else:
            if (interval[0]+step < self.interval[1] and (self.is_tw_lb() or interval[1]+step >= self.interval[0])):
                return 1
        return 0
    def is_tw_lb(self):
        return abs(self.interval[0] - self.tw_lb)< 0.0001
    def is_tw_ub(self):
        return abs(self.interval[1] - self.tw_ub)< 0.0001
    def is_lowest_reachable(self,interval,step,ub_included=0):
        if (interval[0]+step < self.interval[1] and (self.is_tw_lb() or interval[0]+step >= self.interval[0])):
            return 1
        return 0


class Arc():
    def __init__(self,head,tail,length):
        self.head = head
        self.head.ingoing_arcs.append(self)
        self.tail = tail
        self.tail.outgoing_arcs.append(self)
        self.length = length

#feasible starting basis: shortest path dualized
#instance_name="n20w20.001.txt"
#vert_num,TWs,adj_matrix,nodes = readData("TEST","DUMAS")
"""
vert_num = 100
sqrt=math.sqrt

adj_matrix = [[0,3,0,0,sqrt(20),0,0,sqrt(8),0,0],
               [0,0,3,0,0,0,0,0,0,0],
               [0,0,0,3,0,0,sqrt(13),0,0,0],
               [0,0,0,0,0,0,0,0,0,3],
               [0,0,sqrt(32),0,0,4,0,0,0,0],
               [0,0,0,0,0,0,sqrt(13),0,0,0],
               [0,0,0,2,0,0,0,0,0,sqrt(13)],
               [0,sqrt(5),0,0,0,0,0,0,sqrt(8),0],
               [0,0,sqrt(20),sqrt(41),0,0,0,0,0,0],
               [0,0,0,0,0,0,0,0,0,0]
               ]
TWs= [[0,1000],
      [186,800],
      [5,190],
      [5,190],
      [24,190],
      [25,190],
      [35,190],
      [184,190],
      [10,190],
    [0,1000]]

for i in range(vert_num):
    nodes.append([i,TWs[i][0],TWs[i][1]])"""
vert_num,TWs,adj_matrix,nodes = readData("test3")

#dynamic_discovery = 0
time_limit = 500
keep_history = 0
GRAPH = Graph(nodes,adj_matrix,0,vert_num-1)
GRAPH.print_log = 1
GRAPH.create_model()

GRAPH2 = Graph(nodes,adj_matrix,0,vert_num-1)
GRAPH2.print_log = 1
GRAPH2.create_model()
model = GRAPH.model
print "Log of initial model with infeasible route:"
model.parameters.lpmethod.set(2)
#model.parameters.preprocessing.presolve.set(0)
model.set_problem_type(0)
model.variables.add(names = ["xx_0_1","xx_1_2","xx_2_4","xx_1_4"],
                             lb=[0]*4,obj=[2,1,120,142])

varis= [
              ["xx_0_1","xx_1_2","xx_1_4"],
              ["xx_1_2","xx_2_4"],
              ]
coefs= [
              [1,-1,-1],
              [1,-1],
              ]

allvars = [cplex.SparsePair([v for v in varis[j]],[coef for coef in coefs[j]]) for j in range(2)]
model.linear_constraints.add(names = ["2","3"],lin_expr = allvars, 
                                                     senses = ['E']*2, rhs = [0,0])
model.linear_constraints.set_coefficients(0,"xx_0_1",-1.0)
model.linear_constraints.set_coefficients(799,"xx_2_4",1.0)
model.linear_constraints.set_coefficients(799,"xx_1_4",1.0)

model.solve()
dual_values = model.solution.get_dual_values()
deleted_dual_val1 = model.solution.get_dual_values("2")
deleted_dual_val2 = model.solution.get_dual_values("3")
print "Objective value: %f" %  model.solution.get_objective_value()
print "\n\nLog of refined model created from previous model:"
model.variables.add(names = ["xx_1_2_2"],lb=[0],ub=[1],obj=[1])
#chosenCols = [i for i in range(1500,1510)]

model.linear_constraints.delete(["2"])
model.linear_constraints.delete(["3"])
varis= [
              ["xx_0_1","xx_1_4"],
              ["xx_1_2_2"],
              ["xx_1_2_2","xx_2_4"],
              ]
coefs= [[1,-1],
              [1],
              [1,-1],
              ]

allvars = [cplex.SparsePair([v for v in varis[j]],[coef for coef in coefs[j]]) for j in range(3)]
model.linear_constraints.add(lin_expr = allvars, 
                                                     senses = ['E']*3, rhs = [0,0,0])

dual_values.pop(-2)
dual_values.pop(-1)
dual_values.append(deleted_dual_val1)
dual_values.append(deleted_dual_val1)
dual_values.append(deleted_dual_val2)
#"""
model.start.set_start(col_status=[],
                     row_status=[],
                     row_dual=dual_values,
                     col_primal=[],
                     row_primal=[],
                     col_dual=[],
                     #row_dual=[]
                     )
#"""
model.solve()
print "Objective value: %f" % model.solution.get_objective_value()
print "\n\nLog of refined model created from scratch without start:"
model2=GRAPH2.model
#model2.parameters.preprocessing.presolve.set(0)
model2.set_problem_type(0)
model2.variables.add(names = ["xx_0_1","xx_1_2","xx_2_4","xx_1_2_2","xx_1_4"],
                             lb=[0]*5,obj=[2,1,120,1,142])

varis= [
              ["xx_0_1","xx_1_4"],
              ["xx_1_2_2"],
              ["xx_1_2_2","xx_2_4"],
              ]
coefs= [[1,-1],
              [1],
              [1,-1],
              ]
allvars = [cplex.SparsePair([v for v in varis[j]],[coef for coef in coefs[j]]) for j in range(3)]
model2.linear_constraints.add(lin_expr = allvars, 
                                                     senses = ['E']*3, rhs = [0,0,0])
model2.linear_constraints.set_coefficients(0,"xx_0_1",-1.0)
model2.linear_constraints.set_coefficients(799,"xx_2_4",1.0)
model2.linear_constraints.set_coefficients(799,"xx_1_4",1.0)
model2.parameters.lpmethod.set(2)
model2.start.set_start(col_status=[],
                     row_status=[],
                     row_dual=dual_values,
                     col_primal=[],
                     row_primal=[],
                     col_dual=[],
                     #row_dual=[]
                     )
model2.solve()
print "Objective value: %f" % model2.solution.get_objective_value()
print "\n\nLog of refined model created from scratch:"
model2=GRAPH2.model
#model2.parameters.preprocessing.presolve.set(0)
model2.set_problem_type(0)
model2.variables.add(names = ["xx_0_1","xx_1_2","xx_2_4","xx_1_2_2","xx_1_4"],
                             lb=[0]*5,obj=[2,1,120,1,142])

varis= [
              ["xx_0_1","xx_1_4"],
              ["xx_1_2_2"],
              ["xx_1_2_2","xx_2_4"],
              ]
coefs= [[1,-1],
              [1],
              [1,-1],
              ]
allvars = [cplex.SparsePair([v for v in varis[j]],[coef for coef in coefs[j]]) for j in range(3)]
model2.linear_constraints.add(lin_expr = allvars, 
                                                     senses = ['E']*3, rhs = [0,0,0])
model2.linear_constraints.set_coefficients(0,"xx_0_1",-1.0)
model2.linear_constraints.set_coefficients(799,"xx_2_4",1.0)
model2.linear_constraints.set_coefficients(799,"xx_1_4",1.0)
model2.parameters.lpmethod.set(2)
model2.start.set_start(col_status=[],
                     row_status=[],
                     row_dual=dual_values,
                     col_primal=[],
                     row_primal=[],
                     col_dual=[],
                     #row_dual=[]
                     )
model2.solve()
print "Objective value: %f" % model2.solution.get_objective_value()