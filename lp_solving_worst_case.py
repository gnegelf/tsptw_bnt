import cplex
import re
import time
import math
import scipy.io
import numpy
#from cplex.callbacks import BranchCallback


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
            if pathList[i-adder][1][0]==path[0][0] and pathList[i-adder][1][1]==path[0][1]:
                path.insert(0,pathList[i-adder][0])
                pathList.pop(i-adder)
                adder += 1
            else:
                if pathList[i-adder][0][0]==path[-1][0] and pathList[i-adder][0][1]==path[-1][1]:
                    path.append(pathList[i-adder][1])
                    pathList.pop(i-adder)
                    adder+=1
    if count == len(solutionStringList)*5:
        print "ERROR"
        print pathList
        print path
        return -1
    return path


def readData(file_name,sparse=1,direcotry_name ="SSPInstances"):
    
    print "reading "+file_name

    file = open(direcotry_name+"/"+file_name, "r")
    graph_data = file.read()
    file.close()
    
    entries = re.split("\n+", graph_data)
    if not sparse:
        vertNum = int(entries.pop(0))
        adj_matrix=[]
        for i in range(vertNum):
            lineStr = entries.pop(0)
            lineStrList = re.split(" +", lineStr)
            #print lineStrList
            adj_matrix.append([int(val) for val in lineStrList])
        cost_matrix=[]
        for i in range(vertNum):
            lineStr = entries.pop(0)
            lineStrList = re.split(" +", lineStr)
            #print lineStrList
            cost_matrix.append([int(val) for val in lineStrList])
        TWs=[]
        for i in range(vertNum):
            lineStr = entries.pop(0)
            lineStrList = re.split(" +", lineStr)
            #lineStrList.pop(-1)
            TWs.append([int(val) for val in lineStrList])
        for i in range(len(TWs)):
            TWs[i] = (TWs[i][0],int(TWs[i][1]))
        nodes = [[i,TWs[i][0],TWs[i][1]] for i in range(vertNum)]
        return vertNum,TWs,adj_matrix,nodes,cost_matrix
    else:
        vertNum = int(entries.pop(0))
        adj_matrix= {j:{} for j in range(vertNum)}
        for i in range(vertNum):
            lineStr = entries.pop(0)
            if len(lineStr)==1:
                continue
            lineStrList = re.split(" +", lineStr)
            #print lineStrList
            for j in range(0,len(lineStrList),2):
                adj_matrix[i][int(lineStrList[j])] = int(lineStrList[j+1])
        cost_matrix={j:{} for j in range(vertNum)}
        for i in range(vertNum):
            lineStr = entries.pop(0)
            if len(lineStr)==1:
                continue
            lineStrList = re.split(" +", lineStr)
            #print lineStrList
            for j in range(0,len(lineStrList),2):
                cost_matrix[i][int(lineStrList[j])] = int(lineStrList[j+1])
        TWs=[]
        for i in range(vertNum):
            lineStr = entries.pop(0)
            lineStrList = re.split(" +", lineStr)
            #lineStrList.pop(-1)
            TWs.append([int(val) for val in lineStrList])
        for i in range(len(TWs)):
            TWs[i] = [TWs[i][0],int(TWs[i][1])]

        nodes = [[i,TWs[i][0],TWs[i][1]] for i in range(vertNum)]
        return vertNum,TWs,adj_matrix,nodes,cost_matrix
 
class Graph():
    def __init__(self,nodes,adj_matrix,cost_matrix,origin,destination):
        self.nodes = [Graph_node(self,node[0],node[1],node[2]) for node in nodes]
        self.adj_matrix = adj_matrix
        self.cost_matrix = cost_matrix
        #self.arc_list = [Arc(jj,ii,adj_matrix[i.name][j.name],cost_matrix[i.name][j.name]) 
        #    for i in self.nodes for ii in i.interval_nodes for j in self.nodes
        #        for jj in j.interval_nodes if i.name!= j.name and adj_matrix[i.name].has_key(j.name) and 
        #        jj.is_reachable(ii.interval,adj_matrix[i.name][j.name])]
        self.arc_list = [Arc(jj,ii,adj_matrix[i.name][j],cost_matrix[i.name][j]) 
            for i in self.nodes for ii in i.interval_nodes for j in adj_matrix[i.name]
                for jj in self.nodes[j].interval_nodes if  
                jj.is_reachable(ii.interval,adj_matrix[i.name][j])]
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
        #self.x_names = ["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
        #                        arc.tail.interval[0],arc.head.interval[0])
        #                for node in self.nodes 
        #                for interval_node in node.interval_nodes
        #                for arc in interval_node.outgoing_arcs]
        all_names = ["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                arc.tail.interval[0],arc.head.interval[0])
                        for node in self.nodes 
                        for interval_node in node.interval_nodes
                        for arc in interval_node.outgoing_arcs]
        all_obj = [self.cost_matrix[arc.tail.name][arc.head.name]+0.0001*self.adj_matrix[arc.tail.name][arc.head.name] 
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
        allnames = []
        
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
                allnames.append("c_%d_%d" % (node.name,interval_node.interval[0]))
                if node.name == self.origin:
                    allrhs.append(-1.0)
                else:
                    if node.name == self.destination:
                        allrhs.append(1.0)
                    else:
                        allrhs.append(0.0)
        
        model.linear_constraints.add(lin_expr = allvars, names = allnames,
                                                     senses = allsenses, rhs = allrhs)
        self.const_num=len(allvars)
        
        self.names =  { n : j for j, n in enumerate(model.variables.get_names()) }
        self.dual_names = { n : j for j, n in enumerate(model.linear_constraints.get_names()) }
        self.var_num = len(self.names)
        
        #self.model.parameters.preprocessing.reduce.set(0)
        
        #self.idx2name = { j : n for j, n in enumerate(model.variables.get_names()) }
        #self.name2idx = { n : j for j, n in enumerate(model.variables.get_names()) }
    def solve_model(self):
        model = self.model
        if self.dual_values != [] and self.use_start:
            model.start.set_start(col_status=[],
                     #row_status=self.row_status,
                     row_status=[],
                     row_dual=self.dual_values,
                     col_primal=[],
                     row_primal=[],
                     col_dual=[],
                     #row_dual=[]
                     )
            self.model.parameters.advance.set(1)
            
        else:
            if self.delete_start:
                self.model.parameters.advance.set(0)
        self.model.parameters.lpmethod.set(2)
        self.model.parameters.preprocessing.presolve.set(0)
        self.model.set_problem_type(0)
        t1 = time.time()
        model.solve()
        self.lp_time += time.time()-t1
        #t0 = time.time()
        solution = model.solution
        #self.feasible = solution.is_primal_feasible()
        self.set_vars = {}
        self.dual_values = solution.get_dual_values()
        #self.row_status = solution.basis.get_basis()[1]
        #self.col_status = solution.basis.get_basis()[0]
        self.objective = solution.get_objective_value()
        self.primal_values = dict(zip(self.model.variables.get_names(),solution.get_values()))
        
        for key,val in self.primal_values.iteritems():
            if val >0.9999:
                self.set_vars[key] = 1
            if val > 0.0001 and val < 0.9999:
                print "Error fractional value detected"

    def split_at2(self,pathWithTime):
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
        #splitList=[splitList[0]]
        return splitList
        return split_at,split_point
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
                lb1 = split_point
                lb2 = float(pathWithTime[ind+1][1])
                ub2 = self.nodes[head].tw_ub
                if lb1+self.adj_matrix[tail][head] > lb2+0.0001 and lb1+self.adj_matrix[tail][head] < ub2+0.0001:
                    splitList.append((head,lb1+self.adj_matrix[tail][head]))
                    #split_at = head
                    #split_point = lb1+self.adj_matrix[tail][head]
                    #break
                split_point += self.adj_matrix[tail][head]
        else:
            return -1
        #splitList=[splitList[0]]
        return splitList
        return split_at,split_point

class Graph_node():
    def __init__(self,graph,name,tw_lb,tw_ub):
        self.graph = graph
        self.name = name
        self.tw_lb = tw_lb
        self.tw_ub = tw_ub
        self.interval_nodes = [Interval_node(name,[tw_lb,tw_ub],tw_lb,tw_ub,1)]
    def split_node(self,split_point):
        idx = self.find_index(split_point)
        self.split_index = idx
        old_ub = self.interval_nodes[idx].interval[1] 
        old_lb = self.interval_nodes[idx].interval[0] 
        if (old_lb > split_point+0.0001
                or (split_point > old_ub+0.0001) or (abs(split_point - old_ub) < 0.0001 and abs(old_ub-self.tw_ub) > 0.0001)):
            print("Error: split point %f outside of node interval[%f,%f]" % (split_point,old_lb,old_ub) )
            return 0
        self.interval_nodes.insert(idx+1,Interval_node(self.name,[split_point,old_ub],self.tw_lb,self.tw_ub,self.interval_nodes[idx].is_ub))
        self.interval_nodes[idx].is_ub = 0
        self.interval_nodes[idx].interval[1] = split_point
        pop_indices = []
        arc_index = 0
        
        for arc in self.interval_nodes[idx].ingoing_arcs:
            if not arc.head.is_lowest_reachable(arc.tail.interval,arc.length,arc.tail.is_tw_ub()):
                if self.graph.adapt_model:  
                    old_var_name = "x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.interval[0],arc.head.interval[0])
                
                arc.head = self.interval_nodes[idx+1]
                self.interval_nodes[idx+1].ingoing_arcs.append(arc)
                if self.graph.adapt_model:
                    const_name = "c_%d_%d" % (self.name,self.interval_nodes[idx].interval[0])
                    self.graph.model.linear_constraints.set_coefficients([(const_name,old_var_name,0.0)]) 
                    new_var_name = "x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.interval[0],
                                                      self.interval_nodes[idx+1].interval[0] )  
                    self.graph.model.variables.set_names(old_var_name,new_var_name)
                    self.graph.names[new_var_name] = self.graph.names.pop(old_var_name)
                    self.graph.primal_values[new_var_name] = self.graph.primal_values.pop(old_var_name)
                pop_indices.insert(0,arc_index)

            else:
                if self.interval_nodes[idx+1].is_lowest_reachable(arc.tail.interval,arc.length,
                                      arc.tail.is_tw_ub()):
                    new_arc = Arc(self.interval_nodes[idx+1],arc.tail,arc.length,arc.cost)
                    self.graph.arc_list.append(new_arc)
                    if self.graph.adapt_model:
                        new_var_name = "x_%d_%d_%d_%d" % (new_arc.tail.name, new_arc.head.name,
                                                          new_arc.tail.interval[0],new_arc.head.interval[0])
                        new_obj = self.graph.cost_matrix[new_arc.tail.name][new_arc.head.name] 
                        self.graph.model.variables.add(names=[new_var_name],types=['C'],obj=[new_obj],lb=[0.0],ub=[1.0])
                        self.graph.var_num += 1
                        self.graph.names[new_var_name] = self.graph.var_num -1
                        self.graph.primal_values[new_var_name] = 0
            arc_index += 1
        for i in pop_indices:
            self.interval_nodes[idx].ingoing_arcs.pop(i)
        pop_indices = []
        arc_index = 0
        for arc in self.interval_nodes[idx].outgoing_arcs:
            if not arc.head.is_lowest_reachable(arc.tail.interval,arc.length,arc.tail.is_tw_ub()):
                if self.graph.adapt_model:
                    new_var_name = "x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.interval[0],arc.head.interval[0])
                    old_var_name = "x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                      self.interval_nodes[idx+1].interval[0],arc.head.interval[0])
                    const_name = "c_%d_%d" % (self.name,self.interval_nodes[idx].interval[0])
                    self.graph.model.linear_constraints.set_coefficients([(const_name,old_var_name,0.0)]) 
                    self.graph.model.variables.set_names(old_var_name,new_var_name)
                    self.graph.names[new_var_name] = self.graph.names.pop(old_var_name)
                    self.graph.primal_values[new_var_name] = self.graph.primal_values.pop(old_var_name)
                arc.tail = self.interval_nodes[idx+1]
                self.interval_nodes[idx+1].outgoing_arcs.append(arc)
                pop_indices.insert(0,arc_index)
            else:
                if arc.head.is_lowest_reachable(self.interval_nodes[idx+1].interval,
                                                arc.length,self.interval_nodes[idx+1].is_tw_ub()):
                    new_arc = Arc(arc.head,self.interval_nodes[idx+1],arc.length,arc.cost)
                    self.graph.arc_list.append(new_arc)
                    if self.graph.adapt_model:
                        new_var_name = "x_%d_%d_%d_%d" % (new_arc.tail.name,new_arc.head.name,
                                                          new_arc.tail.interval[0],new_arc.head.interval[0])
                        new_obj = self.graph.cost_matrix[new_arc.tail.name][new_arc.head.name] 
                        self.graph.model.variables.add(names=[new_var_name],types=['C'],obj=[new_obj],lb=[0.0],ub=[1.0])
                        self.graph.var_num += 1
                        self.graph.names[new_var_name] = self.graph.var_num -1
                        self.graph.primal_values[new_var_name] = 0
                        const_name = "c_%d_%d" % (new_arc.head.name,new_arc.head.interval[0])
                        self.graph.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)])  
                else:
                    is_reachable,new_head = self.graph.nodes[arc.head.name].find_lowest_reachable(self.interval_nodes[idx+1].interval,
                                                arc.length,self.interval_nodes[idx+1].is_tw_ub())
                    if is_reachable:
                        new_arc = Arc(new_head,self.interval_nodes[idx+1],arc.length,arc.cost)
                        self.graph.arc_list.append(new_arc)
                        if self.graph.adapt_model:
                            new_var_name = "x_%d_%d_%d_%d" % (new_arc.tail.name,new_arc.head.name,
                                                              new_arc.tail.interval[0],new_arc.head.interval[0])
                            new_obj = self.graph.cost_matrix[new_arc.tail.name][new_arc.head.name] 
                            self.graph.model.variables.add(names=[new_var_name],types=['C'],obj=[new_obj],lb=[0.0],ub=[1.0])
                            self.graph.var_num += 1
                            self.graph.names[new_var_name] = self.graph.var_num -1
                            self.graph.primal_values[new_var_name] = 0
                            const_name = "c_%d_%d" % (new_arc.head.name,new_arc.head.interval[0])
                            self.graph.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)])
            arc_index += 1
        for i in pop_indices:
            self.interval_nodes[idx].outgoing_arcs.pop(i)
        if self.graph.adapt_model:
            allvars = []
            allrhs = []
            allsenses = []
            allnames = []
            interval_node = self.interval_nodes[idx+1]
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
            allnames.append("c_%d_%d" % (self.name,interval_node.interval[0]))
            if self.name == self.graph.origin:
                allrhs.append(-1.0)
                print "Splitting origin"
            else:
                if self.name == self.graph.destination:
                    allrhs.append(1.0)
                    print "splitting destination"
                else:
                    allrhs.append(0.0)
            
            self.graph.model.linear_constraints.add(lin_expr = allvars, names = allnames,
                                                         senses = allsenses, rhs = allrhs)
            self.graph.const_num += 1
            self.graph.dual_values.append(self.graph.dual_values[
                    self.graph.dual_names["c_%d_%d" % (self.name,self.interval_nodes[idx].interval[0])]])
            #self.graph.row_status.append(self.graph.row_status[
            #        self.graph.dual_names["c_%d_%d" % (self.name,self.interval_nodes[idx].interval[0])]])
            #self.graph.dual_values.append(0)
            #self.graph.row_status[-1]= 0
            self.graph.dual_names["c_%d_%d" % (self.name,interval_node.interval[0])] = self.graph.const_num - 1
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
    def __init__(self,name,interval,tw_lb,tw_ub,is_ub):
        self.name = name
        self.interval = interval
        self.tw_lb = tw_lb
        self.tw_ub = tw_ub
        self.ingoing_arcs = []
        self.outgoing_arcs = []
        self.is_ub = is_ub
    def is_reachable(self,interval,step,ub_included=1):
        if self.is_tw_ub:
            if not ub_included:
                if (interval[0]+step <= self.interval[1]+0.0001 and (self.is_tw_lb() or interval[1]+step > self.interval[0]+0.0001)):
                    return 1
            else:
                if (interval[0]+step <= self.interval[1]+0.0001 and (self.is_tw_lb() or interval[1]+step >= self.interval[0]-0.0001)):
                    return 1
        else:
            if not ub_included:
                if (interval[0]+step < self.interval[1]-0.0001 and (self.is_tw_lb() or interval[1]+step > self.interval[0]+0.0001)):
                    return 1
            else:
                if (interval[0]+step < self.interval[1]-0.0001 and (self.is_tw_lb() or interval[1]+step >= self.interval[0]-0.0001)):
                    return 1
        return 0
    def is_tw_lb(self):
        return abs(self.interval[0] - self.tw_lb)< 0.0001
    def is_tw_ub(self):
       # return abs(self.interval[1] - self.tw_ub)< 0.0001
       return self.is_ub
    def is_lowest_reachable(self,interval,step,ub_included=0):
        if self.is_tw_ub():
            if (interval[0]+step <= self.interval[1]+0.0001 and (self.is_tw_lb() or interval[0]+step >= self.interval[0]-0.0001)):
                return 1
        else:
            if (interval[0]+step < self.interval[1]-0.0001 and (self.is_tw_lb() or interval[0]+step >= self.interval[0]-0.0001)):
                return 1
        return 0
    def __str__(self):
        return str(self.interval)
    def __repr__(self):
        return str(self.interval)

class Arc():
    def __init__(self,head,tail,length,cost):
        self.head = head
        self.head.ingoing_arcs.append(self)
        self.tail = tail
        self.tail.outgoing_arcs.append(self)
        self.length = length
        self.cost = cost

def paramReader(fileName):
    print "reading params"

    file = open(fileName, "r")
    paramData = file.read()
    file.close()
    params=[]
    entries = re.split("\n+", paramData)
    entries.pop(-1)
    for lineStr in entries:
        lineStrList = re.split(",+", lineStr)
        #print lineStrList
        params.append((int(lineStrList[0]),int(lineStrList[1])))
    return params

counti= 0
timeDict = [[],[],[]]
for instanceInt in range(2,16):
    
    vert_num,TWs,adj_matrix,nodes,cost_matrix = readData("worstCase_%d" % (instanceInt),direcotry_name ="SPPTW_worst_case_instances")
    #500_21 is interesting
    #500_25 is interesting
    #newInst3000_0 is interesting
    #newInst_4500_3
    #newInst_4200_4
    #newInst_6000_1
    
    #dynamic_discovery = 0
    time_limit = 20000
    keep_history = 0
    
    print "creating Graph"
    GRAPH = Graph(nodes,adj_matrix,cost_matrix,0,vert_num-1)
    path_feasible = 0 
    iteras = 0
    
    lp_time = 0.0
    lp_time2 = 0.0
    #global expand
    GRAPH.expand = 0
    GRAPH.print_log = 0
    GRAPH.use_start = 1
    GRAPH.delete_start = 0
    GRAPH.adapt_model = 1
    GRAPH.lp_time = 0.0
    GRAPH.create_model()
    if GRAPH.expand:
        for i in range(1,len(TWs)-1):
            for j in range(TWs[i][0]+1,TWs[i][1]+1):
                GRAPH.nodes[i].split_node(j)
        GRAPH.create_model()
    
    t0 = time.time()
    solve_times = []
    print "starting to solve"
    simplex_iteras = []
    while time.time()-t0 < time_limit:
        iteras += 1
        if GRAPH.adapt_model == 0 and not iteras == 1:
            GRAPH.create_model()
        
    
        GRAPH.solve_model()
    
        #solve_times.append(time.time()-t1)
        simplex_iteras.append(int(GRAPH.model.solution.progress.get_num_iterations()))
        path = solToPaths(GRAPH.set_vars)
        if path == -1:
            break
        splitList = GRAPH.split_at(path)
        if splitList == -1:
            break
        else:
            for split_node,split_point in splitList:
                GRAPH.nodes[split_node].split_node(int(split_point))
                if not GRAPH.adapt_model:
                    insert_index = 0
                    for i in range(split_node):
                        insert_index += len(GRAPH.nodes[i].interval_nodes)
                    insert_index += GRAPH.nodes[split_node].split_index
                    GRAPH.dual_values.insert(insert_index,GRAPH.dual_values[insert_index])
                    #break
        #print "Current lower bound %.2f" %GRAPH.model.solution.get_objective_value()
        #print path
    
    if time.time()-t0 > time_limit:
        print "TIme limit reached"
    
    else:
        t=0
        """
        last = -1
        if path != -1:
            for p in path:
                if last!=-1:
                    t+=adj_matrix[last][p[0]]
                if t < TWs[p[0]][0]:
                    t= TWs[p[0]][0]
                #print "node : %d, TW = [%d,%d] at %d" %(p[0],TWs[p[0]][0],TWs[p[0]][1],t)
                last=p[0]
                    
            #print "location (%.2f,%.2f)" % 
        summe=0
        for j in GRAPH.nodes:
            summe+=len(j.interval_nodes)
        print "Time spent on solving lps with start: %.2f" % GRAPH.lp_time
        print "Total solve time: %.2f" % (time.time()-t0) 
        print "Number of points: %d" % summe
        print "Objective : %f" %GRAPH.model.solution.get_objective_value()
        #print solve_times
        print "Simplex iteras: %d, average simplex iteras: %.2f" %(int(sum(simplex_iteras)),float(sum(simplex_iteras))/len(simplex_iteras))
        print str(iteras)
        """
    timeDict[0].append(iteras)
    timeDict[1].append(GRAPH.lp_time)
    timeDict[2].append(sum(simplex_iteras))

solTimesMat = numpy.matrix([ timeDict[1] ])
iterasMat = numpy.matrix([ timeDict[0] ])
simplexIterasMat = numpy.matrix([ timeDict[2] ])
if GRAPH.expand:
    scipy.io.savemat('Results/worst_case_times_and_iteras_e', dict([('times',solTimesMat),('iteras',iterasMat) ,('simplexIteras',simplexIterasMat) ]))   
else:
    scipy.io.savemat('Results/worst_case_times_and_iteras', dict([('times',solTimesMat),('iteras',iterasMat) ,('simplexIteras',simplexIterasMat) ]))   