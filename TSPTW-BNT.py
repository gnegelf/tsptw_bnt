import cplex
import re
import time
import numpy
def readData(file_name,direcotry_name ="AFG"):
    
    print "reading "+file_name

    file = open(direcotry_name+"/"+file_name, "r")
    TSP_data = file.read()
    file.close()
    
    entries = re.split("\n+", TSP_data)
    
    vertNum = int(entries.pop(0))
    adj_matrix={}
    for i in range(vertNum):
        lineStr = entries.pop(0)
        lineStrList = re.split(" +", lineStr)
        #print lineStrList
        adj_matrix[i] = {}
        for j,val in enumerate(lineStrList):
            if i!=j:
                adj_matrix[i][j]= int(val) 
    TWs=[]
    for i in range(vertNum):
        lineStr = entries.pop(0)
        lineStrList = re.split(" +", lineStr)
        lineStrList.pop(-1)
        TWs.append([int(val) for val in lineStrList])
    return vertNum,TWs,adj_matrix

def rotateList(l, n):
    return l[n:] + l[:n]

def solToComponents(solutionStringList,depot=0):
    arcList=[]
    for string in solutionStringList:
        arcList.append([int(re.split("[_]",string)[1]),int(re.split("[_]",string)[2])])
    #notS = range(1,n)
    S=[0]
    while len(arcList)>0:
        popIndices=[]
        for i in range(len(arcList)):
            if arcList[i][0] in S:
                popIndices.append(i)
                if arcList[i][1] not in S:
                    S.append(arcList[i][1])
        if len(popIndices)== 0:
            break
        while len(popIndices)>0:
            arcList.pop(popIndices.pop(-1))
    return S

def solToPaths(solutionStringList,depot=0):
    pathList=[]
    for string in solutionStringList:
        pathList.append([int(re.split("[_]",string)[1]),int(re.split("[_]",string)[2])])
    prevLength=0
    curLength=len(pathList)
    while (prevLength!=curLength):
        prevLength=len(pathList)
        for i,path in enumerate(pathList):
            for j in range(len(pathList)):
                if i!=j:
                    if path[-1]==pathList[j][0]:
                        pathList[j].pop(0)
                        path+=pathList.pop(j)
                        break
                    else:
                        if pathList[j][0] == pathList[j][-1] : 
                            breaker=0
                            for num,vert in enumerate(path):
                                for num2,vert2 in enumerate(pathList[j]):
                                    if vert2==vert:
                                        tempPath=pathList.pop(j)
                                        tempPath.pop(-1)
                                        tempPath=rotateList(tempPath,num2)
                                        path[num:num]=tempPath
                                        
                                        breaker=1
                                        break
                                if breaker:
                                    break
                            if breaker:
                                break
                        
            curLength=len(pathList)
            if (curLength!=prevLength):
                break
    if len (pathList) == 1:
        depotInd = 0
        while depotInd<len(pathList[0])and pathList[0][depotInd] != depot:#needs fixing if depot is not chosen
            depotInd += 1
        pathList[0].pop(-1)
        pathList[0]=rotateList(pathList[0],depotInd)
        pathList[0].append(pathList[0][0])
    return pathList 

 
class Tsp():
    def __init__(self,nodes,adj_matrix,cost_matrix,depot,goal):
        self.nodes = [Tsp_node(self,node[0],node[1],node[2]) for node in nodes]
        self.adj_matrix = adj_matrix
        self.cost_matrix = cost_matrix
        #self.arc_list = [Arc(jj,ii,adj_matrix[i.name][j.name],cost_matrix[i.name][j.name]) 
        #    for i in self.nodes for ii in i.interval_nodes for j in self.nodes
        #        for jj in j.interval_nodes if i.name!= j.name and adj_matrix[i.name].has_key(j.name) and 
        #        jj.is_reachable(ii.interval,adj_matrix[i.name][j.name])]
        self.arc_list = [Arc(jj,ii,adj_matrix[i.name][j]) 
            for i in self.nodes for ii in i.interval_nodes for j in adj_matrix[i.name]
                for jj in self.nodes[j].interval_nodes if  
                jj.is_reachable(ii.interval,adj_matrix[i.name][j])]
        self.depot = depot
        self.goal = goal
        self.dual_values = []
        self.adapt_model = 1
    def create_model(self,var_type='C'):
        #t0 = time.time()
        self.model = cplex.Cplex()
        model = self.model
        model.set_results_stream(None)
        model.set_log_stream(None)
        self.x_names = ["x_%d_%d" %(i.name,j) for i in self.nodes for j in self.adj_matrix[i.name]]
        
        self.y_names = ["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                arc.tail.interval[0],arc.head.interval[0])
                        for node in self.nodes 
                        for interval_node in node.interval_nodes
                        for arc in interval_node.outgoing_arcs]
        self.z_names = ["z_%d_%d" % (node.name,interval_node.interval[0]) for node in self.nodes for interval_node in node.interval_nodes]

        
        
        all_names = self.x_names+self.y_names+self.z_names
        all_obj = [self.cost_matrix[i.name][j] for i in self.nodes for j in self.adj_matrix[i.name]]
        all_obj += [0.0]*(len(self.y_names)+len(self.z_names))
        
        all_lb = [0.0]*len(all_names)
        all_ub = [1.0]*len(all_names)
        
        model.variables.add(names = all_names,
                            types=[var_type]*len(all_names),obj=all_obj,
                            lb = all_lb,ub = all_ub)
        allvars = []
        allrhs = []
        allsenses = []
        all_names = []
        for node in self.nodes:
            thevars = ["z_%d_%d" %(node.name,interval_node.interval[0]) for interval_node in node.interval_nodes]
            thecoefs = [1]*len(thevars)
            allvars.append(cplex.SparsePair(thevars,thecoefs))
            allsenses.append("E")
            allrhs.append(1.0)
            all_names.append("visit_%d" % node.name)
        
        for node in self.nodes:
            for interval_node in node.interval_nodes:
                if node.name != self.goal:
                    thevars = ["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                   arc.tail.interval[0],arc.head.interval[0])
                                for arc in interval_node.outgoing_arcs]+["z_%d_%d" % (node.name,interval_node.interval[0])]
                    
                    thecoefs = [1]*len(interval_node.outgoing_arcs)+[-1]
                    
                        
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("E")
                    allrhs.append(0.0)
                    all_names.append("goout_%d_%d" %(node.name,interval_node.interval[0]))
                if node.name != self.depot:
                    thevars = ["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                   arc.tail.interval[0],arc.head.interval[0])
                                for arc in interval_node.ingoing_arcs]+["z_%d_%d" % (node.name,interval_node.interval[0])]
                    
                    thecoefs = [1]*len(interval_node.ingoing_arcs)+[-1]
    
                        
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("E")
                    allrhs.append(0.0)
                    all_names.append("goin_%d_%d" %(node.name,interval_node.interval[0]))
        
        for i in self.nodes:
            for j in self.adj_matrix[i.name]:
                if i.name!= j:
                    thevars = ["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                   arc.tail.interval[0],arc.head.interval[0])
                               for interval_node in i.interval_nodes 
                               for arc in interval_node.outgoing_arcs if arc.head.name == j]
                    thecoefs = [1]*len(thevars)
                    thevars += ["x_%d_%d" %(i.name,j)]
                    thecoefs += [-1]
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("E")
                    allrhs.append(0.0)
                    all_names.append("arcuse_%d_%d" %(i.name,j))
        
        model.linear_constraints.add(names=all_names,lin_expr = allvars, 
                                                     senses = allsenses, rhs = allrhs)
        self.const_num=len(allvars)
        
        self.names =  { n : j for j, n in enumerate(model.variables.get_names()) }
        self.var_num = len(self.names)
        if var_type == 'C':
            self.model.set_problem_type(0)
            self.model.parameters.lpmethod.set(2)
        #self.model_creation_time += time.time()-t0
        self.idx2name = { j : n for j, n in enumerate(model.variables.get_names()) }
        self.name2idx = { n : j for j, n in enumerate(model.variables.get_names()) }
    def solve_model(self,branches=[]):
        #self.lp_solves += 1
        #t0 = time.time()
        self.model.set_problem_type(0)
        self.model.solve()
        #self.lp_time += time.time()-t0
        #self.average_lp_time = self.lp_time / self.lp_solves
        primal_feasible = self.model.solution.is_primal_feasible()
        return primal_feasible
    def add_ste_cut(self,S):
        in_arc_list = []
        out_arc_list = []
        notS = []
        for i in range(0,len(self.nodes)):
            if i not in S:
                notS.append(i)
        for i in S:
            for j in notS:
                if self.adj_matrix[i].has_key(j):
                    out_arc_list.append("x_%d_%d" % (i,j))
                if self.adj_matrix[j].has_key(i):
                    in_arc_list.append("x_%d_%d" % (j,i))
        self.model.linear_constraints.add(lin_expr=[cplex.SparsePair(in_arc_list,[1.0]*len(in_arc_list))],senses=['G'],rhs=[1.0])
        self.model.linear_constraints.add(lin_expr=[cplex.SparsePair(out_arc_list,[1.0]*len(out_arc_list))],senses=['G'],rhs=[1.0])
    def has_expansion(self,path):
        arrival_time = self.nodes[path[0]].tw_lb
        for i in range(0,len(path)-1):
            earliest_arrival = max(self.nodes[path[i+1]].tw_lb,
                                   arrival_time+self.adj_matrix[path[i]][path[i+1]])
            #print [self.nodes[path[i+1]].tw_lb,arrival_time+self.adj_matrix[path[i]][path[i+1]],earliest_arrival,self.nodes[path[i+1]].tw_ub]
            if earliest_arrival-0.001 < self.nodes[path[i+1]].tw_ub:
                arrival_time = earliest_arrival
            else:
                return 0

        return 1
class Ste_cut():
    def __init__(self,arc_list,S,notS):
        self.arc_list = arc_list
        self.S = S
        self.notS = notS
            
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

class Tsp_node():
    def __init__(self,tsp,name,tw_lb,tw_ub):
        self.tsp = tsp
        self.name = name
        self.tw_lb = tw_lb
        self.tw_ub = tw_ub
        self.interval_nodes = [Interval_node(name,[tw_lb,tw_ub],tw_lb,tw_ub,1)]
        
    def split_nodeOld(self,split_point):
        idx = self.find_index(split_point)
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
                    self.tsp.arc_list.append(new_arc)
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
                    self.tsp.arc_list.append(new_arc)
                else:
                    is_reachable,new_head = self.tsp.nodes[arc.head.name].find_lowest_reachable(self.interval_nodes[idx+1].interval,
                                                arc.length,self.interval_nodes[idx+1].is_tw_ub())
                    if is_reachable:
                        new_arc = Arc(new_head,self.interval_nodes[idx+1],arc.length)
                        self.tsp.arc_list.append(new_arc)
                    
            arc_index += 1
        for i in pop_indices:
            self.interval_nodes[idx].outgoing_arcs.pop(i)
    def find_indexOld(self,split_point):
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
    def find_lowest_reachableOld(self,interval,step,ub_included=0):
        for interval_node in self.interval_nodes:
            if interval_node.is_lowest_reachable(interval,step,ub_included):
                return 1,interval_node
        return 0,-1
    def split_node(self,split_point):
        idx = self.find_index(split_point)
        self.split_index = idx
        old_ub = self.interval_nodes[idx].interval[1] 
        old_lb = self.interval_nodes[idx].interval[0]
        if idx < len(self.interval_nodes)-1:
            if abs(split_point - old_ub ) < 0.0001:
                return 1
        if (old_lb > split_point+0.0001
                or (split_point > old_ub+0.0001) or (abs(split_point - old_ub) < 0.0001 and abs(old_ub-self.tw_ub) > 0.0001)):
            print("Error: split point %f outside of node interval[%f,%f]" % (split_point,old_lb,old_ub) )
            return 0
        self.interval_nodes.insert(idx+1,Interval_node(self.name,[split_point,old_ub],self.tw_lb,self.tw_ub,self.interval_nodes[idx].is_ub))
        self.interval_nodes[idx].is_ub = 0
        self.interval_nodes[idx].interval[1] = split_point
        
        #TODO: fix types
        
        #add variable for new node
        if self.tsp.adapt_model:
            self.tsp.model.variables.add(names=["z_%d_%d" % (self.name,int(split_point))],types=['C'],lb=[0.0],ub=[1.0],obj=[0.0])
            self.tsp.var_num += 1
            self.tsp.names["z_%d_%d" % (self.name,int(split_point))] = self.tsp.var_num -1
            self.tsp.z_names.append("z_%d_%d" % (self.name,int(split_point)))
            #TODO korrekte syntax
            self.tsp.model.linear_constraints.set_coefficients([("visit_%d"% self.name,"z_%d_%d" %(self.name,int(split_point)),1.0) ])
            
            #generate constraints for new node
            self.tsp.model.linear_constraints.add(names=["goout_%d_%d" %(self.name,int(split_point))],
                                                         lin_expr=[cplex.SparsePair(["z_%d_%d" % (self.name,int(split_point))],[-1.0])],senses=["E"],rhs=[0.0])
            self.tsp.model.linear_constraints.add(names=["goin_%d_%d" %(self.name,int(split_point))],
                                                         lin_expr=[cplex.SparsePair(["z_%d_%d" % (self.name,int(split_point))],[-1.0])],senses=["E"],rhs=[0.0])
            
        pop_indices = []
        arc_index = 0
        
        for arc in self.interval_nodes[idx].ingoing_arcs:
            if not arc.head.is_lowest_reachable(arc.tail.interval,arc.length,arc.tail.is_tw_ub()):
                if self.tsp.adapt_model:  
                    old_var_name = "y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.interval[0],arc.head.interval[0])
                
                arc.head = self.interval_nodes[idx+1]
                self.interval_nodes[idx+1].ingoing_arcs.append(arc)
                if self.tsp.adapt_model:
                    const_name = "goin_%d_%d" % (self.name,self.interval_nodes[idx].interval[0])
                    self.tsp.model.linear_constraints.set_coefficients([(const_name,old_var_name,0.0)]) 
                    new_var_name = "y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.interval[0],
                                                      self.interval_nodes[idx+1].interval[0] )
                    const_name = "goin_%d_%d" % (self.name,self.interval_nodes[idx+1].interval[0])
                    self.tsp.model.linear_constraints.set_coefficients([(const_name,old_var_name,1.0)]) 
                    self.tsp.model.variables.set_names(old_var_name,new_var_name)
                    
                    self.tsp.names[new_var_name] = self.tsp.names.pop(old_var_name)
                    self.tsp.y_names.append(new_var_name)
                    self.tsp.y_names.remove(old_var_name)
                    #self.primal_values[new_var_name] = self.primal_values.pop(old_var_name)
                pop_indices.insert(0,arc_index)

            else:
                if self.interval_nodes[idx+1].is_lowest_reachable(arc.tail.interval,arc.length,
                                      arc.tail.is_tw_ub()):
                    print "I thought this was unreachable, adjust code here"
                    new_arc = Arc(self.interval_nodes[idx+1],arc.tail,arc.length)
                    self.graph.arc_list.append(new_arc)
                    if self.graph.adapt_model:
                        new_var_name = "y_%d_%d_%d_%d" % (new_arc.tail.name, new_arc.head.name,
                                                          new_arc.tail.interval[0],new_arc.head.interval[0])
                        self.graph.model.variables.add(names=[new_var_name],types=['C'],obj=[0.0],lb=[0.0],ub=[1.0])
                        self.graph.var_num += 1
                        self.graph.names[new_var_name] = self.graph.var_num -1
                        #self.graph.primal_values[new_var_name] = 0
            arc_index += 1
        for i in pop_indices:
            self.interval_nodes[idx].ingoing_arcs.pop(i)
        pop_indices = []
        arc_index = 0
        for arc in self.interval_nodes[idx].outgoing_arcs:
            if not arc.head.is_lowest_reachable(arc.tail.interval,arc.length,arc.tail.is_tw_ub()):
                if self.tsp.adapt_model:
                    new_var_name = "y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.interval[0],arc.head.interval[0])
                    old_var_name = "y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                      self.interval_nodes[idx+1].interval[0],arc.head.interval[0])
                    const_name = "goout_%d_%d" % (self.name,self.interval_nodes[idx].interval[0])
                    self.tsp.model.linear_constraints.set_coefficients([(const_name,old_var_name,0.0)])
                    const_name = "goout_%d_%d" % (self.name,self.interval_nodes[idx+1].interval[0])
                    self.tsp.model.linear_constraints.set_coefficients([(const_name,old_var_name,1.0)])
                    self.tsp.model.variables.set_names(old_var_name,new_var_name)
                    self.tsp.names[new_var_name] = self.tsp.names.pop(old_var_name)
                    self.tsp.y_names.append(new_var_name)
                    self.tsp.y_names.remove(old_var_name)
                    #self.tsp.primal_values[new_var_name] = self.tsp.primal_values.pop(old_var_name)
                arc.tail = self.interval_nodes[idx+1]
                self.interval_nodes[idx+1].outgoing_arcs.append(arc)
                pop_indices.insert(0,arc_index)
            else:
                if arc.head.is_lowest_reachable(self.interval_nodes[idx+1].interval,
                                                arc.length,self.interval_nodes[idx+1].is_tw_ub()):
                    new_arc = Arc(arc.head,self.interval_nodes[idx+1],arc.length)
                    self.tsp.arc_list.append(new_arc)
                    if self.tsp.adapt_model:
                        new_var_name = "y_%d_%d_%d_%d" % (new_arc.tail.name,new_arc.head.name,
                                                          new_arc.tail.interval[0],new_arc.head.interval[0])
                        self.tsp.model.variables.add(names=[new_var_name],types=['C'],obj=[0.0],lb=[0.0],ub=[1.0])
                        self.tsp.var_num += 1
                        self.tsp.names[new_var_name] = self.tsp.var_num -1
                        self.tsp.y_names.append(new_var_name)
                        #self.tsp.primal_values[new_var_name] = 0
                        const_name = "goin_%d_%d" % (new_arc.head.name,new_arc.head.interval[0])
                        self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)])
                        const_name = "goout_%d_%d" % (new_arc.tail.name,new_arc.tail.interval[0])
                        self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)]) 
                        const_name = "arcuse_%d_%d" % (new_arc.tail.name,new_arc.head.name)
                        self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)]) 
                else:
                    is_reachable,new_head = self.tsp.nodes[arc.head.name].find_lowest_reachable(self.interval_nodes[idx+1].interval,
                                                arc.length,self.interval_nodes[idx+1].is_tw_ub())
                    if is_reachable:
                        new_arc = Arc(new_head,self.interval_nodes[idx+1],arc.length)
                        self.tsp.arc_list.append(new_arc)
                        if self.tsp.adapt_model:
                            new_var_name = "y_%d_%d_%d_%d" % (new_arc.tail.name,new_arc.head.name,
                                                              new_arc.tail.interval[0],new_arc.head.interval[0])
                            self.tsp.model.variables.add(names=[new_var_name],types=['C'],obj=[0.0],lb=[0.0],ub=[1.0])
                            self.tsp.var_num += 1
                            self.tsp.names[new_var_name] = self.tsp.var_num -1
                            self.tsp.y_names.append(new_var_name)
                            #self.tsp.primal_values[new_var_name] = 0
                            const_name = "goin_%d_%d" % (new_arc.head.name,new_arc.head.interval[0])
                            self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)])  
                            const_name = "goout_%d_%d" % (new_arc.tail.name,new_arc.tail.interval[0])
                            self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)])
                            const_name = "arcuse_%d_%d" % (new_arc.tail.name,new_arc.head.name)
                            self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)]) 
            arc_index += 1
        for i in pop_indices:
            self.interval_nodes[idx].outgoing_arcs.pop(i)
        #TODO: check if this is really unnecessary
        if self.tsp.adapt_model and 0:
            allvars = []
            allrhs = []
            allsenses = []
            allnames = []
            interval_node = self.interval_nodes[idx+1]
            thevars = (["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                           arc.tail.interval[0],arc.head.interval[0])
                        for arc in interval_node.outgoing_arcs]+
                        ["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
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
                
class Branch():
    def __init__(self,var,sense,rhs):
        self.var = var
        self.name= var + str(rhs)
        self.sense = sense
        self.rhs = rhs
        self.lin_expr = cplex.SparsePair([var],[1])  
    def __repr__(self):
        return self.var +" sense: " +(self.sense)
    def __str__(self):
        return self.var +" sense: " +(self.sense)           

class Arc():
    def __init__(self,head,tail,length):
        self.head = head
        self.head.ingoing_arcs.append(self)
        self.tail = tail
        self.tail.outgoing_arcs.append(self)
        self.length = length


class Tree():
    def __init__(self,tsp,start_control):
        self.start_control = start_control
        self.branch_variable_selection_time = 0.0
        self.node_selection_time = 0.0
        self.total_relaxation_time = 0.0
        self.closed_nodes = []
        self.tsp = tsp
        self.ub = 40094.1
        self.root = Tree_node(self,[])
        self.open_nodes = [self.root]
        
        self.branch_history = {key:[] for key in tsp.x_names}
        self.cutFinder = cutFinder(self.tsp.adj_matrix)
        
    def branch_and_split(self):
        count=0
        breaker = 1
        oldLb=0.0
        while len(self.open_nodes)> 0 and breaker:
            node = self.choose_node()
            if len(self.open_nodes)<10:
                splitNodes=1
            else:
                splitNodes=0
            if len(self.open_nodes)<50:
                addCut=1
            else:
                addCut=0
            print node.lower_bound
            print "Open nodes: %d" % len(self.open_nodes)
            cutAdded = 0
            count+=1
            comp = solToComponents(node.find_set_x_vars())
            #print comp
            if len(comp)!=len(self.tsp.nodes):
                cutAdded = 1
                self.tsp.add_ste_cut(comp)
            else:
                if len(node.fractionals) > 0 and addCut:
                    #ms="".join("" if val<0.001 else key+" %.3f,"%val for key,val in node.primal_y_values.iteritems())
                    #print ms
                    for i in range(1,len(self.tsp.adj_matrix)-1):
                        S = self.cutFinder.findCut(i,len(self.tsp.nodes)-1,node.primal_x_values)
                        if S!=-1:
                            print(S)
                            cutAdded = 1
                            self.tsp.add_ste_cut(S)
                            break
            if count > 10006:
                print "count exceeded"
                break
            if cutAdded:
                print "adding cut"
                pop_indices=[]
                self.open_nodes.append(node)
                for i,node2 in enumerate(self.open_nodes):
                    node2.solve_lp_relaxation()
                    if not node2.feasible or node2.lower_bound >= self.ub:
                        pop_indices.append(i)
                while (len(pop_indices)>0):
                    self.open_nodes.pop(pop_indices.pop(-1))
                continue
            else:
                #break
                #print node.primal_z_values
                #print node.primal_x_values
                #print solToPaths(node.find_set_x_vars())
                times,split_points= find_times(len(self.tsp.nodes),node.primal_x_values,node.primal_y_values,self.tsp.adj_matrix)
                split = 0
                times.pop(-1)
                #print times
                newLb=node.lower_bound
                if len(node.fractionals)==0 or (splitNodes and abs(oldLb-newLb)>0.001):
                    for i,t in enumerate(times):
                        if t> self.tsp.nodes[i].tw_ub:
                            split = 1
                            break
                    if split ==1:
                        for i,t in enumerate(times):
                            if split_points[i][0]>0:
                                self.tsp.nodes[i].split_node(split_points[i][0])
                                
                    
                    #print split_points
                    #print(solToPaths(node.find_set_x_vars()))
                    #print(count)
                    #print(node.fractionals)
                    #print split
                    
                    if split == 0:
                        oldLb = node.lower_bound
                        timefeasible=1
                    else:
                        oldLb = node.lower_bound
                        print "Splitting nodes"
                        pop_indices=[]
                        self.open_nodes.append(node)
                        for i,node2 in enumerate(self.open_nodes):
                            node2.solve_lp_relaxation()
                            if not node2.feasible or node2.lower_bound >= self.ub:
                                pop_indices.append(i)
                        while (len(pop_indices)>0):
                            self.open_nodes.pop(pop_indices.pop(-1))
                        continue
                else:
                    oldLb = node.lower_bound
                    split=0
                    timefeasible=0
            if split==0:
                if len(node.fractionals) == 0 and timefeasible:
                    #updatebound
                    print "integer feasible solution found, objective: %f" %node.lower_bound
                    self.ub = node.lower_bound
                    self.solution=node.primal_x_values
                    self.y_solution=node.primal_y_values
                    self.sol_arr_times=times
                    self.y_fracs=node.y_fractionals
                    self.split_points=split_points
                    pop_indices=[]
                    for i,node2 in enumerate(self.open_nodes):
                        #node2.solve_lp_relaxation()
                        if not node2.feasible or node2.lower_bound >= self.ub:
                            pop_indices.append(i)
                    while (len(pop_indices)>0):
                        self.open_nodes.pop(pop_indices.pop(-1))
                    continue
                else:
                    branch_var,branch_val = node.choose_branch_var()
                    print "branching"
                    f_1 = 1.0-branch_val
                    f_0 = branch_val
                    new_node_list = [Tree_node(node.tree,node.branches+[Branch(branch_var,'L',0.0)]),
                                     Tree_node(node.tree,node.branches+[Branch(branch_var,'G',1.0)])]
                    if new_node_list[0].feasible and new_node_list[1].feasible:
                        convex_factor = 0.1
                        c_1 = (new_node_list[1].lower_bound-node.lower_bound)/f_1
                        c_0 = (new_node_list[0].lower_bound-node.lower_bound)/f_0
                        frac_1 = -1.0*(len(new_node_list[1].fractionals)-len(node.fractionals))/f_1
                        frac_0 = -1.0*(len(new_node_list[0].fractionals)-len(node.fractionals))/f_0
                        if frac_1 < 0.0 or frac_0 < 0.0:
                            convex_factor = 1.0
                        c_1 = convex_factor * c_1+(1-convex_factor)* frac_1
                        c_0 = convex_factor * c_0+(1-convex_factor)* frac_0
                        self.branch_history[branch_var].append((c_0,c_1))
                for new_node1 in new_node_list:
                    if new_node1.feasible:
                        if new_node1.lower_bound < self.ub-0.5:
                            self.open_nodes.append(new_node1)
            else:
                continue
            #"""    
    def choose_node(self,selection=1):
        t0=time.time()
        if selection == 1:
            minInd = 0
            minVal= 100000000
            for i,node in enumerate(self.open_nodes):
                if node.lower_bound < minVal:
                    minInd = i
                    minVal = node.lower_bound
        if selection == 0:
            minInd = 0
        if selection == 2:
            minInd = 0
            minVal= 100000000
            for i,node in enumerate(self.open_nodes):
                if len(node.fractionals)+node.lower_bound < minVal:
                    minInd = i
                    minVal = len(node.fractionals)+node.lower_bound
        if selection == 3:
            minInd = 0
            minVal= 100000000
            for i,node in enumerate(self.open_nodes):
                if len(node.fractionals)+0.2*node.lower_bound < minVal:
                    minInd = i
                    minVal = len(node.fractionals)+0.2*node.lower_bound
        self.node_selection_time += time.time() - t0
        return self.open_nodes.pop(minInd)

class Tree_node():
    def __init__(self,tree,branches,dual_values=0,primal_values = 0,slacks = 0,red_costs = 0):
        self.tree = tree
        self.descendents = {}
        self.branches = branches
        self.branch_names = [branch.name for branch in self.branches]
        self.branch_lin_exprs = [branch.lin_expr for branch in self.branches]
        self.branch_senses = [branch.sense for branch in self.branches]
        self.branch_rhs = [branch.rhs for branch in self.branches]
        self.dual_values = dual_values
        self.primal_values = primal_values
        self.slacks = slacks
        self.red_costs = red_costs
        self.branch_var = -1
        self.lower_bound = 0
        
        if self.solve_lp_relaxation() and self.lower_bound < self.tree.ub:
            self.status = 1
        else:
            self.status = 0
    def solve_lp_relaxation(self,warm_start=1):
        #t0 = time.time()
        tsp = self.tree.tsp
        model = tsp.model
        model.linear_constraints.add(names = self.branch_names,lin_expr = self.branch_lin_exprs,
                                         senses = self.branch_senses,rhs = self.branch_rhs)
        if self.tree.start_control and warm_start and self.dual_values != 0 and self.primal_values != 0:
            model.start.set_start(col_status=[],
                     row_status=[],
                     #col_primal=self.primal_values,

                     row_dual=self.dual_values,
                     col_primal=[],
                     row_primal=[],
                     col_dual=[],
                     #row_dual=[]
                     )
        
        self.feasible = tsp.solve_model()
        #t0 = time.time()
        solution = model.solution
        #self.feasible = solution.is_primal_feasible()
        self.fractionals = {}
        self.y_fractionals = {}
        self.branch_val = -1
        if self.feasible:
            self.dual_values = solution.get_dual_values()
            self.primal_values = solution.get_values()
            self.lower_bound = solution.get_objective_value()
            #self.slacks = solution.get_linear_slacks()
            #self.red_costs = solution.get_reduced_costs()
            #t0 = time.time()
            self.primal_x_values = {name:self.primal_values[self.tree.tsp.names[name]] for name in self.tree.tsp.x_names}
            self.primal_y_values = {name:self.primal_values[self.tree.tsp.names[name]] for name in self.tree.tsp.y_names}
            self.primal_z_values = {name:self.primal_values[self.tree.tsp.names[name]] for name in self.tree.tsp.z_names}
            #self.primal_x_values = dict(zip(self.tree.tsp.x_names,solution.get_values(self.tree.tsp.x_names)))
            #self.primal_y_solution = dict(zip(self.tree.tsp.y_names,solution.get_values(self.tree.tsp.y_names)))
            
            for key,val in self.primal_x_values.iteritems():
                if abs(0.5-val)<0.4999:
                    self.branch_val = val
                    self.branch_var = key
                    self.fractionals[key] = val
            for key,val in self.primal_y_values.iteritems():
                if abs(0.5-val)<0.4999:
                    self.branch_val = val
                    self.branch_var = key
                    self.y_fractionals[key] = val
        model.linear_constraints.delete(xrange(model.linear_constraints.get_num()-len(self.branches),model.linear_constraints.get_num()))
        #self.tree.total_relaxation_time += time.time() - t0
        return self.feasible
    def is_x_integer(self):
        if len(self.fractionals.values()) == 0:
            return 1
        else:
            return 0
    def find_set_x_vars(self):
        return [var_name for var_name,val in self.primal_x_values.iteritems() if val > 0.001]
    def find_split_points(self):
        split_points = []
        for key,val in self.primal_x_values.iteritems():
            if val >0.999:
                strings = re.split("[_]",key)
                tail = int(strings[1])
                head = int(strings[2])
                #this way of determining lbs is bad
                lb1 = float(strings[3])
                lb2 = float(strings[4])
                if lb1+self.tree.tsp.adj_matrix[tail][head] > lb2+0.0001:
                    split_points += [(head,lb1+self.tree.tsp.adj_matrix[tail][head])]
        return split_points
    def choose_branch_var(self):
        #return self.branch_var, self.branch_val
        #t0 = time.time()
        max_score = -100
        chosen_var = -1
        chosen_val = -1
        for var_name,var_val in self.fractionals.iteritems():
            score = self.calc_score(var_name,var_val,1.0-var_val)
            if score > max_score:
                max_score = score
                chosen_var = var_name
                chosen_val = var_val
        #self.tree.branch_variable_selection_time += time.time()-t0
        return chosen_var, chosen_val
    def calc_score(self,var_name,f_0,f_1):
        branch_history = self.tree.branch_history
        if len(branch_history[var_name])>0:
            psi_0 = 0.0
            psi_1 = 0.0
            for tup in branch_history[var_name]:
                psi_0 += tup[0]
                psi_1 += tup[1]
            psi_0 = psi_0 /len(branch_history[var_name])
            psi_1 = psi_1 /len(branch_history[var_name])
            return (5.0/6)*min(f_0*psi_0,f_1*psi_1)+1.0/6*max(f_0*psi_0,f_1*psi_1)
        else:
            val_set = 0
            average_1 = 0.0
            average_0 = 0.0
            for var_name in branch_history:
                if len(branch_history[var_name])>0:
                    val_set += 1
                    psi_0 = 0.0
                    psi_1 = 0.0
                    for tup in branch_history[var_name]:
                        psi_0 += tup[0]
                        psi_1 += tup[1]
                    psi_0 = psi_0 /len(branch_history[var_name])
                    psi_1 = psi_1 /len(branch_history[var_name])
                    average_1 += psi_1
                    average_0 += psi_0
            
            if val_set > 0:
                average_1 = average_1 /val_set
                average_0 = average_0 /val_set
            else:
                average_1 = 1.0
                average_0 = 1.0
            return (5.0/6)*min(f_0*average_0,f_1*average_1)+1.0/6*max(f_0*average_0,f_1*average_1)

class cutFinder():
    def __init__(self,adj_matrix):
        self.adj_matrix = adj_matrix
        self.createFlowModel(adj_matrix,len(adj_matrix))
    def findCut(self,s,t,capacities):
        s_coefs = self.model.linear_constraints.get_rows("flow_%d" % s)
        t_coefs = self.model.linear_constraints.get_rows("flow_%d" % t)
        self.model.linear_constraints.delete("flow_%d" % s)
        self.model.linear_constraints.delete("flow_%d" % t)
        self.dual_name2idx = { n : j for j, n in enumerate(self.model.linear_constraints.get_names()) }
        for key,val in capacities.iteritems():
            self.model.linear_constraints.set_rhs('c'+key[1:],val)
        self.model.set_problem_type(0)
        self.model.solve()
        dual_values = self.model.solution.get_dual_values()
        obj = self.model.solution.get_objective_value()

        cut_S = [s]
        indices = range(0,len(self.adj_matrix)-1)
        indices.pop(s)
        if obj < 0.9999:
            #print dual_values
            print obj
            #for p,name in enumerate(self.model.variables.get_names()):
            #    if self.model.solution.get_values(name)> 0.01:
            #        print name +": " + str(self.model.solution.get_values(name))
            print "goal cannot be reached from %d at full capacity" % s
            for index in indices:
                if dual_values[self.dual_name2idx["flow_%d"%index]]> 0.001:
                    continue
                else:
                    cut_S.append(index)
        else:
            cut_S=-1
        self.model.linear_constraints.add(names=["flow_%d" % s],lin_expr=[s_coefs],rhs=[0],senses=["E"])
        self.model.linear_constraints.add(names=["flow_%d" % t],lin_expr=[t_coefs],rhs=[0],senses=["E"])
        self.dual_name2idx = { n : j for j, n in enumerate(self.model.linear_constraints.get_names()) }
        return cut_S
    def createFlowModel (self,adj_matrix,n):
        model = cplex.Cplex()
        self.model=model
        model.set_results_stream(None)
        model.set_log_stream(None)
        model.objective.set_sense(model.objective.sense.maximize)
        for i in adj_matrix:
            for j in adj_matrix[i]:
                if j == len(adj_matrix)-1:
                    obj = 1.0
                else:
                    obj = 0.0
                model.variables.add(names=["f_%d_%d" %(i,j)],types=["C"],obj=[obj],lb=[0.0])
                model.linear_constraints.add(names=["c_%d_%d" %(i,j)],lin_expr = [
                                                    cplex.SparsePair(["f_%d_%d" %(i,j)],[1.0])],senses=["L"],rhs=[0.0])
                
        for i in adj_matrix:
            thevars = ["f_%d_%d" %(i,j) for j in adj_matrix[i]]
            thecoefs = [1.0 for j in adj_matrix[i]]
            for j in adj_matrix:
                for k in adj_matrix[j]:
                    if k == i:
                        thevars+=["f_%d_%d" %(j,k)]
                        thecoefs+=[-1.0]
            model.linear_constraints.add(names=["flow_%d" %i ],lin_expr = [cplex.SparsePair(thevars,thecoefs)],senses=["E"],rhs=[0.0])
        self.dual_name2idx = { n : j for j, n in enumerate(model.linear_constraints.get_names()) }
        return model

#finds the times for a fractional solution where y_frac 
#has nodes as keys and dictionarys values with tuples as keys and the fracs as values
def find_times(n,x_frac,y_frac,adj_matrix):
    A = numpy.zeros((n,n))
    b = numpy.zeros(n)
    for j in range(n):
        A[j,j] = 1
        for i in range(n):
            if j in adj_matrix[i]:
                A[j,i] -= x_frac["x_%d_%d" %(i,j)]
    split_points={i:(-1,0) for i in range(n)}
    for key,val in y_frac.iteritems():
        i,j,lb_1,lb_2=(int(re.split("[_]",key)[1]),int(re.split("[_]",key)[2]),int(re.split("[_]",key)[3]),int(re.split("[_]",key)[4]))
        if val>0.0001:
            if lb_2-lb_1>adj_matrix[i][j]:
                b[j] += (lb_2-lb_1)*val
            else:
                b[j] += adj_matrix[i][j]*val
                arrTime = lb_1+adj_matrix[i][j]
                score=(lb_1+adj_matrix[i][j]-lb_2)*val
                if score > split_points[j][1]:
                    split_points[j] = (arrTime,score)
            
    #print A.tolist()[-1]
    #print b.tolist()[-1]
    times = numpy.linalg.solve(A,b)
    times = times.tolist()
    return times,split_points
        
                    
#instance_name = "n40w60.001.txt"
instance_name = "rbg048a.tw"
vert_num,TWs,adj_matrix = readData(instance_name,"AFG")
#instance_name = "testCase7_1"
#vert_num,TWs,adj_matrix = readData(instance_name,"TSPInstances")
vert_num += 1
TWs.append(TWs[0])
adj_matrix[vert_num-1] = {}
nodes = [[i,TWs[i][0],TWs[i][1]] for i in range(vert_num)]
for i in  range(vert_num):
    if adj_matrix[i].has_key(0):
        adj_matrix[i][vert_num-1]=adj_matrix[i].pop(0)
    if adj_matrix[i].has_key(i):
        adj_matrix[i].pop(i)
for i in  range(vert_num):
    for j in range(vert_num):
        if adj_matrix[i].has_key(j):
            if TWs[i][0]+adj_matrix[i][j]>TWs[j][1]:
                adj_matrix[i].pop(j)
tsp = Tsp(nodes,adj_matrix,adj_matrix,0,vert_num-1)

"""
tsp.adapt_model = 0
for i in range(1,len(TWs)):
    for j in range(TWs[i][0]+1,TWs[i][1]+1,1):
        tsp.nodes[i].split_node(j)
#"""
tsp.create_model()
tree = Tree(tsp,0)
t0=time.time()
tree.branch_and_split()
t1=time.time()
print "Total time: %f" %(t1-t0)
