import cplex
import re
import time


def readData(file_name):
    comment_line = re.compile('#')
    
    print "reading '"+file_name

    file = open("AFG/"+file_name, "r")
    TSP_data = file.read()
    file.close()
    
    entries = re.split("\n+", TSP_data)
    
    vertNum = int(entries.pop(0))
    adj_matrix=[]
    for i in range(vertNum):
        lineStr = entries.pop(0)
        lineStrList = re.split(" +", lineStr)
        adj_matrix.append([int(val) for val in lineStrList])
    TWs=[]
    for i in range(vertNum):
        lineStr = entries.pop(0)
        lineStrList = re.split(" +", lineStr)
        lineStrList.pop(-1)
        TWs.append([int(val) for val in lineStrList])
    nodes = [[i,TWs[i][0],TWs[i][1]] for i in range(vertNum)]
    return vertNum,TWs,adj_matrix,nodes

def rotateList(l, n):
    return l[n:] + l[:n]
def solToPaths(solutionStringList):
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

    return pathList 

 
class Tsp():
    def __init__(self,nodes,adj_matrix,depot=-1):
        self.nodes = [Tsp_node(self,node[0],node[1],node[2]) for node in nodes]
        self.adj_matrix = adj_matrix
        self.arc_list = [Arc(jj,ii,adj_matrix[i.name][j.name]) 
            for i in self.nodes for ii in i.interval_nodes for j in self.nodes
                for jj in j.interval_nodes if i.name!= j.name and adj_matrix[i.name][j.name] > -0.5 and 
                jj.is_reachable(ii.interval,adj_matrix[i.name][j.name])]
        self.depot=depot
        self.ste_cuts = []
    def create_model(self,var_type='C'):
        self.model = cplex.Cplex()
        model = self.model
        self.x_names = []
        #variable declaration
        for node in self.nodes:
            for interval_node in node.interval_nodes:
                for arc in interval_node.outgoing_arcs:
                    names = ["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                                    arc.tail.interval[0],arc.head.interval[0])]
                    self.x_names += names
                    model.variables.add(names = names,
                                        types=[var_type],obj=[0.0],lb = [0.0],ub = [1.0])
                if self.depot == -1 or node.name == self.depot:
                    model.variables.add(names = ["start%d_%d" % (node.name,interval_node.interval[0])],
                                            types=[var_type],lb = [0.0],ub = [1.0])
                    model.variables.add(names = ["end%d_%d" % (node.name,interval_node.interval[0])],
                                            types=[var_type],lb = [0.0],ub = [1.0])
                else:
                    model.variables.add(names = ["start%d_%d" % (node.name,interval_node.interval[0])],
                                            types=[var_type],lb = [0.0],ub = [0.0])
                    model.variables.add(names = ["end%d_%d" % (node.name,interval_node.interval[0])],
                                            types=[var_type],lb = [0.0],ub = [0.0])
        self.y_names = []
        for node in self.nodes:
            for j in self.nodes:
                if node.name!=j.name and self.adj_matrix[node.name][j.name] > -0.5:
                    names = ["y_%d_%d" % (node.name,j.name)]
                    self.y_names += names
                    model.variables.add(names = names,
                                        types=[var_type],obj=[self.adj_matrix[node.name][j.name]],
                                        lb = [0.0],ub = [1.0])
        if self.depot == -1:
            thevars = ["start%d_%d" % (node.name,interval_node.interval[0]) 
                            for node in self.nodes for interval_node in node.interval_nodes]
            thecoefs = [1]*len(thevars)
            model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                                     senses = ["E"], rhs = [1.0])
        else:
            depot=self.nodes[self.depot]
            thevars = ["start%d_%d" % (depot.name,interval_node.interval[0]) 
                            for interval_node in depot.interval_nodes]
            thecoefs = [1]*len(thevars)
            model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                                     senses = ["E"], rhs = [1.0])
        
        for node in self.nodes:
            thevars = (["start%d_%d" % (node.name,interval_node.interval[0]) 
                        for interval_node in node.interval_nodes]+
                        ["end%d_%d" % (node.name,interval_node.interval[0]) 
                        for interval_node in node.interval_nodes]
                        )
            thecoefs = ([1 for interval_node in node.interval_nodes]+
                        [-1 for interval_node in node.interval_nodes]
                        )
            model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                                     senses = ["E"], rhs = [0.0])
        
        for node in self.nodes:
            for interval_node in node.interval_nodes:
                thevars = (["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                               arc.tail.interval[0],arc.head.interval[0])
                            for arc in interval_node.outgoing_arcs]+
                            ["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                arc.tail.interval[0],arc.head.interval[0])
                            for arc in interval_node.ingoing_arcs]+
                            ["start%d_%d" % (node.name,interval_node.interval[0])]+
                            ["end%d_%d" % (node.name,interval_node.interval[0])]
                            )
                
                thecoefs = [-1]*len(interval_node.outgoing_arcs)+[1]*len(interval_node.ingoing_arcs)+[1,-1]

                    
                model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                                         senses = ["E"], rhs = [0.0])
                
            thevars = (["y_%d_%d" %(node.name,j.name)
                        for j in self.nodes if node.name!=j.name and self.adj_matrix[node.name][j.name] > -0.5]
                        )
            thecoefs = [1]*len(thevars)

            model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                                     senses = ["E"], rhs = [1.0])
            for j in self.nodes:
                if node.name!=j.name and self.adj_matrix[node.name][j.name] > -0.5:
                    thevars = (["x_%d_%d_%d_%d" %(arc.tail.name,arc.head.name,
                                                  arc.tail.interval[0],arc.head.interval[0])
                                for interval_node in node.interval_nodes
                                for arc in interval_node.outgoing_arcs if arc.head.name == j.name]
                                )
                    thecoefs = [1]*len(thevars)
                    thevars += ['y_%d_%d' % (node.name,j.name)]
                    thecoefs += [-1]
                    model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                                             senses = ["E"], rhs = [0.0])   
        for cut in self.ste_cuts:
            thevars = [arc_var_name for arc_var_name in cut.arc_list]
            thecoefs = [1.0 for arc_var_name in cut.arc_list]
            model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], 
                                                     senses = ["G"], rhs = [2.0])
            self.names =  { n : j for j, n in enumerate(TSP.model.variables.get_names()) }
    def solve_model(self,branches=[]):
        
        self.model.solve()
        primal_feasible = self.model.solution.is_primal_feasible()
        return primal_feasible
    def add_ste_cut(self,pathList):
        arc_list = {}
        S=pathList[0]
        notS = []
        for i in range(1,len(pathList)):
            notS += pathList[i]
        for i in S:
            for j in notS:
                if self.adj_matrix[i][j] > -0.5:
                    arc_list["y_%d_%d" % (i,j)] = 1
                if self.adj_matrix[j][i] > -0.5:
                     arc_list["y_%d_%d" % (j,i)] = 1
        self.ste_cuts += [Ste_cut(arc_list,S)]
    def has_expansion(self,path):
        arrival_time = self.nodes[path[0]].tw_lb
        for i in range(0,len(path)-1):
            earliest_arrival = max(self.nodes[path[i+1]].tw_lb,
                                   arrival_time+self.adj_matrix[path[i]][path[i+1]])
            print [self.nodes[path[i+1]].tw_lb,arrival_time+self.adj_matrix[path[i]][path[i+1]],earliest_arrival,self.nodes[path[i+1]].tw_ub]
            if earliest_arrival-0.001 < self.nodes[path[i+1]].tw_ub:
                arrival_time = earliest_arrival
            else:
                return 0

        return 1
class Ste_cut():
    def __init__(self,arc_list,S):
        self.arc_list = arc_list
        self.S = S
            
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

class Tsp_node():
    def __init__(self,tsp,name,tw_lb,tw_ub):
        self.tsp = tsp
        self.name = name
        self.tw_lb = tw_lb
        self.tw_ub = tw_ub
        self.interval_nodes = [Interval_node(name,[tw_lb,tw_ub],tw_lb,tw_ub)]
        
    def split_node(self,split_point):
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
    def __init__(self,tsp):
        self.closed_nodes = []
        self.tsp = tsp
        self.ub = 1305.0
        self.root = Tree_node(self,[])
        self.open_nodes = [self.root]
    def choose_node(self):
        minInd = 0
        minVal= 100000000
        for i,node in enumerate(self.open_nodes):
            if node.lower_bound < minVal:
                minInd = i
                minVal = node.lower_bound
        #minInd = 0
        return self.open_nodes.pop(minInd)
        

class Tree_node():
    def __init__(self,tree,branches):
        self.tree = tree
        self.descendents = {}
        self.branches = branches
        self.branch_var = -1
        if self.solve_lp_relaxation() and self.lower_bound < self.tree.ub:
            self.status = 1
        else:
            self.status = 0
    def solve_lp_relaxation(self):
        tsp = self.tree.tsp
        model = tsp.model
        for branch in self.branches:
            model.linear_constraints.add(names = [branch.name],lin_expr = [branch.lin_expr],
                                         senses = [branch.sense],rhs = [branch.rhs])
        self.feasible = tsp.solve_model()
        
        solution = model.solution
        #self.feasible = solution.is_primal_feasible()

        if self.feasible:
            self.lower_bound = solution.get_objective_value()
            self.primal_x_values = dict(zip(self.tree.tsp.x_names,solution.get_values(self.tree.tsp.x_names)))
            self.primal_y_solution = dict(zip(self.tree.tsp.y_names,solution.get_values(self.tree.tsp.y_names)))
        for branch in self.branches:
            model.linear_constraints.delete(branch.name)
        return self.feasible
    def is_y_integer(self):
        for key,val in self.primal_y_solution.iteritems():
            if abs(0.5-val)<0.4999:
                self.branch_val = val
                self.branch_var = key
                return 0
        return 1
    def find_set_y_vars(self):
        return [var_name for var_name,val in self.primal_y_solution.iteritems() if val > 0.999]
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
        return self.branch_var

instance_name="rbg010a.tw"
vert_num,TWs,adj_matrix,nodes = readData(instance_name)

TSP = Tsp(nodes,adj_matrix,0)

TSP.create_model()
bnt_tree = Tree(TSP)

TSP.solve_model()
bnt_tree.root.is_y_integer() 
iteras = 0
tree_sizes=[]
branchVars = []
t0 = time.time()
time_spent = 0.0
while len(bnt_tree.open_nodes)>0:
    iteras += 1
    t1 = time.time()
    print "selecting node"
    node = bnt_tree.choose_node()
    tree_sizes.append([len(bnt_tree.open_nodes)])
    branch_var = node.choose_branch_var()
    if branch_var != -1:
        branchVars += [branch_var,node.branch_val]
    transform = 0
    if branch_var != -1:
        new_node_list = [Tree_node(node.tree,node.branches+[Branch(branch_var,'L',0.0)]),
                         Tree_node(node.tree,node.branches+[Branch(branch_var,'G',1.0)])]
    else:
        new_node_list = [node]
    firstNewNode=new_node_list[0]
    new_ub = 0
    for new_node1 in new_node_list:
        if new_node1.feasible:
            if new_node1.is_y_integer():
                paths = solToPaths(new_node1.find_set_y_vars())
                if len(paths) > 1:
                    bnt_tree.open_nodes.append(node)
                    bnt_tree.tsp.add_ste_cut(paths)
                    transform = 1
                    break
                else:
                    depotInd = 0
                    if TSP.depot != -1:
                        while paths[0][depotInd] != TSP.depot:#needs fixing if depot is not chosen
                            depotInd += 1
                        paths[0].pop(-1)
                        paths[0]=rotateList(paths[0],depotInd)
                        paths[0].append(paths[0][0])
                    if bnt_tree.tsp.has_expansion(paths[0]):
                        if bnt_tree.ub-0.0001 > new_node1.lower_bound:
                            new_ub = 1
                            bnt_tree.ub = new_node1.lower_bound
                    else:
                        transform = 1
                        sps = new_node1.find_split_points()
                        for sp in sps:
                            bnt_tree.tsp.nodes[sp[0]].split_node(sp[1])
                        bnt_tree.open_nodes.append(node)
                        break
            else:
                if bnt_tree.ub-0.0001 > new_node1.lower_bound:
                    bnt_tree.open_nodes.append(new_node1)
    #it might be necessary to add new nodes to the tree if tree transformation does not cut them off
    if new_ub:
        pop_indices2 = []
        for i,node in enumerate(bnt_tree.open_nodes):
            if not (node.feasible and node.lower_bound < bnt_tree.ub-0.0001):
                pop_indices2.insert(0,i)
        for ind in pop_indices2:
            bnt_tree.open_nodes.pop(ind)
    time_spent += time.time()-t1
    if transform:
        TSP.create_model()
        bnt_tree = Tree(TSP)
        
        TSP.solve_model()
        bnt_tree.root.is_y_integer()
comp_time = time.time()-t0
"""
idx2name = { j : n for j, n in enumerate(TSP.model.variables.get_names()) }
solutionStringList = []
for i,val in enumerate(TSP.model.solution.get_values()):
    if val >0.5:
        if len(re.split("[_]",idx2name[i])) == 5:
            solutionStringList.append(idx2name[i])
        print(idx2name[i])

print solToPaths(solutionStringList)
"""
TSP.create_model('B')
TSP.solve_model()
print TSP.model.solution.get_objective_value()
print bnt_tree.ub
print "iteras: %d" % iteras
print comp_time
print str(tree_sizes)
print time_spent