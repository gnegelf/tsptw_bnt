import cplex
import re


class Tree():
    def __init__(self,root):
        self.root = root
        self.open_nodes = [root]
        self.closed_nodes = []

class Tree_node():
    def __init__(self,tree,parent,extra_inequalities,desc_index,depth):
        self.tree = tree
        self.parent = parent
        self.depth = depth
        self.descendents = {}
        self.desc_index = desc_index
        self.extra_inequalities = extra_inequalities
        if self.solve_lp_relaxation() and self.lower_bound < self.tree.upper_bound:
            self.status = 1
        else:
            self.status = 0
    def solve_lp_relaxation(self):
        if not self.primal_feasible:
            return 0
        self.lower_bound = solval
        self.primal_solution = sol
        self.dual_solution = dual
        return 1
    def transform(self,new_inequalities):
        self.extra_inequalties = new_inequalities
        if self.solve_lp_relaxation() and self.lower_bound < self.tree.upper_bound:
            self.status = 1
        else:
            self.status = 0

def rotateList(l, n):
    return l[n:] + l[:n]
def solToPaths(solutionStringList):
    pathList=[]
    for string in solutionStringList:
        pathList.append([(re.split("[_]",string)[1]),(re.split("[_]",string)[2])])
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
    def create_model(self,var_type='C'):
        self.model = cplex.Cplex()
        model = self.model
        #variable declaration
        for node in self.nodes:
            for interval_node in node.interval_nodes:
                for arc in interval_node.outgoing_arcs:
                    model.variables.add(names = ["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.interval[0],arc.head.interval[0])],
                                        types=[var_type],obj=[0.0])
                model.variables.add(names = ["start%d_%d" % (node.name,interval_node.interval[0])],
                                        types=[var_type])
                model.variables.add(names = ["end%d_%d" % (node.name,interval_node.interval[0])],
                                        types=[var_type])
        for node in self.nodes:
            for j in self.nodes:
                if node.name!=j.name and self.adj_matrix[node.name][j.name] > -0.5:
                    model.variables.add(names = ["y_%d_%d" % (node.name,j.name)],
                                        types=[var_type],obj=[self.adj_matrix[node.name][j.name]])
        if self.depot == -1:
            thevars = ["start%d_%d" % (node.name,interval_node.interval[0]) 
                            for node in self.nodes for interval_node in node.interval_nodes]
            thecoefs = [1]*len(thevars)
            model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
        else:
            depot=self.nodes[self.depot]
            thevars = ["start%d_%d" % (depot.name,interval_node.interval[0]) 
                            for interval_node in depot.interval_nodes]
            thecoefs = [1]*len(thevars)
            model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
        
        for node in self.nodes:
            thevars = (["start%d_%d" % (node.name,interval_node.interval[0]) 
                        for interval_node in node.interval_nodes]+
                        ["end%d_%d" % (node.name,interval_node.interval[0]) 
                        for interval_node in node.interval_nodes]
                        )
            thecoefs = ([1 for interval_node in node.interval_nodes]+
                        [-1 for interval_node in node.interval_nodes]
                        )
            model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
        
        for node in self.nodes:
            for interval_node in node.interval_nodes:
                thevars = (["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.interval[0],arc.head.interval[0])
                            for arc in interval_node.outgoing_arcs]+
                            ["x_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.interval[0],arc.head.interval[0])
                            for arc in interval_node.ingoing_arcs]+
                            ["start%d_%d" % (node.name,interval_node.interval[0])]+
                            ["end%d_%d" % (node.name,interval_node.interval[0])]
                            )
                thecoefs = [-1]*len(interval_node.outgoing_arcs)+[1]*len(interval_node.ingoing_arcs)+[1,-1]

                    
                model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                
            thevars = (["y_%d_%d" %(node.name,j.name)
                        for j in self.nodes if node.name!=j.name and self.adj_matrix[node.name][j.name] > -0.5]
                        )
            thecoefs = [1]*len(thevars)

            model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [1.0])
            for j in self.nodes:
                if node.name!=j.name and self.adj_matrix[node.name][j.name] > -0.5:
                    thevars = (["x_%d_%d_%d_%d" %(arc.tail.name,arc.head.name,arc.tail.interval[0],arc.head.interval[0])
                                for interval_node in node.interval_nodes
                                for arc in interval_node.outgoing_arcs if arc.head.name == j.name]
                                )
                    thecoefs = [1]*len(thevars)
                    thevars += ['y_%d_%d' % (node.name,j.name)]
                    thecoefs += [-1]
                    model.linear_constraints.add(lin_expr = [cplex.SparsePair(thevars,thecoefs)], senses = ["E"], rhs = [0.0])
                
    def solve_model(self):
        self.model.solve()
        
    
        
            
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
                if self.interval_nodes[idx+1].is_lowest_reachable(arc.tail.interval,arc.length,arc.tail.is_tw_ub()):
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
                if arc.head.is_lowest_reachable(self.interval_nodes[idx+1].interval,arc.length,self.interval_nodes[idx+1].is_tw_ub()):
                    new_arc = Arc(arc.head,self.interval_nodes[idx+1],arc.length)
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
                    
                
                    

class Arc():
    def __init__(self,head,tail,length):
        self.head = head
        self.head.ingoing_arcs.append(self)
        self.tail = tail
        self.tail.outgoing_arcs.append(self)
        self.length = length






vert_num = 11       
node_ind = range(0,vert_num)
TWs= [[0,9396],     
[0,865,],
[399,1299],
[550,1450],
[685,1585],
[819,1719],
[1208,2108],  
[1412,2312],  
[1701,2330],
[2413,3313],
[3798,4698]]
nodes = [[i,TWs[i][0],TWs[i][1]] for i in node_ind]
#adj_matrix = [[(1)*(i!=j) for j in node_ind] for i in node_ind]
adj_matrix=[[0,0,0,0,0,0,0,0,0,0,0],[71,0,85,85,85,96,85,85,97,85,77],
[50,65,0,77,77,62,77,77,62,77,74],
[64,74,88,0,88,80,88,88,82,88,87],
[44,54,68,68,0,60,68,68,62,68,67],
[51,75,51,51,51,0,51,51,80,51,65],
[53,64,78,78,78,68,0,78,69,78,76],
[51,67,69,69,69,75,69,0,75,69,63],
[43,61,71,71,71,43,71,71,0,71,68],
[53,64,78,78,78,68,78,78,69,0,80],
[42,66,52,52,52,70,52,52,70,52,0]]
TSP = Tsp(nodes,adj_matrix,0)
for sp in [100,200,300,400,500,600,700,800,900,1000,2000,3000,4000]:
    for node in TSP.nodes:
        node.split_node(sp)
TSP.create_model()
TSP.solve_model()

idx2name = { j : n for j, n in enumerate(TSP.model.variables.get_names()) }
solutionStringList = []
for i,val in enumerate(TSP.model.solution.get_values()):
    if val >0.5:
        if len(re.split("[_]",idx2name[i])) == 5:
            solutionStringList.append(idx2name[i])
        print(idx2name[i])
print solToPaths(solutionStringList)