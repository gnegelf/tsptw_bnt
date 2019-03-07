import cplex
import re
import numpy
import time
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

class Tsp():
    def __init__(self,nodes,adj_matrix,cost_matrix,depot,goal):
        self.update_duals=0
        self.depot = depot
        self.goal = goal
        self.check_interval_precedences = 1
        self.n=len(nodes)
        self.indices=range(self.n)
        self.nodes = [Tsp_node(self,node[0],node[1],node[2]) for node in nodes]
        self.arc_dict = {}
        for i in adj_matrix:
            for j in adj_matrix[i].keys():
                if not self.nodes[j].interval_nodes[0].is_reachable(self.nodes[i].interval_nodes[0],adj_matrix[i][j]):
                    adj_matrix[i].pop(j)
                else:
                    self.arc_dict[(i,j)]=[]
                    Arc(self.nodes[j].interval_nodes[0],
                        self.nodes[i].interval_nodes[0],
                        adj_matrix[i][j],
                        cost_matrix[i][j])
                
        self.adj_matrix = adj_matrix
        self.cost_matrix = cost_matrix
        self.adapt_model = 1
    def create_model(self,var_type='C'):
        #t0 = time.time()
        self.model = cplex.Cplex()
        self.const_num = 0
        self.const_name2idx={}
        self.idx2name = {}
        self.name2idx = {}
        self.x_names = []
        self.y_names = []
        self.z_names = []
        model = self.model
        model.set_results_stream(None)
        model.set_log_stream(None)
        model.parameters.advance.set(1)
        #TODO: Find out why warning gets triggered
        model.set_warning_stream(None)
        x_names = ["x_%d_%d" %(i,j) for i in self.indices for j in self.adj_matrix[i]]
        x_obj = [self.cost_matrix[i][j] for i in self.indices for j in self.adj_matrix[i]]
        y_names = ["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                arc.tail.id,arc.head.id)
                        for key,arc_list in self.arc_dict.iteritems() for arc in arc_list]
        z_names = ["z_%d_%d" % (node.name,interval_node.id) for node in self.nodes for interval_node in node.interval_nodes]

        self.add_variables(x_names,[var_type]*len(x_names),x_obj,[0.0]*len(x_names),[1.0]*len(x_names),'x')
        self.add_variables(y_names,[var_type]*len(x_names),[0.0]*len(x_names),[0.0]*len(y_names),[1.0]*len(y_names),'y')
        self.add_variables(z_names,[var_type]*len(z_names),[0.0]*len(z_names),[0.0]*len(z_names),[1.0]*len(z_names),'z')
        allvars = []
        allrhs = []
        allsenses = []
        all_names = []
        for node in self.nodes:
            thevars = ["z_%d_%d" %(node.name,interval_node.id) for interval_node in node.interval_nodes]
            thecoefs = [1]*len(thevars)
            allvars.append(cplex.SparsePair(thevars,thecoefs))
            allsenses.append("E")
            allrhs.append(1.0)
            all_names.append("visit_%d" % node.name)
        
        for node in self.nodes:
            for interval_node in node.interval_nodes:
                if node.name != self.goal:
                    thevars = ["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                   arc.tail.id,arc.head.id)
                                for arc in interval_node.outgoing_arcs]+["z_%d_%d" % (node.name,interval_node.id)]
                    
                    thecoefs = [1]*len(interval_node.outgoing_arcs)+[-1]
                    
                        
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("E")
                    allrhs.append(0.0)
                    all_names.append("goout_%d_%d" %(node.name,interval_node.id))
                if node.name != self.depot:
                    thevars = ["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                   arc.tail.id,arc.head.id)
                                for arc in interval_node.ingoing_arcs]+["z_%d_%d" % (node.name,interval_node.id)]
                    
                    thecoefs = [1]*len(interval_node.ingoing_arcs)+[-1]
    
                        
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("E")
                    allrhs.append(0.0)
                    all_names.append("goin_%d_%d" %(node.name,interval_node.id))
        
        for i,j in self.arc_dict:
            thevars = ["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                           arc.tail.id,arc.head.id)
                       for arc in self.arc_dict[i,j]]
            thecoefs = [1]*len(thevars)
            thevars += ["x_%d_%d" %(i,j)]
            thecoefs += [-1]
            allvars.append(cplex.SparsePair(thevars,thecoefs))
            allsenses.append("E")
            allrhs.append(0.0)
            all_names.append("arcuse_%d_%d" %(i,j))
        
        self.add_constraints(all_names,allvars,allsenses,allrhs)
        if var_type == 'C':
            self.model.set_problem_type(0)
            self.model.parameters.lpmethod.set(2)
        #self.model_creation_time += time.time()-t0
        #self.idx2name = { j : n for j, n in enumerate(model.variables.get_names()) }
        #self.name2idx = { n : j for j, n in enumerate(model.variables.get_names()) }
    def add_constraints(self,all_names,allvars,allsenses,allrhs):
        old_inds=self.const_num
        self.model.linear_constraints.add(names=all_names,lin_expr = allvars, 
                                                     senses = allsenses, rhs = allrhs)
        self.const_name2idx.update({name:old_inds+j for j,name in enumerate(all_names)})
        self.const_num += len(allvars)
    def add_variables(self,all_names,all_types,all_obj,all_lb,all_ub,var_type):
        old_inds=self.model.variables.get_num()
        self.model.variables.add(names = all_names,
                            types=all_types,obj=all_obj,
                            lb = all_lb,ub = all_ub)
        if var_type=='x':
            self.x_names += all_names
        if var_type=='y':
            self.y_names += all_names
        if var_type=='z':
            self.z_names += all_names
        self.name2idx.update({name:old_inds+j for j,name in enumerate(all_names)})
        self.idx2name.update({old_inds+j:name for j,name in enumerate(all_names)})
        self.var_num = len(self.name2idx)
    def change_variable_name(self,old_var_name,new_var_name):
        self.model.variables.set_names(old_var_name,new_var_name)
        self.idx2name[self.name2idx[old_var_name]] = new_var_name
        self.name2idx[new_var_name] = self.name2idx.pop(old_var_name)
        if new_var_name[0] == 'y':
            self.y_names.append(new_var_name)
            self.y_names.remove(old_var_name)
        else:
            print "Changing names of non y-variables is currently not supported :("
    def solve_model(self,branches=[]):
        self.model.set_problem_type(0)
        self.model.solve()
        primal_feasible = self.model.solution.is_primal_feasible()
        return primal_feasible
    def add_ste_cut(self,notS):
        in_arc_list = []
        out_arc_list = []
        S = []
        for i in range(0,len(self.nodes)):
            if i not in notS:
                S.append(i)
        sigma = self.sigmaOfS(S)
        pi = self.piOfS(S)
        #pi=[]
        #sigma=[]
        #print S
        #print pi
        #print sigma
        #old_inds=self.const_num
        
        for i in S:
            for j in notS:
                if (i not in pi and j not in pi) or 0 in S or self.n-1 in S :
                    if self.adj_matrix[i].has_key(j):
                        out_arc_list.append("x_%d_%d" % (i,j))
                if (i not in sigma and j not in sigma) or 0 in S or self.n-1 in S:
                    if self.adj_matrix[j].has_key(i):
                        in_arc_list.append("x_%d_%d" % (j,i))
        self.model.linear_constraints.add(lin_expr=[cplex.SparsePair(in_arc_list,[1.0]*len(in_arc_list))],senses=['G'],rhs=[1.0])
        self.model.linear_constraints.add(lin_expr=[cplex.SparsePair(out_arc_list,[1.0]*len(out_arc_list))],senses=['G'],rhs=[1.0])
        self.const_num += 2
        #self.const_name2idx.update({name:old_inds+j for j,name in enumerate(all_names)})
    def piOfS(self,S):
        pi=[]
        for i in self.indices:
            if i not in pi:
                for j in self.precedence_graph[i].keys():
                    if j in S:
                        pi.append(i)
                        break
        return pi
    def sigmaOfS(self,S):
        sigma=[]
        for j in self.indices:
            if j not in sigma:
                for i in S:
                    if j in self.precedence_graph[i]:
                        sigma.append(j)
                        break
        return sigma
            

#class for arcs between interval nodes adding itself to the respective in- and outgoing lists
#and also to the arc dictionary of the tsp class
class Arc():
    def __init__(self,head,tail,length,cost):
        self.head = head
        self.head.ingoing_arcs.append(self)
        self.tail = tail
        self.tail.outgoing_arcs.append(self)
        self.length = length
        self.cost = cost
        self.tail.tsp_node.tsp.arc_dict[tail.name,head.name].append(self)


#class for nodes in the original tsp-graph containing a subset of nodes of the time expanded graph
#also allows for changes in the time expanded graph
class Tsp_node():
    def __init__(self,tsp,name,tw_lb,tw_ub):
        self.tsp = tsp
        self.name = name
        self.tw_lb = tw_lb
        self.tw_ub = tw_ub
        self.interval_nodes = [Interval_node(name,[tw_lb,tw_ub],self,1)]
    def split_node(self,split_point,dual_value_locations=[]):
        idx = self.find_index(split_point)
        old_ub = self.interval_nodes[idx].interval[1] 
        if idx == -1:
            print split_point
            print self.name
            print ("Error: split point %f already used in node %d's intervals " % (split_point,self.name) )
            return 0
        self.interval_nodes.insert(idx+1,Interval_node(self.name,[split_point,old_ub],self,self.interval_nodes[idx].is_ub))
        self.interval_nodes[idx].is_ub=0
        self.interval_nodes[idx].interval[1] = split_point
        
        #print self.interval_nodes[idx].id
        #print split_point
        #add variable for new node
        if self.tsp.adapt_model:
            self.tsp.add_variables(["z_%d_%d" % (self.name,self.interval_nodes[idx+1].id)],['C'],[0.0],[0.0],[1.0],'z')
            self.tsp.model.linear_constraints.set_coefficients([("visit_%d"% self.name,
                                                                 "z_%d_%d" %(self.name,self.interval_nodes[idx+1].id)
                                                                 ,1.0) ])
            names = ["goout_%d_%d" %(self.name,self.interval_nodes[idx+1].id),
                     "goin_%d_%d" %(self.name,self.interval_nodes[idx+1].id)]
            lin_expr=[cplex.SparsePair(["z_%d_%d" % (self.name,self.interval_nodes[idx+1].id)],
                                        [-1.0]),
                      cplex.SparsePair(["z_%d_%d" % (self.name,self.interval_nodes[idx+1].id)],
                                        [-1.0])]
            self.tsp.add_constraints(names,lin_expr,["E","E"],[0.0,0.0])
            if self.tsp.update_duals:
                dual_value_locations += [self.tsp.const_name2idx["goout_%d_%d"%(self.name,self.interval_nodes[idx].id)],
                                         self.tsp.const_name2idx["goin_%d_%d"%(self.name,self.interval_nodes[idx].id)]]

        pop_indices = []
        arc_index = 0
        
        for arc in self.interval_nodes[idx].ingoing_arcs:
            if not arc.head.is_lowest_reachable(arc.tail,arc.length):
                if self.tsp.adapt_model:
                    old_var_name = "y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.id,arc.head.id)
                pop_indices.insert(0,arc_index)
                arc.head = self.interval_nodes[idx+1]
                self.interval_nodes[idx+1].ingoing_arcs.append(arc)
                if self.tsp.adapt_model:
                    const_name = "goin_%d_%d" % (self.name,self.interval_nodes[idx].id)
                    self.tsp.model.linear_constraints.set_coefficients([(const_name,old_var_name,0.0)]) 
                    new_var_name = "y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.id,
                                                      self.interval_nodes[idx+1].id )
                    const_name = "goin_%d_%d" % (self.name,self.interval_nodes[idx+1].id)
                    
                    self.tsp.model.linear_constraints.set_coefficients([(const_name,old_var_name,1.0)]) 
                    self.tsp.change_variable_name(old_var_name,new_var_name)

            else:
                if self.interval_nodes[idx+1].is_lowest_reachable(arc.tail,arc.length):
                    print "A bug occurred this event should be impossible :("
            arc_index += 1
        for i in pop_indices:
            self.interval_nodes[idx].ingoing_arcs.pop(i)
        
        pop_indices = []
        arc_index = 0
        for arc in self.interval_nodes[idx].outgoing_arcs:
            if not arc.head.is_lowest_reachable(arc.tail,arc.length):
                print "A bug occurred this event should not happen:("
                #blub-8
            else:
                if arc.head.is_lowest_reachable(self.interval_nodes[idx+1],
                                                arc.length):
                    
                    all_nodes_reachable=1
                    #print (arc.tail.name,arc.head.name)
                    for k in range(1,self.tsp.n-1):
                        #print k
                        if k != arc.tail.name and k!=arc.head.name:
                            if ((arc.head.name == self.tsp.n-1 or self.interval_nodes[idx+1].interval[0]+arc.length+
                                 self.tsp.old_adj_matrix[arc.head.name][k]>TWs[k][1])
                                and (arc.tail.name == 0 or TWs[k][0]+self.tsp.old_adj_matrix[k][arc.tail.name]
                                +arc.length>TWs[arc.head.name][1])):
                                all_nodes_reachable = 0
                                #time.sleep(1)
                                #print("")
                                break
                    if (not self.tsp.check_interval_precedences or all_nodes_reachable) or 1:
                        new_arc = Arc(arc.head,self.interval_nodes[idx+1],arc.length,arc.cost)
                    if self.tsp.adapt_model and (not self.tsp.check_interval_precedences or all_nodes_reachable or 1):
                        new_var_name = "y_%d_%d_%d_%d" % (new_arc.tail.name,new_arc.head.name,
                                                          new_arc.tail.id,new_arc.head.id)
                        if (not self.tsp.check_interval_precedences or all_nodes_reachable):
                            self.tsp.add_variables([new_var_name],['C'],[0.0],[0.0],[1.0],'y')
                        else:
                            self.tsp.add_variables([new_var_name],['C'],[0.0],[0.0],[0.0],'y')
                        const_name = "goin_%d_%d" % (new_arc.head.name,new_arc.head.id)
                        self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)])
                        const_name = "goout_%d_%d" % (new_arc.tail.name,new_arc.tail.interval[0])
                        self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)]) 
                        const_name = "arcuse_%d_%d" % (new_arc.tail.name,new_arc.head.name)
                        self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)]) 
                else:
                    is_reachable,new_head = self.tsp.nodes[arc.head.name].find_lowest_reachable(self.interval_nodes[idx+1],
                                                arc.length)
                    
                    if is_reachable:
                        all_nodes_reachable=1
                        for k in range(1,self.tsp.n-1):
                            if k != arc.tail.name and k!=arc.head.name:
                                if ((arc.head.name== self.tsp.n-1 or self.interval_nodes[idx+1].interval[0]+arc.length+
                                     self.tsp.old_adj_matrix[arc.head.name][k]>TWs[k][1])
                                    and (arc.tail.name==0 or TWs[k][0]+self.tsp.old_adj_matrix[k][arc.tail.name]+
                                         arc.length>TWs[arc.head.name][1])):
                                    all_nodes_reachable = 0
                                    #time.sleep(10)
                                    #print("Checking precedences does something, no arc added")
                                    break
                        if (not self.tsp.check_interval_precedences or all_nodes_reachable) or 1:
                            new_arc = Arc(new_head,self.interval_nodes[idx+1],arc.length,arc.cost)
                        if self.tsp.adapt_model and (not self.tsp.check_interval_precedences or all_nodes_reachable or 1):
                            new_var_name = "y_%d_%d_%d_%d" % (new_arc.tail.name,new_arc.head.name,
                                                              new_arc.tail.id,new_arc.head.id)
                            if (not self.tsp.check_interval_precedences or all_nodes_reachable):
                                self.tsp.add_variables([new_var_name],['C'],[0.0],[0.0],[1.0],'y')
                            else:
                                self.tsp.add_variables([new_var_name],['C'],[0.0],[0.0],[0.0],'y')
                            const_name = "goin_%d_%d" % (new_arc.head.name,new_arc.head.interval[0])
                            self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)])  
                            const_name = "goout_%d_%d" % (new_arc.tail.name,new_arc.tail.interval[0])
                            self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)])
                            const_name = "arcuse_%d_%d" % (new_arc.tail.name,new_arc.head.name)
                            self.tsp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)]) 
            arc_index += 1
        for i in pop_indices:
            self.interval_nodes[idx].outgoing_arcs.pop(i)
    def find_index(self,split_point,tol=0.000001):
        if split_point < self.tw_lb-tol:
            print ("Error: split point outside of time window (less)")
            return 0
        else:
            if split_point > self.tw_ub+tol:
                print ("Error: split point outside of time window (larger)")
            for i,i_node in enumerate(self.interval_nodes):
                if i_node.interval[0]+tol < split_point and i_node.interval[1]-tol+2*tol*(i_node.is_tw_ub())> split_point:
                    return i
            return -1
    def find_lowest_reachable(self,node,step):
        for interval_node in self.interval_nodes:
            if interval_node.is_lowest_reachable(node,step):
                return 1,interval_node
        return 0,-1   

#gives intersection of two interval (a,b) and (c,d)
def has_intersection(interval_1,interval_2):
    return ((interval_1[0]-interval_2[0]>0 and interval_1[0]-interval_2[1]<0) 
            or (interval_2[0]-interval_1[0]>0 and interval_2[0]-interval_1[1]<0)
            or (interval_1[1]-interval_2[0]>0 and interval_1[1]-interval_2[1]<0)
            or (interval_2[1]-interval_1[0]>0 and interval_2[1]-interval_1[1]<0)
            )

#a node in the time expanded graph with an interval assigned to it that may be changed
class Interval_node():
    def __init__(self,name,interval,tsp_node,is_ub,tol=0.00001):
        self.name = name
        self.id = interval[0]
        self.interval = interval
        self.tsp_node = tsp_node
        self.ingoing_arcs = []
        self.outgoing_arcs = []
        self.is_ub = is_ub
    def is_reachable(self,node,shift,tol=0.00001):
        interv1=(node.interval[0]+shift,node.interval[1]+shift-2*tol+4*tol*node.is_tw_ub())
        
        interv2=((not self.is_tw_lb())*self.interval[0]-tol,self.interval[1]-2*tol+4*tol*self.is_tw_ub())
        return has_intersection(interv1,interv2)   
    def is_tw_lb(self,tol=0.00001):
        return abs(self.interval[0] - self.tsp_node.tw_lb)< tol
    def is_tw_ub(self,tol=0.00001):
        return self.is_ub
    def is_lowest_reachable(self,node,step,tol=0.00001):
        interv1=(node.interval[0]+step,node.interval[1]-2*tol+4*tol*node.is_tw_ub()+step)
        return self.is_reachable(node,step) and (self.is_tw_lb() or not has_intersection((0,self.interval[0]-tol),interv1))

class Tree_node():
    def __init__(self,tree,branches,dual_values_model=0,dual_values_branches=0,primal_values = 0,slacks = 0,red_costs = 0):
        self.tree = tree
        self.descendents = {}
        self.branches = branches
        self.branch_names = [branch.name for branch in self.branches]
        self.branch_lin_exprs = [branch.lin_expr for branch in self.branches]
        self.branch_senses = [branch.sense for branch in self.branches]
        self.branch_rhs = [branch.rhs for branch in self.branches]
        self.dual_values_model = dual_values_model
        self.dual_values_branches = dual_values_branches
        self.primal_values = primal_values
        self.slacks = slacks
        self.red_costs = red_costs
        self.lower_bound = 0
        self.fractionals = {}
        self.y_fractionals = {}
        if self.solve_lp_relaxation() and self.lower_bound < self.tree.ub:
            self.feasible = 1
        else:
            self.feasible = 0
    def solve_lp_relaxation(self,dual_value_locations=[],warm_start=1,tol=0.000001):
        
        tsp = self.tree.tsp
        if tsp.update_duals:
            for ind in dual_value_locations:
                self.dual_values_model.append(self.dual_values_model[ind])
        model = tsp.model
        model.linear_constraints.add(names = self.branch_names,lin_expr = self.branch_lin_exprs,
                                         senses = self.branch_senses,rhs = self.branch_rhs)
        if self.tree.tsp.update_duals and self.dual_values_model != 0:
            t_start0 = time.time()
            model.parameters.advance.set(1)
            model.start.set_start(col_status=[],
                     row_status=[],
                     #col_primal=self.primal_values,

                     row_dual=self.dual_values_model+self.dual_values_branches,
                     col_primal=[],
                     row_primal=[],
                     col_dual=[],
                     #row_dual=[]
                     )
            self.tree.add_start_time += time.time() - t_start0
        model.parameters.preprocessing.presolve.set(0)
        t0 = time.time()
        #print "start"
        feas = tsp.solve_model()
        self.tree.lp_times.append((time.time()-t0))
        self.tree.simp_iteras.append(model.solution.progress.get_num_iterations())
        #time.sleep(10)
        model.parameters.advance.set(0)
        #print "no start"
        
        #t0=time.time()
        #feas = tsp.solve_model()
        #self.tree.lp_no_dual_times.append((time.time()-t0))
        #self.tree.simp_nd_iteras.append(model.solution.progress.get_num_iterations())
        #time.sleep(10)
        if not feas:
            
            self.lower_bound = 1000000
            model.linear_constraints.delete(xrange(model.linear_constraints.get_num()-len(self.branches),model.linear_constraints.get_num()))
            return 0
        else:
            t_sol_eval0=time.time()
            solution = model.solution
            dual_values = solution.get_dual_values()
            self.dual_values_model = dual_values[:self.tree.tsp.const_num]
            self.dual_values_branches = dual_values[self.tree.tsp.const_num:]
            self.primal_values = solution.get_values()
            self.lower_bound = solution.get_objective_value()
            self.primal_x_values = {name:self.primal_values[self.tree.tsp.name2idx[name]] for name in self.tree.tsp.x_names 
                                    if self.primal_values[self.tree.tsp.name2idx[name]]>tol}
            self.primal_y_values = {name:self.primal_values[self.tree.tsp.name2idx[name]] for name in self.tree.tsp.y_names
                                    if self.primal_values[self.tree.tsp.name2idx[name]]>tol}
            self.primal_z_values = {name:self.primal_values[self.tree.tsp.name2idx[name]] for name in self.tree.tsp.z_names
                                    if self.primal_values[self.tree.tsp.name2idx[name]]>tol}
            for key,val in self.primal_x_values.iteritems():
                if abs(0.5-val)<0.5-tol:
                    self.fractionals[key] = val
            for key,val in self.primal_y_values.iteritems():
                if abs(0.5-val)<0.5-tol:
                    self.y_fractionals[key] = val
            self.tree.sol_eval_time += time.time() - t_sol_eval0
        model.linear_constraints.delete(xrange(model.linear_constraints.get_num()-len(self.branches),model.linear_constraints.get_num()))
        #self.tree.total_relaxation_time += time.time() - t0
        return 1
    def is_x_integer(self):
        if len(self.fractionals) == 0:
            return 1
        else:
            return 0
    def choose_branch_var(self):
        #return self.branch_var, self.branch_val
        #t0 = time.time()
        max_score = -100
        chosen_var = -1
        chosen_val = -1
        scores = []
        for var_name,var_val in self.fractionals.iteritems():
            #return var_name,var_val
            score,has_history = self.calc_score(var_name)
            if has_history:
                scores.append(score)
            if score > max_score:
                max_score = score
                chosen_var = var_name
                chosen_val = var_val
        if len (scores)>0:
            self.tree.psi_avg = sum(scores)/len(scores)
        #self.tree.branch_variable_selection_time += time.time()-t0
        return chosen_var, chosen_val
    def calc_score(self,var_name):
        branch_history = self.tree.branch_history
        if len(branch_history[var_name])>0:
            psi_0 = 0.0
            psi_1 = 0.0
            for tup in branch_history[var_name]:
                psi_0 += tup[0]
                psi_1 += tup[1]
            psi_0 = psi_0 /len(branch_history[var_name])
            psi_1 = psi_1 /len(branch_history[var_name])
            return (5.0/6)*min(psi_0,psi_1)+1.0/6*max(psi_0,psi_1),1
        else:
            return self.tree.psi_avg,0


class cutFinder():
    def __init__(self,tsp):
        self.tsp = tsp
        self.createFlowModel(tsp.adj_matrix)
    def findCut(self,s,t,capacities,tol=0.0001):
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
        indices = range(0,self.tsp.n-1)
        indices.pop(s)
        if obj < 1-tol:
            #print dual_values
            print obj
            #for p,name in enumerate(self.model.variables.get_names()):
            #    if self.model.solution.get_values(name)> 0.01:
            #        print name +": " + str(self.model.solution.get_values(name))
            print "goal cannot be reached from %d at full capacity" % s
            for index in indices:
                if dual_values[self.dual_name2idx["flow_%d"%index]]> tol:
                    continue
                else:
                    cut_S.append(index)
        else:
            cut_S=-1
        self.model.linear_constraints.add(names=["flow_%d" % s],lin_expr=[s_coefs],rhs=[0],senses=["E"])
        self.model.linear_constraints.add(names=["flow_%d" % t],lin_expr=[t_coefs],rhs=[0],senses=["E"])
        self.dual_name2idx = { n : j for j, n in enumerate(self.model.linear_constraints.get_names()) }
        return cut_S
    def createFlowModel (self,adj_matrix):
        model = cplex.Cplex()
        self.model=model
        model.set_results_stream(None)
        model.set_log_stream(None)
        model.set_warning_stream(None)
        model.objective.set_sense(model.objective.sense.maximize)
        for i in adj_matrix:
            for j in adj_matrix[i]:
                if j == self.tsp.n-1:
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
        #return model

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

def find_times(tree_node,tol=0.0001,add_all=1):
    adj_matrix = tree_node.tree.tsp.adj_matrix
    n = tree_node.tree.tsp.n
    A = numpy.zeros((n,n))
    b = numpy.zeros(n)
    x_frac = tree_node.primal_x_values
    y_frac = tree_node.primal_y_values
    for j in range(n):
        A[j,j] = 1
    for key,val in x_frac.iteritems():
        i,j = (int(re.split("[_]",key)[1]),int(re.split("[_]",key)[2]))
        A[j,i] -= val
    if add_all:
        split_points=[]
    else:
        split_points=[]
        split_points_temp={i:(0,-1) for i in adj_matrix}
    for key,val in y_frac.iteritems():
        if val>tol:
            #Todo: adjust code so that this is unneccessary
            i,j,lb_1,lb_2=(int(re.split("[_]",key)[1]),int(re.split("[_]",key)[2]),int(re.split("[_]",key)[3]),int(re.split("[_]",key)[4]))
            if lb_2-lb_1>adj_matrix[i][j]-0.0001:
                b[j] += (lb_2-lb_1)*val
            else:
                b[j] += adj_matrix[i][j]*val
                arrTime = lb_1+adj_matrix[i][j]
                if add_all:
                    if j in range(1,len(adj_matrix)-1):
                        split_points.append((j,arrTime))
                else:
                    score=(lb_1+adj_matrix[i][j]-lb_2)*val
                    if score > split_points_temp[j][1]:
                        split_points_temp[j] = (arrTime,score,i)
    if not add_all:
        for j in split_points_temp:
            if j in range(1,len(adj_matrix)-1):
                if split_points_temp[j][1] >0:
                    split_points.append((j,split_points_temp[j][0]))
    times = numpy.linalg.solve(A,b)
    times = times.tolist()
    times.pop(-1)
    return times,split_points

class Tree():
    def __init__(self,tsp,start_control):
        self.print_interval = 25
        self.add_all_split_points=1
        #self.print_switch = 0
        self.psi_avg = 0
        self.lp_times=[]
        self.lp_no_dual_times=[]
        self.simp_iteras=[]
        self.simp_nd_iteras=[]
        self.start_control = start_control
        self.branch_variable_selection_time = 0.0
        self.node_selection_time = 0.0
        self.total_relaxation_time = 0.0
        self.find_ste_time = 0.0
        self.add_cut_time = 0.0
        self.split_time = 0.0
        self.find_split_time = 0.0
        self.sol_eval_time = 0.0
        self.add_start_time = 0.0
        self.closed_nodes = []
        self.tsp = tsp
        self.ub = 94020
        self.lb = 0.0
        self.root = Tree_node(self,[])
        self.open_nodes = [self.root]
        self.branch_history = {key:[] for key in tsp.x_names}
        self.time_limit = 100000
        self.cutFinder = cutFinder(self.tsp)
    def conditional_print(self,string):
        if self.count % self.print_interval == 0:
            print string
    def choose_node(self,selection=1):
        t0=time.time()
        if selection == 1:
            minInd = 0
            minVal= 100000000
            for i,node in enumerate(self.open_nodes):
                #if (len(node.fractionals)==0):
                #    return self.open_nodes.pop(i)
                if node.lower_bound < minVal:
                    minInd = i
                    minVal = node.lower_bound
            self.lb=minVal
        if selection == 3:
            minInd = 0
            minVal= 100000000
            minLb = 100000000
            for i,node in enumerate(self.open_nodes):
                fac=0.5
                
                if len(node.fractionals)+fac*node.lower_bound < minVal:
                    minInd = i
                    minVal = len(node.fractionals)+fac*node.lower_bound
                if node.lower_bound < minLb:
                    minLb = node.lower_bound
            self.lb = minLb
        self.node_selection_time += time.time() - t0
        return self.open_nodes.pop(minInd)
    def branch_and_split(self):
        self.count=1
        t0=time.time()
        splitNodesForNonInteger = 1
        addCutForNonInteger = 1
        oldLb=0
        while len(self.open_nodes)>0 and time.time()-t0< self.time_limit:
            self.count += 1
            cutAdded = 0
            splitNodes = 0
            node = self.choose_node()
            if self.count>50:
                addCutForNonInteger = 0
            #if abs(oldLb-node.lower_bound)<0.1:
            #    splitNodesForNonInteger = 0
            #if abs(oldLb-node.lower_bound)>5:
            #    splitNodesForNonInteger = 0
            oldLb=node.lower_bound
            self.conditional_print("Current lower bound: %f" % self.lb)
            self.conditional_print("Current best upper Bound: %f" % self.ub)
            self.conditional_print("Number of open nodes: %d" % len(self.open_nodes))
            t_ste_0 = time.time()
            S = solToComponents(node.primal_x_values)
            self.find_ste_time +=time.time()-t_ste_0
            #Segment to add STE Cuts, if a cut is added loop is continued
            if len(S) != self.tsp.n:
                print "Adding STE cut for set: " + str(S)
                cutAdded = 1
                self.tsp.add_ste_cut(S)
            else:
                if len(node.fractionals)>0 and addCutForNonInteger:
                    t_ste_0 = time.time()
                    for i in range(1,len(self.tsp.adj_matrix)-1):
                        S = self.cutFinder.findCut(i,len(self.tsp.nodes)-1,node.primal_x_values)
                        if S!=-1:
                            print "Adding STE cut for set: " + str(S)
                            cutAdded = 1
                            self.tsp.add_ste_cut(S)
                            self.find_ste_time +=time.time()-t_ste_0
                            break
                    self.find_ste_time +=time.time()-t_ste_0
            if cutAdded:
                t_add_cut0 = time.time()
                pop_indices=[]
                self.open_nodes.append(node)
                for i,node2 in enumerate(self.open_nodes):
                    if self.tsp.update_duals:
                        node2.dual_values_model += [0.0,0.0]
                    node2.solve_lp_relaxation()
                    if not node2.feasible or node2.lower_bound >= self.ub:
                        pop_indices.append(i)
                while (len(pop_indices)>0):
                    self.open_nodes.pop(pop_indices.pop(-1))
                self.add_cut_time += time.time() - t_add_cut0 
                continue
            
            #Segment to split nodes, if nodes are split loop is continued
            
            if len(node.fractionals)==0 or splitNodesForNonInteger:
                t_find_split0 = time.time()
                times,split_points= find_times(node,add_all=self.add_all_split_points)
                for i,t in enumerate(times):
                    if t> self.tsp.nodes[i].tw_ub+0.0001:
                        splitNodes = 1
                        self.find_split_time += time.time() - t_find_split0
                        break
                self.find_split_time += time.time() - t_find_split0
            if splitNodes:
                t_split0 = time.time()
                print "Splitting nodes"
                #print split_points
                
                dual_value_locations=[]
                if self.tsp.update_duals:
                    for i,t in split_points:
                        self.tsp.nodes[i].split_node(t,dual_value_locations)
                else:
                    for i,t in split_points:
                        self.tsp.nodes[i].split_node(t)
                if len(split_points)==0:
                    splitNodesForNonInteger = 0
                pop_indices=[]
                oldLb=node.lower_bound
                node.solve_lp_relaxation(dual_value_locations)
                if abs(oldLb-node.lower_bound)<0.001:
                    splitNodesForNonInteger = 0
                self.open_nodes.append(node)
                for i,node2 in enumerate(self.open_nodes):
                    node2.solve_lp_relaxation(dual_value_locations)
                    if not node2.feasible or node2.lower_bound >= self.ub:
                        pop_indices.append(i)
                while (len(pop_indices)>0):
                    self.open_nodes.pop(pop_indices.pop(-1))
                #time.sleep(20)
                self.split_time += time.time()-t_split0
                continue
            
            #segment if no cut was added and nodes were not split
            if len(node.fractionals) == 0:
                print "Integer feasible solution found, objective: %f" %node.lower_bound
                self.ub = node.lower_bound
                self.solution_node=node
                pop_indices=[]
                #pruning of tree
                for i,node2 in enumerate(self.open_nodes):
                    if not node2.feasible or node2.lower_bound >= self.ub:
                        pop_indices.append(i)
                while (len(pop_indices)>0):
                    self.open_nodes.pop(pop_indices.pop(-1))
            else:
                #branching step
                branch_var,branch_val = node.choose_branch_var()
                #print "branching"
                f_1 = 1.0-branch_val
                f_0 = branch_val
                if self.tsp.update_duals:
                    new_node_list = [Tree_node(node.tree,node.branches+[Branch(branch_var,'L',0.0)],
                                                                        node.dual_values_model,
                                                                        node.dual_values_branches+[0.0]),
                                     Tree_node(node.tree,node.branches+[Branch(branch_var,'G',1.0)],
                                                                        node.dual_values_model,
                                                                        node.dual_values_branches+[0.0])]
                else:
                    new_node_list = [Tree_node(node.tree,node.branches+[Branch(branch_var,'L',0.0)]),
                                     Tree_node(node.tree,node.branches+[Branch(branch_var,'G',1.0)])]  
                if new_node_list[0].feasible:
                    c_0 = (new_node_list[0].lower_bound-node.lower_bound)/f_0
                else:
                    c_0 = self.psi_avg/f_0
                if new_node_list[1].feasible:
                    c_1 = (new_node_list[1].lower_bound-node.lower_bound)/f_1
                else:
                    c_1 = self.psi_avg/f_1
                self.branch_history[branch_var].append((c_0,c_1))
                    
                #time.sleep(10)
                for new_node1 in new_node_list:
                    if new_node1.feasible:
                        if new_node1.lower_bound < self.ub-0.0001:
                            self.open_nodes.append(new_node1)
    def dynamic_discovery(self,split_limit=10000):
        t0=time.time()
        self.count=0
        timefeasible=0
        splits=0
        addCutForNonInteger = 1
        while not timefeasible and splits<split_limit:
            addCutForNonInteger = 1
            while len(self.open_nodes)>0 and time.time()-t0< self.time_limit:
                self.count+=1
                cutAdded = 0

                node = self.choose_node()
                if len(self.open_nodes)>4:
                    addCutForNonInteger = 0
                self.conditional_print("Current lower bound: %f" % self.lb)
                self.conditional_print("Current best upper Bound: %f" % self.ub)
                self.conditional_print("Number of open nodes: %d" % len(self.open_nodes))
                t_find_cut0 = time.time()
                S = solToComponents(node.primal_x_values)
                self.find_ste_time += time.time() - t_find_cut0
                if len(S) != self.tsp.n:
                    print "Adding STE cut for set: " + str(S)
                    cutAdded = 1
                    self.tsp.add_ste_cut(S)
                else:
                    if len(node.fractionals)>0 and addCutForNonInteger:
                        t_find_cut0 = time.time()
                        for i in range(1,len(self.tsp.adj_matrix)-1):
                            S = self.cutFinder.findCut(i,len(self.tsp.nodes)-1,node.primal_x_values)
                            if S!=-1:
                                print "Adding STE cut for set: " + str(S)
                                cutAdded = 1
                                self.tsp.add_ste_cut(S)
                                self.find_ste_time += time.time() - t_find_cut0
                                break
                        self.find_ste_time += time.time() - t_find_cut0
                if cutAdded:
                    t_cut_0 = time.time()
                    pop_indices=[]
                    self.open_nodes.append(node)
                    for i,node2 in enumerate(self.open_nodes):
                        if self.tsp.update_duals:
                            node2.dual_values_model += [0.0,0.0]
                        node2.solve_lp_relaxation()
                        if not node2.feasible or node2.lower_bound >= self.ub:
                            pop_indices.append(i)
                    while (len(pop_indices)>0):
                        self.open_nodes.pop(pop_indices.pop(-1))
                    self.add_cut_time += time.time()-t_cut_0
                    continue
                
                
                
                #segment if no cut was added
                if len(node.fractionals) == 0:
                    print "Integer feasible tour without cycle found, objective: %f" %node.lower_bound
                    self.ub = node.lower_bound
                    self.solution_node=node
                    pop_indices=[]
                    #pruning of tree
                    for i,node2 in enumerate(self.open_nodes):
                        if not node2.feasible or node2.lower_bound >= self.ub:
                            pop_indices.append(i)
                    while (len(pop_indices)>0):
                        self.open_nodes.pop(pop_indices.pop(-1))
                else:
                    #branching step
                    branch_var,branch_val = node.choose_branch_var()
                    #print "branching"
                    f_1 = 1.0-branch_val
                    f_0 = branch_val
                    if self.tsp.update_duals:
                        new_node_list = [Tree_node(node.tree,node.branches+[Branch(branch_var,'L',0.0)],
                                                                            node.dual_values_model,
                                                                            node.dual_values_branches+[0.0]),
                                         Tree_node(node.tree,node.branches+[Branch(branch_var,'G',1.0)],
                                                                            node.dual_values_model,
                                                                            node.dual_values_branches+[0.0])]
                    else:
                        new_node_list = [Tree_node(node.tree,node.branches+[Branch(branch_var,'L',0.0)]),
                                         Tree_node(node.tree,node.branches+[Branch(branch_var,'G',1.0)])]
                    if new_node_list[0].feasible:
                        c_0 = (new_node_list[0].lower_bound-node.lower_bound)/f_0
                    else:
                        c_0 = self.psi_avg/f_0
                    if new_node_list[1].feasible:
                        c_1 = (new_node_list[1].lower_bound-node.lower_bound)/f_1
                    else:
                        c_1 = self.psi_avg/f_1
                    self.branch_history[branch_var].append((c_0,c_1))
                    #time.sleep(10)
                    for new_node1 in new_node_list:
                        if new_node1.feasible:
                            if new_node1.lower_bound < self.ub-0.0001:
                                self.open_nodes.append(new_node1)
            #time.sleep(1)
            timefeasible = 1
            t_split0 = time.time()
            t_find_split0 = time.time()
            times,split_points= find_times(self.solution_node,add_all=self.add_all_split_points)
            for i,t in enumerate(times):
                if t> self.tsp.nodes[i].tw_ub+0.0001:
                    self.ub=100000
                    timefeasible = 0
                    self.find_split_time += time.time() - t_find_split0
                    break
            self.find_split_time += time.time() - t_find_split0
            if not timefeasible:
                #t_split0 = time.time()
                splits+=1
                print "Splitting nodes"
                #print split_points
                for i,t in split_points:
                        self.tsp.nodes[i].split_node(t)
                self.root = Tree_node(self,[])
                self.open_nodes = [self.root]
            self.split_time += time.time()-t_split0
                
            
def pathPossible(P,TWs,old_adj_matrix):
    #print P
    sp=TWs[P[0]][0]
    for i in range(len(P)-1):
        ep=TWs[P[i+1]][1]
        if sp+old_adj_matrix[P[i]][P[i+1]]>ep:
            return 0
        else:
            sp=max(TWs[P[i+1]][0],sp+old_adj_matrix[P[i]][P[i+1]])
    return 1
        
  
def process_adj_matrix(adj_matrix,adj_matrix_tr,TWs,old_adj_matrix,deep_check=0):
    changed=1
    deep_check_status=0
    while changed:
        changed=0
        for i in  range(vert_num):
            for j in range(vert_num):
                if adj_matrix[i].has_key(j):
                    if TWs[i][0]+adj_matrix[i][j]>TWs[j][1]+0.0001:
                        changed=1
                        adj_matrix[i].pop(j)
        for i in adj_matrix:
            if len(adj_matrix[i])==1:
                k=adj_matrix[i].keys()[0]
                for j in adj_matrix:
                    if j!=i:
                        if adj_matrix[j].has_key(k):
                            adj_matrix[j].pop(k)
                            changed=1
        for i in adj_matrix:
            if len(adj_matrix_tr[i])==1:
                k=adj_matrix_tr[i].keys()[0]
                for j in adj_matrix[k].keys():
                    if i!=j:
                        adj_matrix[k].pop(j)
                        changed=1
        
        if (not changed) and deep_check:
            print "Doing deep check"
            arcsToBeDeleted=[]
            for k in adj_matrix:
                #if k==0:
                #    continue
                for l in adj_matrix[k]:
                    deletekl=0
                    for i in adj_matrix:
                        if not i in [k,l]:
                            logical2=0
                            if old_adj_matrix[i].has_key(k):
                                logical2=logical2 or pathPossible([i,k,l],TWs,old_adj_matrix)
                            if old_adj_matrix[l].has_key(i):
                                logical2=logical2 or pathPossible([k,l,i],TWs,old_adj_matrix)
                            if not logical2:
                                deletekl=1
                                arcsToBeDeleted.append((k,l))
                                #print "Popping arc:(%d,%d) because of %d" %(k,l,i)  
                                break
                        for j in adj_matrix:
                            if (i not in [j,k,l]) and (j not in [i,k,l]):
                                logical1=0
                                if old_adj_matrix[i].has_key(j) and old_adj_matrix[j].has_key(k):
                                    logical1=logical1 or pathPossible([i,j,k,l],TWs,old_adj_matrix)
                                if old_adj_matrix[j].has_key(i) and old_adj_matrix[i].has_key(k):
                                    logical1=logical1 or pathPossible([j,i,k,l],TWs,old_adj_matrix)
                                if  old_adj_matrix[l].has_key(i) and old_adj_matrix[i].has_key(j) :
                                    logical1=logical1 or pathPossible([k,l,i,j],TWs,old_adj_matrix)
                                if  old_adj_matrix[l].has_key(j) and old_adj_matrix[j].has_key(i) :
                                    logical1=logical1 or pathPossible([k,l,j,i],TWs,old_adj_matrix)
                                if  old_adj_matrix[i].has_key(k) and old_adj_matrix[l].has_key(j) :
                                    logical1=logical1 or pathPossible([i,k,l,j],TWs,old_adj_matrix)
                                if  old_adj_matrix[j].has_key(k) and old_adj_matrix[l].has_key(i) :
                                    logical1=logical1 or pathPossible([j,k,l,i],TWs,old_adj_matrix)  
                                if not logical1:
                                    deletekl=1
                                    arcsToBeDeleted.append((k,l))
                                    #print "Popping arc:(%d,%d) because of (%d,%d)" %(k,l,i,j)  
                                    break
                        if deletekl:
                            break
            for (k,l) in arcsToBeDeleted:
                changed=1
                adj_matrix[k].pop(l)
                deep_check_status=1
        if changed:
            adj_matrix_tr.update({i:{} for i in adj_matrix})
            for i in adj_matrix:
                for j in adj_matrix[i]:
                    adj_matrix_tr[j][i]=adj_matrix[i][j]
    return deep_check_status

def build_precedence_graph(adj_matrix,adj_matrix_tr,TWs,old_adj_matrix):
    precedence_graph={i:{} for i in adj_matrix}
    precedence_graph[0]={j:-1 for j in old_adj_matrix[0]}
    for i in adj_matrix:
        for j in adj_matrix:
            if old_adj_matrix[j].has_key(i):
                if TWs[j][0]+old_adj_matrix[j][i]>0.0001+TWs[i][1]:
                    precedence_graph[i][j]=-1
    for i in adj_matrix:
        if i!=len(adj_matrix)-1:
            if not precedence_graph[i].has_key(len(adj_matrix)-1):
                precedence_graph[i][len(adj_matrix)-1]=-1
    for i in precedence_graph:
        for j in precedence_graph[i]:
            for k in precedence_graph[j]:
                if adj_matrix[i].has_key(k):
                    adj_matrix[i].pop(k)
    adj_matrix_tr.update({i:{} for i in adj_matrix})
    for i in adj_matrix:
        for j in adj_matrix[i]:
            adj_matrix_tr[j][i]=adj_matrix[i][j]
    return precedence_graph
            
def adjust_TWs(adj_matrix,adj_matrix_tr,TWs): 
    TWs_changed=1
    while TWs_changed:
        TWs_changed=0
        for  k in adj_matrix_tr:
            if len(adj_matrix_tr[k])>0:
                minList=[TWs[i][0]+adj_matrix[i][k] for i in adj_matrix_tr[k]]
                if min(minList)>TWs[k][0]:
                    #print "Increasing tw of %d, which was %d to %d" % (k,TWs[k][0],min(minList))
                    TWs[k][0]=min(minList)
                    TWs_changed=1
    
        for  k in adj_matrix:
            if len(adj_matrix[k])>0 :
                minList=[TWs[j][0]-adj_matrix[k][j] for j in adj_matrix[k]]+[TWs[k][1]]
                if min(minList)>TWs[k][0]:
                    TWs[k][0]=min(minList)
                    #print "Increasing tw of %d, which was %d to %d" % (k,TWs[k][0],min(minList))
                    TWs_changed=1
    
        for k in adj_matrix_tr:
            if len(adj_matrix_tr[k])>0:
                maxList=[TWs[i][1]+adj_matrix[i][k] for i in adj_matrix_tr[k]]+[TWs[k][0]]
                if max(maxList)<TWs[k][1]:
                    #print "Decreasing tw ub of %d, which was %d to %d" % (k,TWs[k][1],max(maxList))
                    TWs[k][1]=max(maxList)
                    TWs_changed=1
    
        for k in adj_matrix:
            if len(adj_matrix[k])>0:
                maxList=[TWs[j][1]-adj_matrix[k][j] for j in adj_matrix[k]]
                if max(maxList)<TWs[k][1]:
                    #print "Decreasing tw ub of %d, which was %d to %d" % (k,TWs[k][1],max(maxList))
                    TWs[k][1]=max(maxList)
                    TWs_changed=1
        if TWs_changed:
            maxi=0
            for i,tw in enumerate(TWs):
                if adj_matrix[i].has_key(vert_num-1):
                    if tw[0]+adj_matrix[i][vert_num-1]>maxi:
                        maxi=tw[0]+adj_matrix[i][vert_num-1]
            TWs[-1][0]=maxi
            process_adj_matrix(adj_matrix,adj_matrix_tr,TWs,old_adj_matrix)
        else:
            retval=process_adj_matrix(adj_matrix,adj_matrix_tr,TWs,old_adj_matrix,1)
            if retval:
                TWs_changed=1

  
#"""
#instance_name = "n40w40.001.txt"
#vert_num,TWs,adj_matrix = readData(instance_name,"Dumas")
instance_name = "rbg034a.tw"
vert_num,TWs,adj_matrix = readData(instance_name,"AFG")
#print adj_matrix[28][1]
vert_num += 1
TWs.append([TWs[0][0],TWs[0][1]])
TWs[0][1]=0
#TWs.append([TWs[0][0],TWs[0][1]])
processed = [0]
toBeUpdated = [i for i in adj_matrix[0]]
adj_matrix[vert_num-1] = {}
#old_adj_matrix = {i:{j:val for j,val in adj_matrix[i].iteritems()} for i in adj_matrix}
nodes = [[i,TWs[i][0],TWs[i][1]] for i in range(vert_num)]
for i in  range(vert_num):
    if adj_matrix[i].has_key(0):
        adj_matrix[i][vert_num-1]=adj_matrix[i].pop(0)
    if adj_matrix[i].has_key(i):
        adj_matrix[i].pop(i)
old_adj_matrix = {i:{j:val for j,val in adj_matrix[i].iteritems()} for i in adj_matrix}
adj_matrix_tr={i:{} for i in adj_matrix}
for i in adj_matrix:
    for j in adj_matrix[i]:
        adj_matrix_tr[j][i]=adj_matrix[i][j]

oldArcAmount=0
for i in old_adj_matrix:
    oldArcAmount+=len(old_adj_matrix[i])
print "Total number of arcs before preprocessing: %d" % oldArcAmount
process_adj_matrix(adj_matrix,adj_matrix_tr,TWs,old_adj_matrix)
TWs_changed=1

adjust_TWs(adj_matrix,adj_matrix_tr,TWs)

g=build_precedence_graph(adj_matrix,adj_matrix_tr,TWs,old_adj_matrix)
processedArcAmount=0
for i in adj_matrix:
    processedArcAmount+=len(adj_matrix[i])
print "Total number of arcs after preprocessing: %d" % processedArcAmount
#time.sleep(3)
print "Starting branch and bound process"
time.sleep(3)
#blub-8
#"""

tsp = Tsp(nodes,adj_matrix,adj_matrix,0,vert_num-1)
tsp.precedence_graph=g
tsp.old_adj_matrix=old_adj_matrix
tsp.update_duals=1
tsp.create_model()
tree = Tree(tsp,0)
t0=time.time()

#tree.dynamic_discovery(5)
#print "Finished dynamic discovery starting branch_and_split"

tree.branch_and_split()
t1=time.time()
print ("___________________________________________________________\n")
print "\n\nTotal time: %f\n\n" %(t1-t0)
print "Average lp time: %f" %(sum(tree.lp_times)/len(tree.lp_times))
print "Total lp time (includes part of the time of adding cuts/splitting): %f" % sum(tree.lp_times)
print "Average simplex iterations: %f" %(sum(tree.simp_iteras)/len(tree.simp_iteras))
print "Total number of nodes visited: %d" % tree.count
print "Node selection time: %f" % tree.node_selection_time
print "Time spend on finding and adding ste cuts: %f" % tree.add_cut_time
print "Time spend on splitting nodes: %f" % tree.split_time