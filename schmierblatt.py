import cplex
class Tsp():
    def __init__(self,nodes,adj_matrix,cost_matrix,depot,goal):
        self.depot = depot
        self.goal = goal
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
        self.idx2name = {}
        self.name2idx = {}
        self.x_names = []
        self.y_names = []
        self.z_names = []
        model = self.model
        model.set_results_stream(None)
        model.set_log_stream(None)
        x_names = ["x_%d_%d" %(i,j) for i in self.indices for j in self.adj_matrix[i]]
        x_obj = [self.cost_matrix[i][j] for i in self.indices for j in self.adj_matrix[i]]
        y_names = ["y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                arc.tail.interval[0],arc.head.interval[0])
                        for key,arc in self.arc_dict.iteritems()]
        z_names = ["z_%d_%d" % (node.name,interval_node.interval[0]) for node in self.nodes for interval_node in node.interval_nodes]

        self.add_x_variables(x_names,[var_type]*len(x_names),x_obj,[0.0]*len(x_names),[0.0]*len(x_names))
        self.add_y_variables(y_names,[var_type]*len(x_names),[0.0]*len(x_names),[0.0]*len(y_names),[0.0]*len(y_names))
        self.add_y_variables(z_names,[var_type]*len(z_names),[0.0]*len(z_names),[0.0]*len(z_names),[0.0]*len(z_names))
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
        self.idx2name = { j : n for j, n in enumerate(model.variables.get_names()) }
        self.name2idx = { n : j for j, n in enumerate(model.variables.get_names()) }
    def add_constraints(self,all_names,allvars,allsenses,allrhs):
        self.model.linear_constraints.add(names=all_names,lin_expr = allvars, 
                                                     senses = allsenses, rhs = allrhs)
        self.const_num += len(allvars)
    def add_x_variables(self,all_names,all_types,all_obj,all_lb,all_ub):
        old_inds=self.model.variables.get_num()
        self.model.variables.add(names = all_names,
                            types=all_types,obj=all_obj,
                            lb = all_lb,ub = all_ub)
        self.x_names += all_names
        self.name2idx.update({name:old_inds+j for j,name in enumerate(all_names)})
        self.idx2name.update({old_inds+j:name for j,name in enumerate(all_names)})
        self.var_num = len(self.name2idx)
    def add_y_variables(self,all_names,all_types,all_obj,all_lb,all_ub):
        old_inds=self.model.variables.get_num()
        self.model.variables.add(names = all_names,
                            types=all_types,obj=all_obj,
                            lb = all_lb,ub = all_ub)
        self.y_names += all_names
        self.name2idx.update({name:old_inds+j for j,name in enumerate(all_names)})
        self.idx2name.update({old_inds+j:name for j,name in enumerate(all_names)})
        self.var_num = len(self.name2idx)
    def add_z_variables(self,all_names,all_types,all_obj,all_lb,all_ub):
        old_inds=self.model.variables.get_num()
        self.model.variables.add(names = all_names,
                            types=all_types,obj=all_obj,
                            lb = all_lb,ub = all_ub)
        self.z_names += all_names
        self.name2idx.update({name:old_inds+j for j,name in enumerate(all_names)})
        self.idx2name.update({old_inds+j:name for j,name in enumerate(all_names)})
        self.var_num = len(self.name2idx)
    def change_variable_name(self,old_var_name,new_var_name):
        #TODO: adapt this to index based access
        self.model.variables.set_names(self.name2idx[old_var_name],new_var_name)
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
        self.interval_nodes = [Interval_node(name,[tw_lb,tw_ub],tw_lb,tw_ub)]
    def split_node(self,split_point):
        idx = self.find_index(split_point)
        old_ub = self.interval_nodes[idx].interval[1] 
        if idx == -1:
            print ("Error: split point %f outside of node %d's interval " % (split_point,self.name) )
            return 0
        self.interval_nodes.insert(idx+1,Interval_node(self.name,[split_point,old_ub],self.tw_lb,self.tw_ub))
        self.interval_nodes[idx] = 0
        self.interval_nodes[idx].interval[1] = split_point
        
        
        #add variable for new node
        if self.tsp.adapt_model:
            self.tsp.add_z_variables(["z_%d_%d" % (self.name,self.interval_nodes[idx+1].id)],['C'],[0.0],[0.0],[1.0])
            self.tsp.model.linear_constraints.set_coefficients([("visit_%d"% self.name,
                                                                 "z_%d_%d" %(self.name,self.interval_nodes[idx+1].id)
                                                                 ,1.0) ])
            names = ["goout_%d_%d" %(self.name,self.interval_nodes[idx+1].id),
                     "goin_%d_%d" %(self.name,self.interval_nodes[idx+1].id)]
            lin_expr=[cplex.SparsePair(["z_%d_%d" % (self.name,self.interval_nodes[idx+1].id),
                                        "z_%d_%d" % (self.name,self.interval_nodes[idx+1].id)],
                                        [-1.0,-1.0])]
            self.tsp.add_constraints(names,lin_expr,["E","E"],[0.0,0.0])

        pop_indices = []
        arc_index = 0
        
        for arc in self.interval_nodes[idx].ingoing_arcs:
            if not arc.head.is_lowest_reachable(arc.tail.interval,arc.length,arc.tail.is_tw_ub()):
                pop_indices.insert(0,arc_index)
                arc.head = self.interval_nodes[idx+1]
                self.interval_nodes[idx+1].ingoing_arcs.append(arc)
                if self.tsp.adapt_model:
                    old_var_name = "y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.id,arc.head.id)
                    const_name = "goin_%d_%d" % (self.name,self.interval_nodes[idx].id)
                    self.tsp.model.linear_constraints.set_coefficients([(const_name,old_var_name,0.0)]) 
                    new_var_name = "y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.id,
                                                      self.interval_nodes[idx+1].id )
                    const_name = "goin_%d_%d" % (self.name,self.interval_nodes[idx+1].id)
                    
                    self.tsp.model.linear_constraints.set_coefficients([(const_name,old_var_name,1.0)]) 
                    self.tsp.change_variable_name(old_var_name,new_var_name)

            else:
                if self.interval_nodes[idx+1].is_lowest_reachable(arc.tail.interval,arc.length,
                                      arc.tail.is_tw_ub()):
                    print "A bug occurred this event should be impossible :("
            arc_index += 1
        for i in pop_indices:
            self.interval_nodes[idx].ingoing_arcs.pop(i)
        
        pop_indices = []
        arc_index = 0
        for arc in self.interval_nodes[idx].outgoing_arcs:
            if not arc.head.is_lowest_reachable(arc.tail.interval,arc.length,arc.tail.is_tw_ub()):
                if self.tsp.adapt_model:
                    new_var_name = "y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,arc.tail.id,arc.head.id)
                    old_var_name = "y_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                                      self.interval_nodes[idx+1].id,arc.head.id)
                    const_name = "goout_%d_%d" % (self.name,self.interval_nodes[idx].id)
                    self.tsp.model.linear_constraints.set_coefficients([(const_name,old_var_name,0.0)])
                    const_name = "goout_%d_%d" % (self.name,self.interval_nodes[idx+1].id)
                    self.tsp.model.linear_constraints.set_coefficients([(const_name,old_var_name,1.0)])
                    self.tsp.change_variable_name(old_var_name,new_var_name)
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
                                                          new_arc.tail.id,new_arc.head.id)
                        self.tsp.add_y_variables([new_var_name],['C'],[0.0],[0.0],[1.0])
                        
                        const_name = "goin_%d_%d" % (new_arc.head.name,new_arc.head.id)
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
                                                              new_arc.tail.id,new_arc.head.id)
                            self.tsp.add_y_variables([new_var_name],['C'],[0.0],[0.0],[1.0])
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
    def find_index(self,split_point,tol=0.000001):
        if split_point < self.tw_lb-tol:
            print ("Error: split point outside of time window (less)")
            return 0
        else:
            if split_point > self.tw_ub+tol:
                print ("Error: split point outside of time window (larger)")
            for i,i_node in enumerate(self.interval_nodes):
                if i_node.interval[0]-tol < split_point and i_node.interval[1]-tol+2*tol*(i_node.is_tw_ub())> split_point:
                    return i
            return -1
    def find_lowest_reachable(self,node,step):
        for interval_node in self.interval_nodes:
            if interval_node.is_lowest_reachable(node.interval,step):
                return 1,interval_node
        return 0,-1   

#gives intersection of two interval (a,b) and (c,d)
def has_intersection(interval_1,interval_2):
    return ((interval_1[0]-interval_2[0]>0 and interval_1[0]-interval_2[1]<0) 
            or (interval_2[0]-interval_1[0]>0 and interval_2[0]-interval_1[1]<0))

#a node in the time expanded graph with an interval assigned to it that may be changed
class Interval_node():
    def __init__(self,name,interval,tsp_node,tol=0.00001):
        self.name = name
        self.id = interval[0]
        self.interval = interval
        self.tsp_node = tsp_node
        self.ingoing_arcs = []
        self.outgoing_arcs = []
    def open_interval(self,tol=0.00001):
        return (self.interval[0]-tol,self.interval[1]+tol*self.is_tw_ub())
    def is_reachable(self,node,shift,tol=0.00001):
        interv1=(node.open_interval()[0]-tol,node.open_interval()[1])
        interv2=(self.is_tw_lb()*self.interval[0]-tol,self.interval[1]+tol*node.is_tw_ub())
        return has_intersection(interv1,interv2,tol=0.0001)   
    def is_tw_lb(self,tol=0.00001):
        return abs(self.interval[0] - self.tsp_node.tw_lb)< tol
    def is_tw_ub(self,tol=0.00001):
        return abs(self.interval[1] - self.tsp_node.tw_ub)< tol
    def is_lowest_reachable(self,node,step,ub_included=0):
        interv1=(node.open_interval()[0]+step,node.open_interval()[1]+step)
        return self.is_reachable(node,step) and (self.is_tw_lb() or not has_intersection((0,self.interval[0]),interv1))