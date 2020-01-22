import cplex
import re
import math
import time
#import sys
import functools

function_times = {}

def register_time(func):
    @functools.wraps(func)
    def wrapper_decorator(*args, **kwargs):
        t0 = time.time()
        value = func(*args, **kwargs)
        if function_times.has_key(func.__name__):
            function_times[func.__name__] += time.time()-t0
        else:
            function_times[func.__name__] = time.time()-t0
        return value
    return wrapper_decorator

def register_time_calls(func):
    @functools.wraps(func)
    def wrapper_decorator(*args, **kwargs):
        value = func(*args, **kwargs)
        if function_times.has_key(func.__name__):
            function_times[func.__name__] += 1
        else:
            function_times[func.__name__] = 1
        return value
    return wrapper_decorator

def solToComponents(solutionStringList,depot=0):
    arcList=[]
    for string in solutionStringList:
        arcList.append([int(re.split("[_]",string)[1]),int(re.split("[_]",string)[2])])
    #notS = range(1,n)
    S=[depot]
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

def solToCyclesMultiVisit(solutionStringList,yStringList,depot=0,goal=51):
    arcList = []
    yArcList = []
    for string in solutionStringList:
        arcList.append([int(re.split("[_]",string)[1]),int(re.split("[_]",string)[2])])
    for string in yStringList:
        yarc = re.split("[_]",string)
        yArcList.append(tuple([int(s) for s in yarc[1:]]))
    head_list = []
    look_up_head = {}
    look_up_tail = {}
    multiVisitList =[]
    #todo depot und goal ausnahme
    for arc in arcList:
        if arc[1]!=51:
            if arc[1] not in head_list:
                head_list.append(arc[1])
            else:
                if arc[1] not in multiVisitList:
                    multiVisitList.append(arc[1])
    for arc in arcList:
        if arc[0] in multiVisitList:
            look_up_tail[tuple(arc)] = 0
        if arc[1] in multiVisitList:
            look_up_head[tuple(arc)] = 0
    
    for arc in look_up_head:
        for yarc in yArcList:
            if yarc[0]==arc[0] and yarc[1]==arc[1]:
                look_up_head[arc] = (yarc[4],yarc[5])
    for arc in look_up_tail:
        for yarc in yArcList:
            if yarc[0]==arc[0] and yarc[1]==arc[1]:
                look_up_tail[arc]= (yarc[2],yarc[3])
    #notS = range(1,n)
    S = [[depot]]
    oS= []
    #if multiVisitList !=[]:
    #    print "Error with multiple visits"
    #    print yArcList
    #    print multiVisitList
    #    print look_up_tail
    #    print look_up_head
    #    time.sleep(3)
    #    time.sleep(3)
    while len(arcList)>0:
        popIndices=[]
        for i in range(len(arcList)):
            if arcList[i][0]== S[len(S)-1][-1]:
                if arcList[i][0] not in multiVisitList:
                    S[len(S)-1].append(arcList[i][1])
                    popIndices.append(i)
                else:
                    tail=S[len(S)-1][-2]
                    if look_up_head[(tail,arcList[i][0])] == look_up_tail[tuple(arcList[i])]:
                        S[len(S)-1].append(arcList[i][1])
                        popIndices.append(i)
        if len(popIndices)== 0:
            if len(S[-1]) == 1:
                S.pop(-1)
                break
            S.append([depot])
            #break
        while len(popIndices)>0:
            arcList.pop(popIndices.pop(-1))
    if len(arcList)>0:
        oS.append(arcList.pop(0))
        while len(arcList)>0:
            popIndices=[]
            for i in range(len(arcList)):
                if arcList[i][0] == oS[len(oS)-1][-1]:
                    oS[len(oS)-1].append(arcList[i][1])
                    popIndices.append(i)
            if len(popIndices)== 0:
                if len(arcList) > 0:
                    oS.append(arcList.pop(0))
                #break
            while len(popIndices)>0:
                arcList.pop(popIndices.pop(-1))
    passCheck = 0
    while passCheck == 0:
        passCheck = 1
        for q in range(len(S)):
            path=S[q]
            for i,node in enumerate(path):
                for j in range(i+1,len(path)):
                    if path[j] == node:
                        passCheck = 0
                        low=i
                        high=j
                        break
                if not passCheck:
                    break
            if not passCheck:
                oS.append([path[low+1:high+1]])
                path=path[0:low]+path[high+1:len(path)]
                
    return S,oS

def solToCycles(solutionStringList,depot=0):
    arcList=[]
    for string in solutionStringList:
        arcList.append([int(re.split("[_]",string)[1]),int(re.split("[_]",string)[2])])
    #notS = range(1,n)
    S = [[depot]]
    oS= []
    while len(arcList)>0:
        popIndices=[]
        for i in range(len(arcList)):
            if arcList[i][0]== S[len(S)-1][-1]:
                S[len(S)-1].append(arcList[i][1])
                popIndices.append(i)
        if len(popIndices)== 0:
            if len(S[-1]) == 1:
                S.pop(-1)
                break
            S.append([depot])
            #break
        while len(popIndices)>0:
            arcList.pop(popIndices.pop(-1))
    if len(arcList)>0:
        oS.append(arcList.pop(0))
        while len(arcList)>0:
            popIndices=[]
            for i in range(len(arcList)):
                if arcList[i][0] == oS[len(oS)-1][-1]:
                    oS[len(oS)-1].append(arcList[i][1])
                    popIndices.append(i)
            if len(popIndices)== 0:
                if len(arcList) > 0:
                    oS.append(arcList.pop(0))
                #break
            while len(popIndices)>0:
                arcList.pop(popIndices.pop(-1))
    passCheck = 0
    while passCheck == 0:
        passCheck = 1
        for q in range(len(S)):
            path=S[q]
            for i,node in enumerate(path):
                for j in range(i+1,len(path)):
                    if path[j] == node:
                        passCheck = 0
                        low=i
                        high=j
                        break
                if not passCheck:
                    break
            if not passCheck:
                oS.append([path[low+1:high+1]])
                path=path[0:low]+path[high+1:len(path)]
                
    return S,oS

def readData(file_name,directory_name ="AFG"):
    
    print "reading "+file_name

    file = open(directory_name+"/"+file_name, "r")
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
    adj_matrix={0:{}}
    if '\r' in depotDists:
        depotDists.remove('\r')
    if '\r' in TWlbs:
        TWlbs.remove('\r')
    if '\r' in TWubs:
        TWubs.remove('\r')
    if '' in profits:
        profits.remove('')
    
    depotDists = [int(i) for i in depotDists]
    popIndices = []
    for j,i in enumerate(depotDists):
        if i>battery_capacity/2:
            popIndices.insert(0,j)
    for j in popIndices:
        depotDists.pop(j)
        TWlbs.pop(j)
        TWubs.pop(j)
        profits.pop(j)
    vertNum = len(depotDists)
    for i in range(1,vertNum+1):
        adj_matrix[0][i]=int(depotDists[i-1])+service_time
        adj_matrix[i] = {}
        for j in range(1,vertNum+1):
            if i!=j:
                adj_matrix[i][j]= service_time+int(depotDists[i-1])+int(depotDists[j-1])
    
    TWs=[[0,time_horizon]]
    for i in range(vertNum):
        TWs.append([int(TWlbs[i])+service_time,int(TWubs[i])+service_time])
    for i in range(len(profits)):
        profits[i]=int(profits[i])
    vertNum+=1
    return vertNum,TWs,adj_matrix,profits,depotDists

def arc_lengths_pre(tail,head,time,battery_level,goal_time_interval,goal_bat_interval):
    tr_time = old_adj_matrix[tail][head]
    tr_bat = battery_matrix[tail][head]
    tr_bat1 = battery_matrix[0][tail]
    tr_bat2 = battery_matrix[0][head]
    if  tr_bat1+battery_level < -0.0001:
        print "Error battery level too low to return to depot"
    forced_load_time = max(goal_bat_interval[0]-(battery_level+tr_bat),0)/load_fac
    forced_wait_time = max(goal_time_interval[0]-(time+tr_time),0)
    load_time=int(math.ceil(max(forced_load_time,forced_wait_time)))
    
    time_delta = int(load_time+tr_time)
    bat_delta = int(min(battery_capacity,battery_level+tr_bat1+load_time*int(load_fac))-battery_level+tr_bat2)
    return time_delta,bat_delta

#returns the deltas to the arriving time,arriving battery
def arc_lengths(tail_bat_node,head_vrp_node):
    tail = tail_bat_node
    head = head_vrp_node
    VRP = tail.interval_node.vrp_node.vrp
    battery_level = tail.bat_interval[1]
    time = tail.interval_node.tw_interval[0]
    tr_time = VRP.adj_matrix[tail.name][head.name]
    tr_bat = VRP.battery_matrix[tail.name][head.name]
    tr_bat1 = VRP.battery_from_depot[tail.name]
    tr_bat2 = VRP.battery_from_depot[head.name]
    if  tr_bat1+battery_level < -0.0001:
        print "Error battery level too low to return to depot"
    forced_load_time = max(head.bat_interval[0]-(battery_level+tr_bat),0)/VRP.loading_speed
    forced_wait_time = max(head.tw_interval[0]-(time+tr_time),0)
    load_time=int(math.ceil(max(forced_load_time,forced_wait_time)))
    
    time_delta = int(load_time+tr_time)
    bat_delta = int(min(VRP.battery_capacity,battery_level+tr_bat1+load_time*int(VRP.loading_speed))-battery_level+tr_bat2)
    return time_delta,bat_delta

class VRP():
    def __init__(self,TWs,bat_intervals,adj_matrix,profits,battery_matrix,depot,goal,loading_speed,battery_capacity):
        self.savedQ = {}
        self.savedZ = {}
        self.multivisit_model = 1
        self.update_duals=1
        self.depot = depot
        self.goal = goal
        self.check_interval_precedences = 1
        self.vehicle_amount = 3
        self.TWs = TWs
        self.n=len(TWs)
        self.indices=range(self.n)
        self.battery_matrix=battery_matrix
        self.battery_from_depot= {i:battery_matrix[0][i] for i in battery_matrix}
        self.nodes = [VRP_node(self,i,TWs[i][0],TWs[i][1],bat_intervals[i][0],bat_intervals[i][1]) for i in self.indices]
        self.arc_dict = {}
        self.loading_speed =loading_speed
        self.battery_capacity = battery_capacity
        self.adj_matrix = adj_matrix
        for i in adj_matrix:
            for j in adj_matrix[i].keys():
                time_delta,bat_delta,load_time = self.calc_deltas(i,j,self.nodes[i].interval_nodes[0].battery_nodes[0].bat_interval[1],
                                                        self.nodes[i].interval_nodes[0].tw_interval[0])
                if ((not self.nodes[j].interval_nodes[0].is_reachable(self.nodes[i].interval_nodes[0],time_delta) )
                    or   (not self.nodes[j].interval_nodes[0].battery_nodes[0].is_reachable(self.nodes[i].interval_nodes[0].battery_nodes[0],bat_delta ))):
                    adj_matrix[i].pop(j)
                else:
                    self.arc_dict[(i,j)]=[]
                    Arc(self.nodes[j].interval_nodes[0].battery_nodes[0],
                            self.nodes[i].interval_nodes[0].battery_nodes[0])
                    #if not arc.head.interval_node.is_lowest_reachable(arc.tail.interval_node,arc.arc_time):
                    #    blub-8
                
        self.adj_matrix_tr = {i:{} for i in self.adj_matrix}
        for i in self.adj_matrix:
            for j in self.adj_matrix[i]:
                self.adj_matrix_tr[j][i]=self.adj_matrix[i][j]
        self.profits = profits
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
        
        model = self.model
        model.set_results_stream(None)
        model.set_log_stream(None)
        model.parameters.advance.set(1)
        #TODO: Find out why warning gets triggered
        model.set_warning_stream(None)
        x_names = ["x_%d_%d" %(i,j) for i in self.indices for j in self.adj_matrix[i]]
        if self.multivisit_model:
            x_obj = [0 for i in self.indices for j in self.adj_matrix[i]]
            self.z_names = []
            z_names = ["z_%d" %(i) for i in self.indices]
            z_obj = [-self.profits[i] for i in self.indices]
        else:
            x_obj = [-self.profits[j] for i in self.indices for j in self.adj_matrix[i]]
        y_names = ["y_%d_%d_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                           arc.tail.id[0],arc.tail.id[1],arc.head.id[0],arc.head.id[1])
                        for key,arc_list in self.arc_dict.iteritems() for arc in arc_list]

        self.add_variables(x_names,[var_type]*len(x_names),x_obj,[0.0]*len(x_names),[1.0]*len(x_names),'x')
        self.add_variables(y_names,['C']*len(y_names),[0.0]*len(y_names),[0.0]*len(y_names),[1.0]*len(y_names),'y')
        if self.multivisit_model:
            self.add_variables(z_names,['C']*len(z_names),z_obj,[0.0]*len(z_names),[1.0]*len(z_names),'z')
        allvars = []
        allrhs = []
        allsenses = []
        all_names = []
        
        if not self.multivisit_model:
            for node in self.nodes:
                if node.name!=self.goal and node.name != self.depot:
                    thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.adj_matrix[node.name]]
                    thecoefs = [1]*len(thevars)
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("L")
                    allrhs.append(1.0)
                    all_names.append("visit_max_once_%d" % node.name)
                if node.name == self.depot:
                    thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.adj_matrix[node.name]]
                    thecoefs = [1]*len(thevars)
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("L")
                    allrhs.append(self.vehicle_amount)
                    all_names.append("visit_max_once_%d" % node.name)
        else:
            for node in self.nodes:
                if node.name!=self.goal and node.name != self.depot:
                    thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.adj_matrix[node.name]]+["z_%d" % node.name]
                    thecoefs = [-1 for node2 in self.adj_matrix[node.name]]+[1]
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("L")
                    allrhs.append(0.0)
                    all_names.append("calc_profits_%d" % node.name)
                if node.name == self.depot:
                    thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.adj_matrix[node.name]]
                    thecoefs = [1]*len(thevars)
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("L")
                    allrhs.append(self.vehicle_amount)
                    all_names.append("visit_max_once_%d" % node.name)
                    

        for node in self.nodes:
            for interval_node in node.interval_nodes:
                for bat_node in interval_node.battery_nodes:
                    if node.name != self.goal and node.name !=self.depot:
                        thevars = ["y_%d_%d_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                               arc.tail.id[0],arc.tail.id[1],arc.head.id[0],arc.head.id[1])
                                    for arc in bat_node.outgoing_arcs]
                        
                        thecoefs = [-1]*len(bat_node.outgoing_arcs)
                        
                            
                        #allvars.append(cplex.SparsePair(thevars,thecoefs))
                        #allsenses.append("E")
                        #allrhs.append(0.0)
                        #all_names.append("goout_%d_%d" %(node.name,interval_node.id))
                        
                        thevars += ["y_%d_%d_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                               arc.tail.id[0],arc.tail.id[1],arc.head.id[0],arc.head.id[1])
                                    for arc in bat_node.ingoing_arcs]
                        
                        thecoefs += [1]*len(bat_node.ingoing_arcs)
        
                            
                        allvars.append(cplex.SparsePair(thevars,thecoefs))
                        allsenses.append("E")
                        allrhs.append(0.0)
                        all_names.append("flow_%d_%d_%d" %(node.name,bat_node.id[0],bat_node.id[1]))
        
        for i,j in self.arc_dict:
            thevars = ["y_%d_%d_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                           arc.tail.id[0],arc.tail.id[1],arc.head.id[0],arc.head.id[1])
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
        self.const_num += len(all_names)
    def add_variables(self,all_names,all_types,all_obj,all_lb,all_ub,var_type):
        old_inds=self.model.variables.get_num()
        self.model.variables.add(names = all_names,
                            types=all_types,obj=all_obj,
                            lb = all_lb,ub = all_ub)
        if var_type=='x':
            self.x_names += all_names
        if var_type=='y':
            self.y_names += all_names
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
    def add_ste_cut(self,S):
        mySet=set(S)
        arc_list = []


        for i in mySet:
            for j in mySet:
                if i != j:
                    if self.adj_matrix[i].has_key(j):
                        arc_list.append("x_%d_%d" % (i,j))
        print arc_list
        self.model.linear_constraints.add(lin_expr=[cplex.SparsePair(arc_list,[1.0]*len(arc_list))],senses=['L'],rhs=[len(mySet)-1])
        self.const_num += 1
    def add_infeasible_path(self,P):
        arc_list = []
        for i in range(len(P)-1):
            arc_list.append("x_%d_%d" % (P[i],P[i+1]) ) 
        self.model.linear_constraints.add(lin_expr=[cplex.SparsePair(arc_list,[1.0]*len(arc_list))],senses=['L'],rhs=[len(P)-1])
        self.const_num += 1
    def add_priority_cut(self,master,slave):
        arc_list = ["x_%d_%d" % (master,j) for j in self.adj_matrix[master]]+["x_%d_%d" % (slave,j) for j in self.adj_matrix[slave]]
        coef_list = [1.0 for j in self.adj_matrix[master]]+[-1.0 for j in self.adj_matrix[slave]]
        self.model.linear_constraints.add(lin_expr=[cplex.SparsePair(arc_list,coef_list)],senses=['G'],rhs=[0])
        self.const_num += 1
        if self.multivisit_model:
            arc_list = ["z_%d" % master, "z_%d" % slave]
            coef_list = [1.0,-1.0]
            self.model.linear_constraints.add(lin_expr=[cplex.SparsePair(arc_list,coef_list)],senses=['G'],rhs=[0])
            self.const_num += 1
        
    def findSplitPoints(self,sol):
        old_split_points = []
        for path in sol:
            new_split_points = []
            battery_level = battery_capacity
            time = 0
            for i in range(len(path)-1):
                time_delta,bat_delta,load_time = self.calc_deltas(path[i],path[i+1],battery_level,time)
                battery_level += bat_delta
                time += time_delta
                if time <= self.nodes[path[i+1]].tw_interval[1]+0.0001 and battery_level >= self.nodes[path[i+1]].bat_interval[0]-0.0001:
                    new_split_points.append((path[i+1],time,battery_level))
                else:
                    #print "Node %d is not reachable from %d with time %d and battery_level %d" % (path[i+1],path[i],time,battery_level)
                    #print "deltas are %d, %d" %(time_delta,bat_delta)
                    #print "battery has to be greater %d, time less than %d"  %(self.nodes[path[i+1]].bat_interval[0],self.nodes[path[i+1]].tw_interval[1])
                    #print "split_points: " + str(new_split_points)
                    old_split_points += new_split_points
                    break
        return old_split_points
    def solToPaths(self,sol):
        old_split_points = []
        for path in sol:
            new_split_points = [(0,0,battery_capacity,"tw_lb: %d" % 0, "tw_ub: %d" % time_horizon,
                                 "bat_lb: %d" % battery_capacity, "bat_ub: %d" % battery_capacity)]
            battery_level = battery_capacity
            time = 0
            for i in range(len(path)-1):
                time_delta,bat_delta,load_time = self.calc_deltas(path[i],path[i+1],battery_level,time)
                battery_level += bat_delta
                time += time_delta
                if time <= self.nodes[path[i+1]].tw_interval[1]+0.0001 and battery_level >= self.nodes[path[i+1]].bat_interval[0]-0.0001:
                    new_split_points.append((path[i+1],time,battery_level,
                                             "tw_lb: %d" %self.nodes[path[i+1]].tw_interval[0] , "tw_ub: %d" % self.nodes[path[i+1]].tw_interval[1],
                                             "bat_lb: %d" %self.nodes[path[i+1]].bat_interval[0], "bat_ub: %d" % self.nodes[path[i+1]].bat_interval[1]))
                else:
                    
                    old_split_points.append("Error %d %d, %d,%d" %(time,battery_level,load_time,time_delta))
                    break
            old_split_points.append(new_split_points)
        return old_split_points
    def calc_deltas(self,tail_idx,head_idx,battery_level,time):
        tail = self.nodes[tail_idx]
        head = self.nodes[head_idx]
        tr_time = old_adj_matrix[tail.name][head.name]
        tr_bat = self.battery_matrix[tail.name][head.name]
        tr_bat1 = self.battery_from_depot[tail.name]
        tr_bat2 = self.battery_from_depot[head.name]
        if  tr_bat1+battery_level < -0.0001:
            print "Error battery level too low to return to depot"
        forced_load_time = max(head.bat_interval[0]-(battery_level+tr_bat),0)/self.loading_speed
        forced_wait_time = max(head.tw_interval[0]-(time+tr_time),0)
        load_time = int(math.ceil(max(forced_load_time,forced_wait_time)))
        time_delta = int(load_time+tr_time)
        bat_delta = int(min(self.battery_capacity,battery_level+tr_bat1+load_time*int(self.loading_speed))-battery_level+tr_bat2)
        return time_delta,bat_delta,load_time
    def testNodeGraph(self):
        for vrp_node in self.nodes:
            for interval_node in vrp_node.interval_nodes:
                for battery_node in interval_node.battery_nodes:
                    for arc in battery_node.ingoing_arcs+battery_node.outgoing_arcs:
                        time_delta,bat_delta = arc_lengths(arc.tail,arc.head.interval_node.vrp_node)
                        if abs(time_delta-arc.arc_time)> 0.1:
                            print "Error: arcs time length is not correctly stored"
                        if abs(bat_delta-arc.arc_bat)> 0.1:
                            print "Error: arcs bat length is not correctly stored"
                        if not arc.head.is_maximal_reachable_battery_node(arc.tail,bat_delta):
                            print "Error: arc is not maximal reachable"
                        if not arc.head.interval_node.is_lowest_reachable(arc.tail.interval_node,time_delta):
                            print "Tail interval: [%d,%d]" % tuple(arc.tail.interval_node.tw_interval)
                            print "Head interval: [%d,%d]" % tuple(arc.head.interval_node.tw_interval)
                            print "Heads complete TW: [%d,%d]" % tuple(arc.head.interval_node.vrp_node.tw_interval)
                            print "Pure travel time: %d" % self.adj_matrix[arc.tail.name][arc.head.name]
                            print "Travel + loading time: %d" % time_delta
                            print "Errror: arcs interval nodes are not lowest reachable"
    def testModel(self):
        return

            
                    
#class for arcs between interval nodes adding itself to the respective in- and outgoing lists
#and also to the arc dictionary of the vrp class
class Arc():
    def __init__(self,head,tail):
        self.head = head
        self.head.ingoing_arcs.append(self)
        self.tail = tail
        self.tail.outgoing_arcs.append(self)
        self.arc_time,self.arc_bat = arc_lengths(tail,head.interval_node.vrp_node)
        self.tail.interval_node.vrp_node.vrp.arc_dict[tail.name,head.name].append(self)


#class for nodes in the original vrp-graph containing a subset of nodes of the time expanded graph
#also allows for changes in the time expanded graph
class VRP_node():
    def __init__(self,vrp,name,tw_lb,tw_ub,bat_lb,bat_ub):
        self.vrp = vrp
        self.name = name
        self.tw_lb = tw_lb
        self.tw_ub = tw_ub
        self.bat_lb = bat_lb
        self.bat_ub = bat_ub
        self.bat_interval = [bat_lb,bat_ub]
        self.tw_interval = [tw_lb,tw_ub]
        self.interval_nodes = [Interval_node(name,[tw_lb,tw_ub],self,1,1,bat_interval=[bat_lb,bat_ub])]
    def idUsed(self,ID):
        for i in self.interval_nodes:
            if i.id == ID:
                return 1
        return 0
    def split_node_both(self,split_point_tw,split_point_bat,dual_value_locations=[]):
        if self.name == self.vrp.goal:
            return 1
        tw_idx,split_tw = self.find_tw_index(split_point_tw)
        #blub-8
        #print tw_idx
        if split_tw:
            new_tw_idx = tw_idx+1
        else:
            new_tw_idx = tw_idx
        bat_idx,split_bat = self.find_bat_index(split_point_bat,tw_idx)
        #print split_bat
        if split_bat:
            old_bat_idx = bat_idx+1
        else:
            old_bat_idx= bat_idx
        if split_bat+split_tw < 1:
            return 0
            print "Error neither battery window nor time_window split"
        old_ub_tw = self.interval_nodes[tw_idx].tw_interval[1]
        old_lb_bat = self.interval_nodes[tw_idx].battery_nodes[bat_idx].bat_interval[0]
        
        #ToDo: Spalten von Batterieinterval, wenn es Zeitpunkt schon gibt, eventuell andere Funktion hierfuer
        if split_tw:
            self.interval_nodes.insert(new_tw_idx,Interval_node(self.name,[split_point_tw,old_ub_tw],self,0,self.interval_nodes[tw_idx].is_tw_ub()))
            for bat_node in self.interval_nodes[tw_idx].battery_nodes:
                self.interval_nodes[new_tw_idx].battery_nodes.append(Battery_node(self.name,
                                   list(bat_node.bat_interval),self.interval_nodes[new_tw_idx],bat_node.is_bat_lb(),bat_node.is_bat_ub()))
        if split_bat:
            self.interval_nodes[new_tw_idx].battery_nodes.insert(bat_idx,
                           Battery_node(self.name,[old_lb_bat,split_point_bat],self.interval_nodes[new_tw_idx],
                                        self.interval_nodes[new_tw_idx].battery_nodes[bat_idx].is_bat_lb(),0))
        #ToDo: Neuen Batterieknoten an split_point_bat_einfuegen
        if split_tw:
            self.interval_nodes[tw_idx].is_ub=0
            self.interval_nodes[tw_idx].tw_interval[1] = split_point_tw
        if split_bat:
            self.interval_nodes[new_tw_idx].battery_nodes[old_bat_idx].is_lb = 0
            self.interval_nodes[new_tw_idx].battery_nodes[old_bat_idx].bat_interval[0] = split_point_bat
        #print self.interval_nodes[idx].id
        #print split_point
        #add variable for new node
        #print "Splitting node: %d" % self.name
        
        node_counter = 0
        bat_idx_reached = 0
        #necessary to insert all correct dual values, because the new battery list has 1 extra node in position bat_idx
        if self.vrp.adapt_model:
            if self.name!=self.vrp.goal:
                if split_tw:
                    for bat_node in self.interval_nodes[new_tw_idx].battery_nodes:
                        names = ["flow_%d_%d_%d" %(self.name,bat_node.id[0],bat_node.id[1])]
                        lin_expr=[cplex.SparsePair([],[] )]
                        self.vrp.add_constraints(names,lin_expr,["E"],[0.0])
                        if self.vrp.update_duals:
                            id1 = self.interval_nodes[tw_idx].battery_nodes[ node_counter-bat_idx_reached ].id[0]
                            id2 = self.interval_nodes[tw_idx].battery_nodes[ node_counter-bat_idx_reached ].id[1]
                            dual_value_locations += [self.vrp.const_name2idx["flow_%d_%d_%d"%(self.name,id1,id2)]]
                        if split_bat:
                            if node_counter == bat_idx:
                                bat_idx_reached = 1
                        node_counter += 1
                else:
                    if split_bat:
                        bat_node = self.interval_nodes[new_tw_idx].battery_nodes[bat_idx]
                        names = ["flow_%d_%d_%d" %(self.name,bat_node.id[0],bat_node.id[1])]
                        lin_expr=[cplex.SparsePair([],[] )]
                        self.vrp.add_constraints(names,lin_expr,["E"],[0.0])
                        if self.vrp.update_duals:
                            id1 = self.interval_nodes[tw_idx].battery_nodes[old_bat_idx].id[0]
                            id2 = self.interval_nodes[tw_idx].battery_nodes[old_bat_idx].id[1]
                            dual_value_locations += [self.vrp.const_name2idx["flow_%d_%d_%d"%(self.name,id1,id2)]]
                

        
        for idx in range(len(self.interval_nodes[tw_idx].battery_nodes)):
            pop_indices = []
            arc_index = 0
            for arc in self.interval_nodes[tw_idx].battery_nodes[idx].ingoing_arcs:
                head_is_lowest_reachable = arc.head.interval_node.is_lowest_reachable(arc.tail.interval_node,arc.arc_time)
                head_is_maximal_bat =  arc.head.is_maximal_reachable_battery_node(arc.tail,arc.arc_bat)
                if (split_tw and not head_is_lowest_reachable) or (
                            ((not split_tw) and not head_is_maximal_bat)):
                    if self.vrp.adapt_model:
                        old_var_name = "y_%d_%d_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                               arc.tail.id[0],arc.tail.id[1],arc.head.id[0],arc.head.id[1])
                    pop_indices.insert(0,arc_index)
                    old_head = arc.head
                    if not head_is_lowest_reachable:
                        is_reachable,new_interval_node = self.find_lowest_reachable(arc.tail.interval_node,arc.arc_time)
                    else:
                        is_reachable,new_interval_node = 1,arc.head.interval_node
                    if is_reachable:
                        is_reachable,arc.head = new_interval_node.find_maximal_reachable_battery_node(arc.tail,arc.arc_bat)
                        if not is_reachable:
                            print "Error no battery node found"
                            print self.vrp.nodes[old_head.name].bat_interval
                    else:
                        print "Error no interval node found"
                        print "Tail interval" + str(arc.tail.interval_node.tw_interval)
                        print "Shifted by: " + str(arc.arc_time)
                        print "Original arc time: " + str(self.vrp.adj_matrix[arc.tail.name][arc.head.name])
                        print "Head interval:" + str(self.tw_interval)
                        time.sleep(10)

                    

                    arc.head.ingoing_arcs.append(arc)
                    if self.vrp.adapt_model:
                        #print "new_arc: %d,%d,%d,%d" %(arc.tail.name, arc.head.name,arc.tail.id,arc.head.id)
                        new_var_name = "y_%d_%d_%d_%d_%d_%d" % (arc.tail.name,arc.head.name,
                                               arc.tail.id[0],arc.tail.id[1],
                                                              arc.head.id[0],arc.head.id[1] )
                        if self.name!=self.vrp.goal:
                            const_name = "flow_%d_%d_%d" % (self.name,old_head.id[0],old_head.id[1])
                            self.vrp.model.linear_constraints.set_coefficients([(const_name,old_var_name,0.0)]) 
                            
                            const_name = "flow_%d_%d_%d" % (self.name,arc.head.id[0],arc.head.id[1])
                            
                            self.vrp.model.linear_constraints.set_coefficients([(const_name,old_var_name,1.0)]) 
                            self.vrp.change_variable_name(old_var_name,new_var_name)
                arc_index += 1
            for i in pop_indices:
                self.interval_nodes[tw_idx].battery_nodes[idx].ingoing_arcs.pop(i)
        
        new_arc_list=[]
        #ToDo maybe optimize breaks for not testing too many nodes
        for node_index in self.vrp.adj_matrix[self.name]:
            vrp_node_head = self.vrp.nodes[node_index]
            if self.interval_nodes[new_tw_idx].tw_interval[0]+self.vrp.adj_matrix[self.name][node_index]>vrp_node_head.tw_interval[1]+0.0001:
                continue
            if split_tw:
                for bat_node in self.interval_nodes[new_tw_idx].battery_nodes:
                    #print bat_node.interval_node.tw_interval
                    arc_time,arc_bat = arc_lengths(bat_node,vrp_node_head)
                    #print (arc_time,arc_bat)
                    
                    #print (self.interval_nodes[new_tw_idx].tw_interval,vrp_node_head.tw_interval)
                    is_reachable,new_interval_node_head = self.vrp.nodes[vrp_node_head.name].find_lowest_reachable(self.interval_nodes[new_tw_idx],
                                                        arc_time)
                    if is_reachable:
                        is_reachable, new_head = new_interval_node_head.find_maximal_reachable_battery_node(bat_node,arc_bat)
                        if is_reachable:
                            #print (bat_node.name,new_head.name)
                            new_arc_list.append( 
                                        Arc(new_head,bat_node)
                                )
            else:
                if split_bat:
                    bat_node = self.interval_nodes[new_tw_idx].battery_nodes[bat_idx]
                    arc_time,arc_bat = arc_lengths(bat_node,vrp_node_head)
                    is_reachable,new_interval_node_head = self.vrp.nodes[vrp_node_head.name].find_lowest_reachable(self.interval_nodes[new_tw_idx],
                                                        arc_time)
                    if is_reachable:
                        is_reachable, new_head = new_interval_node_head.find_maximal_reachable_battery_node(bat_node,arc_bat)
                        if is_reachable:
                            new_arc_list.append( 
                                        Arc(new_head,bat_node)
                                )
                
        
        if self.vrp.adapt_model:
            for new_arc in new_arc_list:             
                #print "new_arc: %d,%d,%d,%d" %(new_arc.tail.name, new_arc.head.name,new_arc.tail.id,new_arc.head.id)
                new_var_name = "y_%d_%d_%d_%d_%d_%d" % (new_arc.tail.name,new_arc.head.name,
                                                  new_arc.tail.id[0],new_arc.tail.id[1],new_arc.head.id[0],new_arc.head.id[1])
                self.vrp.add_variables([new_var_name],['C'],[0.0],[0.0],[1.0],'y')
                if new_arc.head.name != self.vrp.goal:
                    const_name = "flow_%d_%d_%d" % (new_arc.head.name,new_arc.head.id[0],new_arc.head.id[1])
                    self.vrp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)])
                if new_arc.tail.name != self.vrp.depot:
                    const_name = "flow_%d_%d_%d" % (new_arc.tail.name,new_arc.tail.id[0],new_arc.tail.id[1])
                    self.vrp.model.linear_constraints.set_coefficients([(const_name,new_var_name,-1.0)]) 
                    
                const_name = "arcuse_%d_%d" % (new_arc.tail.name,new_arc.head.name)
                self.vrp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)]) 
        return 1
    def find_tw_index(self,split_point,tol=0.000001):
        if split_point < self.tw_lb-tol:
            print ("Error: split point outside of time window (less)")
            return 0,0
        else:
            if split_point > self.tw_ub+tol:
                print ("Error: split point outside of time window (larger)")
            for i,i_node in enumerate(self.interval_nodes):
                if i_node.tw_interval[0]+tol > split_point and i_node.tw_interval[0]-tol < split_point:
                    return i,0
                if i_node.tw_interval[0]+tol < split_point and i_node.tw_interval[1]-tol+2*tol*(i_node.is_tw_ub())> split_point:
                    return i,1
                
            return -1,0
    def find_bat_index(self,split_point,tw_index,tol=0.000001):
        if split_point < self.bat_lb-tol:
            print ("Error: split point outside of battery window (less)")
            return 0,0
        else:
            if split_point > self.bat_ub+tol:
                print ("Error: split point outside of battery window (larger)")
                return 0,0
            for i,i_node in enumerate(self.interval_nodes[tw_index].battery_nodes):
                if  i_node.bat_interval[1]+tol > split_point and i_node.bat_interval[1]-tol < split_point:
                    return i,0
                if i_node.bat_interval[0]+tol-2*tol*(i_node.is_bat_lb()) < split_point and i_node.bat_interval[1]+tol > split_point:
                    return i,1
                

    def find_index_ub(self,split_point,tol=0.000001):
        if split_point < self.tw_lb-tol:
            print ("Error: split point outside of time window (less)")
            return 0
        else:
            if split_point > self.tw_ub+tol:
                print ("Error: split point outside of time window (larger)")
            for i,i_node in enumerate(self.interval_nodes):
                if (i_node.interval[0]+tol < split_point and i_node.interval[1]-tol+2*tol*(i_node.is_tw_ub())> split_point 
                     and i_node.interval[1]-tol > split_point):
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
    def __init__(self,name,tw_interval,VRP_node,is_tw_lb,is_tw_ub,tol=0.00001,bat_interval=[]):
        self.name = name
        self.id = tw_interval[0]
        self.tw_interval = tw_interval
        self.bat_interval = bat_interval
        self.vrp_node = VRP_node
        self.is_ub = is_tw_ub
        self.is_lb = is_tw_lb
        self.big_num = 100000
        if bat_interval!=[]:
            self.battery_nodes = [Battery_node(name,[bat_interval[0],bat_interval[1]],self,1,1)]
        else:
            self.battery_nodes = []
    def is_reachable(self,node,shift_tw,tol=0.00001):
        interv_tw1= [node.tw_interval[0]+shift_tw,node.tw_interval[1]+shift_tw-2*tol+4*tol*node.is_tw_ub()]
        interv_tw2= [(not self.is_tw_lb())*self.tw_interval[0]-tol,self.tw_interval[1]-2*tol+4*tol*self.is_tw_ub()]
        #print interv_tw1
        #print interv_tw2
        return has_intersection(interv_tw1,interv_tw2)  
    def is_tw_lb(self,tol=0.00001):
        return self.is_lb
    def is_tw_ub(self,tol=0.00001):
        return self.is_ub
    def is_lowest_reachable(self,node,step_tw,tol=0.00001):
        interv_tw=(node.tw_interval[0]+step_tw,node.tw_interval[1]-2*tol+4*tol*node.is_tw_ub()+step_tw)
        """
        if not (self.is_reachable(node,step_tw) and (
                self.is_tw_lb() or not has_intersection((0,self.tw_interval[0]-tol),interv_tw))):
            print "head interval: %d,%d" % tuple(self.tw_interval)
            print "shifted tw_interval: %f,%f" % interv_tw
            print has_intersection((0,self.tw_interval[0]-tol),interv_tw)
            print self.is_reachable(node,step_tw)
        """
        return (self.is_reachable(node,step_tw) and (
                self.is_tw_lb() or not has_intersection((0,self.tw_interval[0]-tol),interv_tw)))
    def find_maximal_reachable_battery_node(self,battery_node,bat_step):
        for bat_node in self.battery_nodes:
            if bat_node.is_maximal_reachable_battery_node(battery_node,bat_step):
                return 1,bat_node
        return 0,-1

class Battery_node():
    def __init__(self,name,bat_interval,interval_node,is_bat_lb,is_bat_ub,tol=0.00001):
        self.name = name
        self.id = (interval_node.id,bat_interval[-1])
        self.bat_interval = bat_interval
        self.interval_node = interval_node
        self.ingoing_arcs = []
        self.outgoing_arcs = []
        self.is_ub = is_bat_ub
        self.is_lb = is_bat_lb
        self.big_num = 100000
    def is_reachable(self,node,shift_bat,tol=0.00001):
        #print self.bat_interval
        interv_bat1=[node.bat_interval[0]+shift_bat+2*tol-4*tol*node.is_bat_lb(),node.bat_interval[1]+shift_bat]
        interv_bat2=[self.bat_interval[0]+2*tol-4*tol*self.is_bat_lb(),(not self.is_bat_ub())*self.big_num+self.bat_interval[1]+tol]
        return  has_intersection(interv_bat1,interv_bat2)  
    def is_bat_lb(self,tol=0.00001):
        return self.is_lb
    def is_bat_ub(self,tol=0.00001):
        return self.is_ub
    def is_maximal_reachable_battery_node(self,node,step_bat,tol=0.00001):
        interv_bat=(node.bat_interval[0]+step_bat+2*tol-4*tol*node.is_bat_lb(),node.bat_interval[1]+step_bat)
        return self.is_reachable(node,step_bat) and (
                        self.is_bat_ub() or not has_intersection((self.bat_interval[1]+tol,self.big_num),interv_bat)
                        )
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
    def update_dual_values(self,dual_value_locations):
        for ind in dual_value_locations:
            self.dual_values_model.append(self.dual_values_model[ind])
    def solve_lp_relaxation(self,dual_value_locations=[],warm_start=1,tol=0.000001):
        
        vrp = self.tree.vrp
        if vrp.update_duals:
            self.update_dual_values(dual_value_locations)
        model = vrp.model
        model.linear_constraints.add(names = self.branch_names,lin_expr = self.branch_lin_exprs,
                                         senses = self.branch_senses,rhs = self.branch_rhs)
        if self.tree.vrp.update_duals and self.dual_values_model != 0:
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
        #model.parameters.preprocessing.presolve.set(0)
        t0 = time.time()
        if self.tree.vrp.adapt_model:
            feas = vrp.solve_model()
        else:
            feas = 1
            vrp.model.solve()
        self.updated_lp_relaxation = 1
        self.tree.lp_times.append((time.time()-t0))
        self.tree.lp_time += time.time()-t0
        self.tree.simp_iteras.append(model.solution.progress.get_num_iterations())
        model.parameters.advance.set(0)
        if not feas:
            self.lower_bound = 1000000
            model.linear_constraints.delete(xrange(model.linear_constraints.get_num()-len(self.branches),model.linear_constraints.get_num()))
            return 0
        else:
            t_sol_eval0=time.time()
            solution = model.solution
            if self.tree.vrp.adapt_model:
                dual_values = solution.get_dual_values()
            else:
                dual_values = []
            self.dual_values_model = dual_values[:self.tree.vrp.const_num]
            self.dual_values_branches = dual_values[self.tree.vrp.const_num:]
            self.primal_values = solution.get_values()
            self.lower_bound = solution.get_objective_value()
            if self.tree.vrp.adapt_model:
                self.reduced_costs = solution.get_reduced_costs()
            else:
                self.reduced_costs = []
            #TODO: Check this reduced cost fixing
            oldLen = len(self.branches)

            self.primal_x_values = {name:self.primal_values[self.tree.vrp.name2idx[name]] for name in self.tree.vrp.x_names 
                                    if self.primal_values[self.tree.vrp.name2idx[name]]>tol}
            self.primal_y_values = {name:self.primal_values[self.tree.vrp.name2idx[name]] for name in self.tree.vrp.y_names
                                    if self.primal_values[self.tree.vrp.name2idx[name]]>tol}
            if self.tree.reduced_cost_fixing:
                for i,val in enumerate(self.reduced_costs):
                    if val > tol:
                        if self.tree.vrp.idx2name[i][0] == 'x' and self.lower_bound + val >= self.tree.ub:
                            self.branches.append(Branch(self.tree.vrp.idx2name[i],'L',0.0))
                            self.branch_names.append(self.branches[-1].name)
                            self.branch_lin_exprs.append(self.branches[-1].lin_expr)
                            self.branch_senses.append(self.branches[-1].sense)
                            self.branch_rhs.append(self.branches[-1].rhs)
                            self.dual_values_branches.append(0.0)
                    
                    if val < -tol and self.primal_values[i] > 1-tol:
                        if (self.tree.vrp.idx2name[i][0] == 'x' and self.lower_bound - val >= self.tree.ub):
                            #print "Fixing variable to 1"
                            self.branches.append(Branch(self.tree.vrp.idx2name[i],'G',1.0))
                            self.branch_names.append(self.branches[-1].name)
                            self.branch_lin_exprs.append(self.branches[-1].lin_expr)
                            self.branch_senses.append(self.branches[-1].sense)
                            self.branch_rhs.append(self.branches[-1].rhs)
                            self.dual_values_branches.append(0.0)
                
            for key,val in self.primal_x_values.iteritems():
                if abs(0.5-val)<0.5-tol:
                    self.fractionals[key] = val
            for key,val in self.primal_y_values.iteritems():
                if abs(0.5-val)<0.5-tol:
                    self.y_fractionals[key] = val
            self.tree.sol_eval_time += time.time() - t_sol_eval0
        model.linear_constraints.delete(xrange(model.linear_constraints.get_num()-oldLen,model.linear_constraints.get_num()))
        #self.tree.total_relaxation_time += time.time() - t0
        return 1
    def is_x_integer(self):
        if len(self.fractionals) == 0:
            return 1
        else:
            return 0
    def choose_branch_var_degree(self):
        maxscore = -100
        chosen_var = -1
        chosen_val = -1
        for var_name,var_val in self.fractionals.iteritems():
            i,j=int(re.split("[_]",var_name)[1]),int(re.split("[_]",var_name)[2])
            score = len(self.tree.vrp.adj_matrix[i])+len(self.tree.vrp.adj_matrix_tr[j])
            if score > maxscore:
                maxscore = score
                chosen_var = var_name
                chosen_val = var_val
        return chosen_var,chosen_val
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
        if chosen_var == -1:
            print "Error"
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

class Tree():
    def __init__(self,vrp,start_control):
        self.print_interval = 10
        self.add_all_split_points=1
        self.lbs = []
        #self.print_switch = 0
        self.psi_avg = 0
        self.reduced_cost_fixing = 1
        self.lp_times=[]
        self.node_count = 1
        self.refinement_count = 0
        self.cut_count = 0
        self.lp_time = 0.0
        self.lp_no_dual_times=[]
        self.simp_iteras=[]
        self.simp_nd_iteras=[]
        self.start_control = start_control
        self.branch_variable_selection_time = 0.0
        self.node_selection_time = 0.0
        self.total_relaxation_time = 0.0
        self.find_pi_sigma_time = 0.0
        self.find_ste_time = 0.0
        self.add_cut_time = 0.0
        self.split_time = 0.0
        self.find_split_time = 0.0
        self.sol_eval_time = 0.0
        self.add_start_time = 0.0
        self.closed_nodes = []
        self.vrp = vrp
        self.vrp.tree = self
        self.ub = 100000
        self.lb = -50
        self.root = Tree_node(self,[])
        self.open_nodes = [self.root]
        self.branch_history = {key:[] for key in vrp.x_names}
        self.time_limit = 7200
        self.count = 0
        self.root_count = 0
        self.refinement = 0  
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
            if  self.lb < minVal:
                self.lb=minVal
        if selection == 3:
            minInd = 0
            minVal= 100000000
            minLb = 100000000
            for i,node in enumerate(self.open_nodes):
                if node.lower_bound < minVal:
                    minInd = i
                    minVal = node.lower_bound
                    minLb = node.lower_bound
            if self.lb < minVal:
                self.lb = minLb
        self.node_selection_time += time.time() - t0
        return self.open_nodes.pop(minInd)
    def branch_and_refine(self):
        if self.use_best_heuristic:
            self.ub = self.heuristic_ub
        """
        for i in self.vrp.adj_matrix:
            for j in self.vrp.adj_matrix:
                if (not self.vrp.precedence_graph[j].has_key(i) )and (not self.vrp.precedence_graph[i].has_key(j)) and i!=j:
                    self.vrp.add_ste_cut([i,j])
                    #print "cut added"
        """
        while len(self.open_nodes)>0 and self.lp_time < self.time_limit:
            self.lbs.append(self.lb)
            self.count += 1
            addedCuts = 0
            #splitCount = 0
            #splitNodesForNonInteger = root_relaxation
            
            
            node = self.choose_node()
            #print "node chosen"
            #time.sleep(0.2)
            if node.updated_lp_relaxation == 0 or 1:
                node.solve_lp_relaxation()
            if not node.feasible or node.lower_bound >= self.ub-0.99:
                continue
            self.conditional_print("Current lower bound: %f" % (self.lb))
            self.conditional_print("Current best upper Bound: %f " % (self.ub))
            self.conditional_print("Number of open nodes: %d" % len(self.open_nodes))
            print node.lower_bound
            if len(node.fractionals)>0:
                #branching step
                branch_var,branch_val = node.choose_branch_var()
                print "branching"
                f_1 = 1.0-branch_val
                f_0 = branch_val
                if self.vrp.update_duals:
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
                        if new_node1.lower_bound < self.ub-0.99:
                            self.open_nodes.append(new_node1)
                #for node in self.open_nodes:
                #    node.solve_lp_relaxation()
                continue
            #this part should only be reached for integer solutions
            t_ste_0 = time.time()
            
            #print "searching for cycles"
            #print node.primal_x_values
            sol,shortCycles = solToCyclesMultiVisit(node.primal_x_values,node.primal_y_values)
            self.find_ste_time +=time.time()-t_ste_0
            
            #print sol
            #print shortCycles
            
            
            #Segment to add STE Cuts, if a cut is added loop is continued
            if len(shortCycles)>0:
                for S in shortCycles:
                    print "Adding STE cut for set: " + str(S)
                    self.vrp.add_ste_cut(S)
                    self.cut_count +=1
                    #cleanup += 1
                t_add_cut0 = time.time()
                pop_indices=[]
                self.open_nodes.append(node)
                for i,node2 in enumerate(self.open_nodes):
                    if self.vrp.update_duals:
                        node2.dual_values_model += [0.0]*addedCuts
                    node2.updated_lp_relaxation = 0
                    #node2.solve_lp_relaxation()
                    if not node2.feasible or node2.lower_bound >= self.ub:
                        pop_indices.append(i)
                while (len(pop_indices)>0):
                    self.open_nodes.pop(pop_indices.pop(-1))
                self.add_cut_time += time.time() - t_add_cut0 
                continue
            
            #Segment to split nodes, if nodes are split loop is continued
            t_find_split0 = time.time()
            split_points = self.vrp.findSplitPoints(sol)#splitPoints pruft, ob die Wege in 
            #                                #S moglich sind und returned tuple hinzuzufugender Punkte der unzulassigen Wege
            #print split_points
            #time.sleep(1)
            dual_value_locations=[]
            if len(split_points)>0:
                print "Splitting nodes"
                #print split_points
                #time.sleep(10)
                self.refinement_count += 1
                #for i,node2 in enumerate(self.open_nodes):
                #    if self.vrp.update_duals:
                #        node2.dual_values_model += [0.0]*addedCuts
                actually_split = 0
                if self.vrp.update_duals:
                    for i,t,b in split_points:
                        actually_split += self.vrp.nodes[i].split_node_both(t,b,dual_value_locations)
                else:
                    for i,t,b in split_points:
                        actually_split += self.vrp.nodes[i].split_node_both(t,b)
                if actually_split == 0:
                    print "Error no node split lb: " + str(node.lower_bound)
                    print str(node.primal_y_values)
                    #self.vrp.testNodeGraph()
                    time.sleep(3)
                    time.sleep(5)



                pop_indices=[]
                #node.solve_lp_relaxation(dual_value_locations)
                self.open_nodes.insert(0,node)
                for i,node2 in enumerate(self.open_nodes):
                    #node2.solve_lp_relaxation(dual_value_locations)
                    node2.update_dual_values(dual_value_locations)
                    node2.updated_lp_relaxation = 0
                    #node2.solve_lp_relaxation(dual_value_locations)
                    #node2.updated_lp_relaxation = 1
                    if not node2.feasible or node2.lower_bound >= self.ub-0.99:#TODO: Adapt this to non integer objective
                        pop_indices.append(i)
                while (len(pop_indices)>0):
                    self.open_nodes.pop(pop_indices.pop(-1))
                self.split_time += time.time()-t_find_split0
                continue
            
            
            #segment if no cut was added and nodes were not split
            if node.lower_bound < self.ub-0.99:
                self.solution_path = self.vrp.solToPaths(sol)
                print "Integer feasible solution found, objective: %f" %node.lower_bound
                self.ub = node.lower_bound
                self.solution_node = node
            pop_indices=[]
            #pruning of tree
            for i,node2 in enumerate(self.open_nodes):
                #node2.solve_lp_relaxation()
                if not node2.feasible or node2.lower_bound >= self.ub-0.99:
                    pop_indices.append(i)
            while (len(pop_indices)>0):
                self.open_nodes.pop(pop_indices.pop(-1))
    def dynamic_discovery(self):
        if self.use_best_heuristic:
            self.ub = self.heuristic_ub
        """
        for i in self.vrp.adj_matrix:
            for j in self.vrp.adj_matrix:
                if (not self.vrp.precedence_graph[j].has_key(i) )and (not self.vrp.precedence_graph[i].has_key(j)) and i!=j:
                    self.vrp.add_ste_cut([i,j])
                    #print "cut added"
        """
        time_feasible = 0
        while not time_feasible:
            while len(self.open_nodes)>0 and self.lp_time < self.time_limit:
                self.lbs.append(self.lb)
                self.count += 1
                addedCuts = 0
                #splitCount = 0
                #splitNodesForNonInteger = root_relaxation
                self.conditional_print("Current lower bound: %f" % (self.lb))
                self.conditional_print("Current best upper Bound: %f " % (self.ub))
                self.conditional_print("Number of open nodes: %d" % len(self.open_nodes))
                
                
                
                node = self.choose_node()
                #print "node chosen"
                #time.sleep(0.2)
                if node.updated_lp_relaxation == 0:
                    node.solve_lp_relaxation()
                if not node.feasible or node.lower_bound >= self.ub:
                    continue
                print node.lower_bound
                if len(node.fractionals)>0:
                    #branching step
                    branch_var,branch_val = node.choose_branch_var()
                    print "branching"
                    f_1 = 1.0-branch_val
                    f_0 = branch_val
                    if self.vrp.update_duals:
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
                            if new_node1.lower_bound < self.ub-0.99:
                                self.open_nodes.append(new_node1)
                    #for node in self.open_nodes:
                    #    node.solve_lp_relaxation()
                    continue
                #this part should only be reached for integer solutions
                t_ste_0 = time.time()
                
                #print "searching for cycles"
                #print node.primal_x_values
                sol,shortCycles = solToCycles(node.primal_x_values)
                self.find_ste_time +=time.time()-t_ste_0
                
                #print sol
                #print shortCycles
                
                
                #Segment to add STE Cuts, if a cut is added loop is continued
                if len(shortCycles)>0:
                    for S in shortCycles:
                        print "Adding STE cut for set: " + str(S)
                        self.vrp.add_ste_cut(S)
                        self.cut_count +=1
                        #cleanup += 1
                    t_add_cut0 = time.time()
                    pop_indices=[]
                    self.open_nodes.append(node)
                    for i,node2 in enumerate(self.open_nodes):
                        if self.vrp.update_duals:
                            node2.dual_values_model += [0.0]*addedCuts
                        node2.updated_lp_relaxation = 0
                        #node2.solve_lp_relaxation()
                        if not node2.feasible or node2.lower_bound >= self.ub:
                            pop_indices.append(i)
                    while (len(pop_indices)>0):
                        self.open_nodes.pop(pop_indices.pop(-1))
                    self.add_cut_time += time.time() - t_add_cut0 
                    continue
                

                
                #segment if no cut was added and nodes were not split
                if node.lower_bound < self.ub-0.99:
                    self.sol = sol
                    self.solution_path = self.vrp.solToPaths(sol)
                    print "Integer feasible solution found, objective: %f" %node.lower_bound
                    self.ub = node.lower_bound
                    self.solution_node = node
                pop_indices=[]
                #pruning of tree
                for i,node2 in enumerate(self.open_nodes):
                    #node2.solve_lp_relaxation()
                    if not node2.feasible or node2.lower_bound >= self.ub-0.99:
                        pop_indices.append(i)
                while (len(pop_indices)>0):
                    self.open_nodes.pop(pop_indices.pop(-1))
            split_points = self.vrp.findSplitPoints(self.sol)#splitPoints pruft, ob die Wege in 
            #                                #S moglich sind und returned tuple hinzuzufugender Punkte der unzulassigen Wege
            #print split_points
            #time.sleep(1)
            if len(split_points)>0:
                print "Splitting nodes"
                #print split_points
                #time.sleep(10)
                self.refinement_count += 1
                #for i,node2 in enumerate(self.open_nodes):
                #    if self.vrp.update_duals:
                #        node2.dual_values_model += [0.0]*addedCuts
                actually_split = 0
                for i,t,b in split_points:
                    actually_split += self.vrp.nodes[i].split_node_both(t,b)
                if actually_split == 0:
                    print "Error no node split"
                self.ub = 0
                self.root = Tree_node(self,[])
                self.open_nodes = [self.root]
            else:
                time_feasible = 1



def process_adj_matrix(adj_matrix,adj_matrix_tr,TWs,old_adj_matrix,deep_check=0):
    for i in adj_matrix:
        toRemove = []
        for j in adj_matrix[i]:
            
            if TWs[i][0]+adj_matrix[i][j]>TWs[j][1]:
                toRemove.append(j)
        for j in toRemove:
            adj_matrix[i].pop(j)
    oldArcAmount = 0
    for i in adj_matrix:
        oldArcAmount += len(adj_matrix[i])
    print "Total number of arcs before second preprocessing step: %d" % oldArcAmount
    #return
    for i in adj_matrix:
        toRemove = []
        for j in adj_matrix[i]:
            for k in adj_matrix[i]:
                if j in adj_matrix[k].keys():
                    time_delta_ik,bat_delta_ik = arc_lengths_pre(i,k,TWs[i][1],bat_intervals[i][0],TWs[k],bat_intervals[k])
                    if TWs[i][1]+time_delta_ik+adj_matrix[k][j]+(battery_capacity-(bat_intervals[i][0]+bat_delta_ik))/load_fac <TWs[j][0]:
                        toRemove.append(j)
                        #print "Removing arc (%d,%d) because %d can be visited in between at no cost" % (i,j,k)
                        #print "Previous time: %f" % adj_matrix[i][j]
                        #print "Extra travel time: %f" % ((adj_matrix[i][k]+adj_matrix[k][j]-adj_matrix[i][j])*(1+1/load_fac))
                        break
        for j in toRemove:
            adj_matrix[i].pop(j)

def write_line(filename,filetype,instance_name,data):
    file = open(filename, "a")
    if filetype == "python":
        
        file.write('"'+instance_name + '"'+":[%.2f,%.2f,%d,%d,%.1f,%.1f,%d,%d,%d,%d,%d,%d],\n" %data)
    
    if filetype == "gnuplot":
        file.write(instance_name + ' '+" %.2E %.2E %d %d %.1f %.1f %d %d %d %d %d %d\n" %data)
    file.close()
        

dynamic_discovery = 0
startHeurIter = 0
#1:[[0,18,3,47,5,11,29,48,13,51],[0,32,14,10,26,42,7,36,51],[0,35,50,15,33,45,6,2,51]]
#2:[[0,2,41,21,9,3,11,12,51],[0,5,19,39,36,34,50,46,51],[0,10,13,15,23,47,28,37,51]]
#7: [[0,1,32,30,40,5,46,36,39,51],[0,20,4,22,12,50,15,11,51],[0,27,8,43,17,6,42,7,29,51]]
instance_names = {
#"SimCologne_C_50_twLength_30_i_1_equalProfits.txt":-22,
#"SimCologne_C_50_twLength_30_i_2_equalProfits.txt":-21,
#"SimCologne_C_50_twLength_30_i_3_equalProfits.txt":-26,
#"SimCologne_C_50_twLength_30_i_4_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_30_i_5_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_30_i_6_equalProfits.txt":24,
#"SimCologne_C_50_twLength_30_i_7_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_30_i_8_equalProfits.txt":-26,
#"SimCologne_C_50_twLength_30_i_9_equalProfits.txt":-24,
#"SimCologne_C_50_twLength_30_i_10_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_30_i_11_equalProfits.txt":-22,
#"SimCologne_C_50_twLength_30_i_12_equalProfits.txt":-22,
"SimCologne_C_50_twLength_30_i_13_equalProfits.txt":-24,
#"SimCologne_C_50_twLength_30_i_14_equalProfits.txt":-24,
#"SimCologne_C_50_twLength_30_i_15_equalProfits.txt":-21,             
                  }

if dynamic_discovery:
    saveFileName = "Results_BAT_dyn_disc_"
else:
    saveFileName = "Results_BAT_BNT_"
file = open(saveFileName, "w")
file.write("{")
file.close()
use_best_heuristic = 0
bat_unit = 2
load_fac = bat_unit*2.5
loading_speed = load_fac
service_time = 5

battery_capacity = bat_unit*120
speed = 60
time_horizon = 720

inst_num = 0
for instance_name in sorted(instance_names.keys()):
    inst_num += 1
    vert_num,TWs,adj_matrix,profits,depotDists = readData(instance_name,"BatVRP")
    depotDists = [0]+depotDists+[0]
    profits = [0]+profits+[0]
    vert_num += 1
    TWs.append([TWs[0][0],TWs[0][1]])
    TWs[0][1]=0
    adj_matrix[vert_num-1] = {}
    for i in range(1,vert_num-1):
        adj_matrix[i][vert_num-1]=adj_matrix[0][i]-service_time
    #old_adj_matrix = {i:{j:val for j,val in adj_matrix[i].iteritems()} for i in adj_matrix}
    
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
    for i in adj_matrix:
        oldArcAmount+=len(adj_matrix[i])
    bat_intervals = [[bat_unit*depotDists[i],battery_capacity-bat_unit*depotDists[i]] for i in adj_matrix]
    bat_intervals[0]=[battery_capacity,battery_capacity]
    battery_matrix = {i : { j : -bat_unit*(depotDists[i]+depotDists[j]) for j in adj_matrix} for i in adj_matrix}
    print "Total number of arcs before preprocessing: %d" % oldArcAmount
    process_adj_matrix(adj_matrix,adj_matrix_tr,TWs,old_adj_matrix)
    

    #g=build_precedence_graph(adj_matrix,adj_matrix_tr,TWs,old_adj_matrix)
    processedArcAmount=0
    for i in adj_matrix:
        processedArcAmount+=len(adj_matrix[i])
    print "Total number of arcs after preprocessing: %d" % processedArcAmount
    time.sleep(5)
    print "Starting branch and bound process"
    
    #time.sleep(30)
    #blub-8
    cycles = []
    #nodes = [[i,TWs[i][0],TWs[i][1]] for i in range(vert_num)]
    for i in adj_matrix:
        for j in adj_matrix[i]:
            if i in adj_matrix[j].keys() and i<j:
                cycles.append( (i,j) )
    #break
    print str(cycles)

    vrp = VRP(TWs,bat_intervals,adj_matrix,profits,battery_matrix,0,len(TWs)-1,load_fac,battery_capacity)
    #break
    #vrp = vrp(TWs,adj_matrix,adj_matrix,0,vert_num-1)
    #vrp_ub = vrp_ub(TWs,adj_matrix,adj_matrix,0,vert_num-1)
    #vrp.precedence_graph = g
    #vrp_ub.precedence_graph = g
    vrp.old_adj_matrix=old_adj_matrix
    #vrp.adj_matrix_tr = adj_matrix_tr
    vrp.update_duals=0
    vrp.adapt_model=0
    vrp.testNodeGraph()
    #time.sleep(3)
    for i in vrp.nodes:
        if i.name == 0:
            continue
        i.split_node_both(i.tw_interval[0],i.bat_interval[0],0)
        i.split_node_both(i.tw_interval[1],i.bat_interval[0],0)
    vrp.update_duals=1
    vrp.adapt_model=1
    vrp.create_model()
    tree = Tree(vrp,0)
    tree.service_time = service_time
    tree.use_best_heuristic = use_best_heuristic
    tree.heuristic_ub = instance_names[instance_name]+1
    tree.add_all_split_points = 0
    t0=time.time()
    infeasible_paths = []
    for i in vrp.adj_matrix:
        timeI = TWs[i][0]
        batI = bat_intervals[i][1]
        for k in vrp.adj_matrix[i]:
            time_delta_ik,bat_delta_ik,lt = vrp.calc_deltas(i,k,batI,timeI)
            for j in vrp.adj_matrix[k]:
                #print (i,k,j)
                time_delta_kj,bat_delta_kj,lt = vrp.calc_deltas(k,j,batI+bat_delta_ik,timeI+time_delta_ik)
                
                if timeI+time_delta_ik+time_delta_kj>TWs[j][1]+0.0001 or batI+bat_delta_ik+bat_delta_kj< bat_intervals[j][0]-0.0001:
                    infeasible_paths.append([i,k,j])
    for P in infeasible_paths:
        vrp.add_infeasible_path(P)
    #break
    #vrp.model.solve()
    #print(vrp.model.solution.get_objective_value())
    #blub-8
    vrp.testNodeGraph()
    #break
    for i in adj_matrix:
        for j in adj_matrix:
            if i!=j and i not in [0,vert_num-1] and j not in [0,vert_num-1]:
                if (TWs[i][0]-depotDists[i] +0.0001 >= TWs[j][0]-depotDists[j] and 
                    TWs[i][1]+depotDists[i] <= TWs[j][1]+depotDists[j]+0.0001 ):
                    vrp.add_priority_cut(i,j)
                    #print "%d dominates %d" %(i,j)
    #break

    for cycle in cycles:
        vrp.add_ste_cut(cycle)
    t0 = time.time()
    tree.branch_and_refine()
    #tree.dynamic_discovery()
    t0 = time.time()-t0
    
    #break
    t1=time.time()
    tree.open_nodes.append(Tree_node(tree,[]))
    tree.open_nodes[0].solve_lp_relaxation()
    tree.lb = -50
    tree.ub = 0
    tree.branch_and_refine()
    #tree.dynamic_discovery()
    print "Original time: " + str(t0)
    print "CHeated time: " + str(time.time()-t1)
    break
    print ("___________________________________________________________\n")
    print "\n\nTotal time: %f\n\n" %(t1-t0)
    print "Average lp time: %f" %(sum(tree.lp_times)/len(tree.lp_times))
    print "Total lp time (includes part of the time of adding cuts/splitting): %f" % sum(tree.lp_times)
    print "Average simplex iterations: %f" %(sum(tree.simp_iteras)/len(tree.simp_iteras))
    print "Total number of nodes visited: %d" % tree.count
    print "Node selection time: %f" % tree.node_selection_time
    print "Time spend on finding and adding ste cuts: %f" % tree.add_cut_time
    print "Time spend on splitting nodes: %f" % tree.split_time
    old_instance_name = instance_name
    number_of_nodes_in_graph = 0
    number_of_arcs_in_graph = 0
    for i in tree.vrp.nodes:
        number_of_nodes_in_graph += len(i.interval_nodes)
    for key,extended_arclist in tree.vrp.arc_dict.iteritems():
        number_of_arcs_in_graph += len(extended_arclist)
    lineData =( sum(tree.lp_times),
               (sum(tree.lp_times)/len(tree.lp_times)),tree.count,tree.root_count,tree.ub,tree.lb,
               (sum(tree.simp_iteras)/len(tree.simp_iteras)),tree.cut_count,
               tree.refinement_count,tree.node_count,number_of_nodes_in_graph,number_of_arcs_in_graph)
    if resultfiletype == "python":
        write_line(saveFileName,resultfiletype,instance_name,lineData)
    else:
        write_line(saveFileName,resultfiletype,"%d"%inst_num,lineData)
    #file = open(saveFileName, "a")
    #file.write('"'+instance_name + '"'+":[%.2f,%.2f,%d,%d,%.1f,%.1f,%d,%d,%d,%d,%d,%d],\n" %( sum(tree.lp_times),
    #           (sum(tree.lp_times)/len(tree.lp_times)),tree.count,tree.root_count,tree.ub,tree.lb,
    #           (sum(tree.simp_iteras)/len(tree.simp_iteras)),tree.cut_count,
    #           tree.refinement_count,tree.node_count,number_of_nodes_in_graph,number_of_arcs_in_graph)
    #write_line(saveFileName,"python",instance_name,lineData))
    #file.close()
if 0:
    file = open(saveFileName, "a")
    file.write("}")
    file.close()