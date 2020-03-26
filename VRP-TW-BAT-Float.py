from datetime import datetime
import cplex
import re
import math
import time
#import sys
import functools

def register_time(func):
    @functools.wraps(func)
    def wrapper_decorator(*args, **kwargs):
        t0 = time.time()
        value = func(*args, **kwargs)
        if func.__name__ in function_times:
            function_times[func.__name__] += time.time()-t0
        else:
            function_times[func.__name__] = time.time()-t0
        return value
    return wrapper_decorator

def register_calls(func):
    @functools.wraps(func)
    def wrapper_decorator(*args, **kwargs):
        value = func(*args, **kwargs)
        if func.__name__ in function_times:
            function_calls[func.__name__] += 1
        else:
            function_calls[func.__name__] = 1
        return value
    return wrapper_decorator

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

def readData(file_name,directory_name ="AFG"):
    
    print("reading "+file_name)

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
    if '' in depotDists:
        depotDists.remove('')
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
                adj_matrix[i][j]= service_time+float(depotDists[i-1])+float(depotDists[j-1])
    
    TWs=[[0.0,time_horizon]]
    for i in range(vertNum):
        TWs.append([float(TWlbs[i])+service_time,float(TWubs[i])+service_time])
    for i in range(len(profits)):
        profits[i]=float(profits[i])
    vertNum+=1
    depotDists = [0]+depotDists+[0]
    profits = [0]+profits+[0]
    vertNum += 1
    TWs.append([TWs[0][0],TWs[0][1]])
    TWs[0][1]=0.0
    adj_matrix[vertNum-1] = {}
    for i in range(1,vertNum-1):
        adj_matrix[i][vertNum-1]=adj_matrix[0][i]-service_time
    #full_adj_matrix = {i:{j:val for j,val in adj_matrix[i].items()} for i in adj_matrix}
    
    for i in  range(vertNum):
        if 0 in adj_matrix[i]:
            adj_matrix[i][vert_num-1]=adj_matrix[i].pop(0)
        if i in adj_matrix[i]:
            adj_matrix[i].pop(i)
    return vertNum,TWs,adj_matrix,profits,depotDists


def arc_length(time,battery_level,head_tw,head_bw,tr_time,bat_delta1,bat_delta2,loading_speed,battery_capacity):
    tr_bat = bat_delta1 + bat_delta2
    if  bat_delta1+battery_level < -0.001:
        print( "Error battery level too low to return to depot")
    forced_load_time = max(head_bw[0]-(battery_level+tr_bat),0)/loading_speed
    forced_wait_time = max(head_tw[0]-(time+tr_time),0)
    load_time=round(max(forced_load_time,forced_wait_time)+0.00001,3)
    time_delta = load_time+tr_time
    bat_delta = min(battery_capacity,battery_level+bat_delta1+load_time*loading_speed)-battery_level+bat_delta2
    return time_delta,bat_delta,load_time

def ub_arc_length(time,battery_level,head_tw,head_bw,tr_time,bat_delta1,bat_delta2,loading_speed,battery_capacity):
    tr_bat = bat_delta1 + bat_delta2
    if  bat_delta1+battery_level < -0.001:
        print( "Error battery level too low to return to depot")
    forced_load_time = max(head_bw[0]-(battery_level+tr_bat),0)/loading_speed
    forced_wait_time = max(head_tw[0]-(time+tr_time),0)
    load_time=max(forced_load_time,forced_wait_time)
    time_delta = load_time+tr_time
    bat_delta = min(battery_capacity,battery_level+bat_delta1+load_time*loading_speed)-battery_level+bat_delta2
    return time_delta,bat_delta,load_time



class flowGraphNode():
    def __init__(self):
        self.inflow = 0.0
        self.predecessors = {}
        self.successors = {}
    def maxSucc(self):
        maxi = 0.0
        #maxS = -1
        for s,val in self.successors.items():
            if val > maxi:
                maxi = val
                maxS = s
        return maxS
            

class VRP():
    def __init__(self,TWs,bat_intervals,full_adj_matrix,adj_matrix,
                 profits,battery_matrix,depot,goal,loading_speed,battery_capacity,start_params={}):

        self.set_start_params(start_params)
        self.loading_speed = loading_speed
        self.battery_capacity = battery_capacity
        
        
        self.depot = depot
        self.goal = goal
        self.TWs = TWs
        self.bat_intervals = bat_intervals
        self.n = len(TWs)
        self.indices = range(self.n)
        self.battery_matrix = battery_matrix
        self.full_adj_matrix = full_adj_matrix
        self.adj_matrix = adj_matrix
        self.profits = profits
        self.battery_from_depot= {i:battery_matrix[0][i] for i in battery_matrix}
        
        self.break_path = 1
        
        self.nodes = [VRP_node(self,i,TWs[i][0],TWs[i][1],bat_intervals[i][0],bat_intervals[i][1]) for i in self.indices]
        self.ub_nodes = [VRP_node(self,i,TWs[i][0],TWs[i][1],bat_intervals[i][0],bat_intervals[i][1]) for i in self.indices]
        self.adapt_model = 0
        
        self.arc_dict = {}
        self.o_arc_dict = {}
        for i in adj_matrix:
            for j in list(adj_matrix[i].keys()):
                time_delta,bat_delta,load_time = arc_length(self.nodes[i].interval_nodes[0].tw_interval[0],
                                                  self.nodes[i].interval_nodes[0].battery_nodes[-1].bat_interval[1],
                                                  self.nodes[j].tw_interval,
                                                  self.nodes[j].bat_interval,
                                                  self.adj_matrix[i][j],
                                                  self.battery_from_depot[i],
                                                  self.battery_from_depot[j],
                                                  self.loading_speed,
                                                  self.battery_capacity)
                
                if ((not self.nodes[j].interval_nodes[0].is_reachable(self.nodes[i].interval_nodes[0],time_delta) )
                    or   (not self.nodes[j].interval_nodes[0].battery_nodes[0].is_reachable(self.nodes[i].interval_nodes[0].battery_nodes[-1],bat_delta ))):
                    adj_matrix[i].pop(j)
                else:
                    self.arc_dict[(i,j)]=[]
                    self.o_arc_dict[(i,j)]=[]
                    Arc(self.nodes[j].interval_nodes[0].battery_nodes[-1],
                            self.nodes[i].interval_nodes[0].battery_nodes[-1])
                    #if not arc.h
        for i in self.nodes:
            if i.name == 0:
                continue
            i.split_node_both(i.tw_interval[0],i.bat_interval[0],0)
            i.split_node_both(i.tw_interval[1],i.bat_interval[0],0)
        #"""
        for i in self.ub_nodes:
            if i.name ==0:
                continue
            i.split_ub_node(i.tw_interval[1],i.bat_interval[1])
            i.split_ub_node(i.tw_interval[0],i.bat_interval[1])
        #"""
        self.adj_matrix_tr = {i:{} for i in self.adj_matrix}
        for i in self.adj_matrix:
            for j in self.adj_matrix[i]:
                self.adj_matrix_tr[j][i]=self.adj_matrix[i][j]
        
        self.adapt_model = 1
        if self.remove_cycles_at_start:
            self.find_cycles()

        if self.remove_infeasible_paths_at_start:
            self.find_infeasible_paths()

        if self.add_priority_cuts_at_start:
            self.find_priority_cuts()
    def set_start_params(self,start_params):
        self.remove_cycles_at_start = start_params["remove_cycles_at_start"]
        self.remove_infeasible_paths_at_start = start_params["remove_infeasible_paths_at_start"]
        self.add_priority_cuts_at_start = start_params["add_priority_cuts_at_start"]
        self.multivisit_model = start_params["multivisit_model"]
        self.update_duals = start_params["update_duals"]
        self.vehicle_amount = start_params["vehicle_amount"]
        self.refine_heuristic = start_params["refine_heuristic"]
    def solve_heuristic(self):
        self.create_ub_graph()
        self.create_ub_model()
        self.ub_model.solve()
        return self.ub_model.solution.get_objective_value()

    @register_time
    @register_calls
    def create_ub_graph(self):
        #clear old graph
        for i in self.full_adj_matrix:
            for j in self.full_adj_matrix[i]:
                self.o_arc_dict[(i,j)]=[]
        for vrp_node in self.ub_nodes:
                for interval_node in vrp_node.interval_nodes:
                    for bat_node in interval_node.battery_nodes:
                        bat_node.ingoing_o_arcs = []
                        bat_node.outgoing_o_arcs = []
        count=0
        for vrp_node in self.ub_nodes:
            for vrp_head_name in self.full_adj_matrix[vrp_node.name]:
                vrp_head = self.ub_nodes[vrp_head_name]
                for interval_node in vrp_node.interval_nodes:
                    for bat_node in interval_node.battery_nodes:
                        time_delta,bat_delta,load_time = arc_length(interval_node.tw_interval[1],
                                                  bat_node.bat_interval[0],
                                                  self.ub_nodes[vrp_head.name].tw_interval,
                                                  self.ub_nodes[vrp_head.name].bat_interval,
                                                  self.full_adj_matrix[vrp_node.name][vrp_head.name],
                                                  self.battery_from_depot[vrp_node.name],
                                                  self.battery_from_depot[vrp_head.name],
                                                  self.loading_speed,
                                                  self.battery_capacity)
                        
                        is_reachable,interval_node_head = vrp_head.find_lowest_guaranteed_reachable(interval_node,time_delta)
                        if is_reachable:
                            additional_load_time = interval_node_head.tw_interval[1] - (interval_node.tw_interval[1]+time_delta)
                            bat_delta = min(battery_capacity,bat_node.bat_interval[0]+self.battery_from_depot[vrp_node.name]+(load_time+
                                        additional_load_time)*loading_speed)-bat_node.bat_interval[0]+self.battery_from_depot[vrp_head.name]
                            is_reachable,bat_node_head = interval_node_head.find_maximal_guaranteed_reachable_battery_node(bat_node,bat_delta)
                        if is_reachable:
                            count = count +1
                            
                            #print (bat_node_head.name,bat_node.name)
                            Arc(bat_node_head,bat_node,1)

        #print count
    @register_time
    @register_calls
    def create_ub_model(self,var_type='I'):
        self.ub_const_num = 0
        self.ub_const_name2idx={}
        self.ub_idx2name = {}
        self.ub_name2idx = {}
        self.ub_x_names = []
        self.ub_y_names = []
        
        self.ub_model = cplex.Cplex()
        model = self.ub_model
        model.set_results_stream(None)
        model.set_log_stream(None)
        model.parameters.advance.set(1)
        model.set_warning_stream(None)
        x_names = ["x_%d_%d" %(i,j) for i in self.indices for j in self.full_adj_matrix[i]]
        if self.multivisit_model:
            x_obj = [0 for i in self.indices for j in self.full_adj_matrix[i]]
            self.ub_z_names = []
            z_names = ["z_%d" %(i) for i in self.indices]
            z_obj = [-self.profits[i] for i in self.indices]
        else:
            x_obj = [-self.profits[j] for i in self.indices for j in self.full_adj_matrix[i]]
        y_names = ["y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (arc.tail.name,arc.head.name,
                                           arc.tail.id_ub[0],arc.tail.id_ub[1],arc.head.id_ub[0],arc.head.id_ub[1])
                        for key,arc_list in self.o_arc_dict.items() for arc in arc_list]

        
        self.ub_add_variables(x_names,[var_type]*len(x_names),x_obj,[0.0]*len(x_names),[1.0]*len(x_names),'x')
        self.ub_x_num=len(x_names)
        if self.multivisit_model:
            self.ub_add_variables(z_names,['C']*len(z_names),z_obj,[0.0]*len(z_names),[1.0]*len(z_names),'z')
        self.ub_add_variables(y_names,['C']*len(y_names),[0.0]*len(y_names),[0.0]*len(y_names),[1.0]*len(y_names),'y')
        allvars = []
        allrhs = []
        allsenses = []
        all_names = []
        
        if not self.multivisit_model:
            for node in self.ub_nodes:
                if node.name!=self.goal and node.name != self.depot:
                    thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.full_adj_matrix[node.name]]
                    thecoefs = [1]*len(thevars)
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("L")
                    allrhs.append(1.0)
                    all_names.append("visit_max_once_%d" % node.name)
                if node.name == self.depot:
                    thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.full_adj_matrix[node.name]]
                    thecoefs = [1]*len(thevars)
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("L")
                    allrhs.append(self.vehicle_amount)
                    all_names.append("visit_max_once_%d" % node.name)
        else:
            for node in self.ub_nodes:
                if node.name!=self.goal and node.name != self.depot:
                    thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.full_adj_matrix[node.name]]+["z_%d" % node.name]
                    thecoefs = [-1 for node2 in self.full_adj_matrix[node.name]]+[1]
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("L")
                    allrhs.append(0.0)
                    all_names.append("calc_profits_%d" % node.name)
                if node.name == self.depot:
                    thevars = ["x_%d_%d" %(node.name,node2) for node2 in self.full_adj_matrix[node.name]]
                    thecoefs = [1]*len(thevars)
                    allvars.append(cplex.SparsePair(thevars,thecoefs))
                    allsenses.append("L")
                    allrhs.append(self.vehicle_amount)
                    all_names.append("visit_max_once_%d" % node.name)
                    
        
        for node in self.ub_nodes:
            for interval_node in node.interval_nodes:
                for bat_node in interval_node.battery_nodes:
                    if node.name != self.goal and node.name !=self.depot:
                        thevars = ["y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (arc.tail.name,arc.head.name,
                                               arc.tail.id_ub[0],arc.tail.id_ub[1],arc.head.id_ub[0],arc.head.id_ub[1])
                                    for arc in bat_node.outgoing_o_arcs]
                        
                        thecoefs = [-1]*len(bat_node.outgoing_o_arcs)
                        
                            
                        #allvars.append(cplex.SparsePair(thevars,thecoefs))
                        #allsenses.append("E")
                        #allrhs.append(0.0)
                        #all_names.append("goout_%d_%d" %(node.name,interval_node.id_ub))
                        
                        thevars += ["y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (arc.tail.name,arc.head.name,
                                               arc.tail.id_ub[0],arc.tail.id_ub[1],arc.head.id_ub[0],arc.head.id_ub[1])
                                    for arc in bat_node.ingoing_o_arcs]
                        
                        thecoefs += [1]*len(bat_node.ingoing_o_arcs)
        
                            
                        allvars.append(cplex.SparsePair(thevars,thecoefs))
                        allsenses.append("E")
                        allrhs.append(0.0)
                        all_names.append("flow_%d_%.3f_%.3f" %(node.name,bat_node.id_ub[0],bat_node.id_ub[1]))
        
        for i,j in self.o_arc_dict:
            thevars = ["y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (arc.tail.name,arc.head.name,
                                           arc.tail.id_ub[0],arc.tail.id_ub[1],arc.head.id_ub[0],arc.head.id_ub[1])
                       for arc in self.o_arc_dict[i,j]]
            thecoefs = [1]*len(thevars)
            thevars += ["x_%d_%d" %(i,j)]
            thecoefs += [-1]
            allvars.append(cplex.SparsePair(thevars,thecoefs))
            allsenses.append("E")
            allrhs.append(0.0)
            all_names.append("arcuse_%d_%d" %(i,j))
        
        self.ub_add_constraints(all_names,allvars,allsenses,allrhs)
    def create_model(self,var_type='C'):
        self.const_num = 0
        self.const_name2idx={}
        self.idx2name = {}
        self.name2idx = {}
        self.x_names = []
        self.y_names = []
        
        self.model = cplex.Cplex()
        model = self.model
        #model.set_results_stream(None)
        #model.set_log_stream(None)
        model.parameters.advance.set(1)
        model.set_warning_stream(None)
        x_names = ["x_%d_%d" %(i,j) for i in self.indices for j in self.adj_matrix[i]]
        if self.multivisit_model:
            x_obj = [0 for i in self.indices for j in self.adj_matrix[i]]
            self.z_names = []
            z_names = ["z_%d" %(i) for i in self.indices]
            z_obj = [-self.profits[i] for i in self.indices]
        else:
            x_obj = [-self.profits[j] for i in self.indices for j in self.adj_matrix[i]]
        y_names = ["y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (arc.tail.name,arc.head.name,
                                           arc.tail.id[0],arc.tail.id[1],arc.head.id[0],arc.head.id[1])
                        for key,arc_list in self.arc_dict.items() for arc in arc_list]
        
        self.add_variables(x_names,[var_type]*len(x_names),x_obj,[0.0]*len(x_names),[1.0]*len(x_names),'x')
        self.x_num=len(x_names)
        if self.multivisit_model:
            self.add_variables(z_names,['C']*len(z_names),z_obj,[0.0]*len(z_names),[1.0]*len(z_names),'z')
        self.add_variables(y_names,['C']*len(y_names),[0.0]*len(y_names),[0.0]*len(y_names),[1.0]*len(y_names),'y')
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
                        thevars = ["y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (arc.tail.name,arc.head.name,
                                               arc.tail.id[0],arc.tail.id[1],arc.head.id[0],arc.head.id[1])
                                    for arc in bat_node.outgoing_arcs]
                        
                        thecoefs = [-1]*len(bat_node.outgoing_arcs)
                        
                            
                        #allvars.append(cplex.SparsePair(thevars,thecoefs))
                        #allsenses.append("E")
                        #allrhs.append(0.0)
                        #all_names.append("goout_%d_%d" %(node.name,interval_node.id))
                        
                        thevars += ["y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (arc.tail.name,arc.head.name,
                                               arc.tail.id[0],arc.tail.id[1],arc.head.id[0],arc.head.id[1])
                                    for arc in bat_node.ingoing_arcs]
                        
                        thecoefs += [1]*len(bat_node.ingoing_arcs)
        
                            
                        allvars.append(cplex.SparsePair(thevars,thecoefs))
                        allsenses.append("E")
                        allrhs.append(0.0)
                        all_names.append("flow_%d_%.3f_%.3f" %(node.name,bat_node.id[0],bat_node.id[1]))
        
        for i,j in self.arc_dict:
            thevars = ["y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (arc.tail.name,arc.head.name,
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
        if self.remove_cycles_at_start:
            self.remove_cycles()
        if self.remove_infeasible_paths_at_start:
            self.remove_infeasible_paths()
        if self.add_priority_cuts_at_start:
            self.add_priority_cuts()
        if var_type == 'C':
            self.model.set_problem_type(0)
            self.model.parameters.lpmethod.set(2)
    def find_infeasible_paths(self):
        self.infeasible_paths = []
        for i in self.adj_matrix:
            timeI = self.TWs[i][0]
            batI = self.bat_intervals[i][1]
            for k in self.adj_matrix[i]:
                time_delta_ik,bat_delta_ik,lt = arc_length(timeI,
                                                  batI,
                                                  self.nodes[k].tw_interval,
                                                  self.nodes[k].bat_interval,
                                                  self.adj_matrix[i][k],
                                                  self.battery_from_depot[i],
                                                  self.battery_from_depot[k],
                                                  self.loading_speed,
                                                  self.battery_capacity)
                for j in self.adj_matrix[k]:
                    time_delta_kj,bat_delta_kj,lt = arc_length(timeI+time_delta_ik,
                                                  batI+bat_delta_ik,
                                                  self.nodes[j].tw_interval,
                                                  self.nodes[j].bat_interval,
                                                  self.adj_matrix[k][j],
                                                  self.battery_from_depot[k],
                                                  self.battery_from_depot[j],
                                                  self.loading_speed,
                                                  self.battery_capacity)
                    if timeI+time_delta_ik+time_delta_kj>self.TWs[j][1]+0.0001 or batI+bat_delta_ik+bat_delta_kj< self.bat_intervals[j][0]-0.0001:
                        self.infeasible_paths.append([i,k,j])
    def remove_infeasible_paths(self):
        if self.remove_infeasible_paths_at_start:
            prev = []
            for P in self.infeasible_paths:
                sol = [[self.depot]+P+[self.goal]]
                #self.add_infeasible_path(P)
                split_points = self.findSplitPoints(sol)
                if split_points != prev:
                    for i,t,b in split_points:
                        self.nodes[i].split_node_both(t,b)
                prev = split_points
    def find_cycles(self):
        self.cycles = []
        for i in adj_matrix:
            for j in adj_matrix[i]:
                if i in adj_matrix[j].keys() and i<j:
                    self.cycles.append( (i,j) )
    def remove_cycles(self):
        if self.remove_cycles_at_start:
            for cycle in self.cycles:
                self.add_ste_cut(cycle)
    def find_priority_cuts(self):
        self.priority_cuts = []
        for i in adj_matrix:
            for j in adj_matrix:
                if i!=j and i not in [0,vert_num-1] and j not in [0,vert_num-1]:
                    if (TWs[i][0]-depotDists[i] +0.001 >= TWs[j][0]-depotDists[j] and 
                        TWs[i][1]+depotDists[i] <= TWs[j][1]+depotDists[j]+0.001 ):
                        self.priority_cuts.append((i,j))
    def add_priority_cuts(self):
        if self.add_priority_cuts_at_start:
            for i,j in self.priority_cuts:
                self.add_priority_cut(i,j)
    def add_constraints(self,all_names,allvars,allsenses,allrhs):
        old_inds=self.const_num
        self.model.linear_constraints.add(names=all_names,lin_expr = allvars, 
                                                     senses = allsenses, rhs = allrhs)
        self.const_name2idx.update({name:old_inds+j for j,name in enumerate(all_names)})
        self.const_num += len(all_names)
    def ub_add_constraints(self,all_names,allvars,allsenses,allrhs):
        old_inds=self.ub_const_num
        self.ub_model.linear_constraints.add(names=all_names,lin_expr = allvars, 
                                                     senses = allsenses, rhs = allrhs)
        self.ub_const_name2idx.update({name:old_inds+j for j,name in enumerate(all_names)})
        self.ub_const_num += len(all_names)
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
    def ub_add_variables(self,all_names,all_types,all_obj,all_lb,all_ub,var_type):
        old_inds=self.ub_model.variables.get_num()
        self.ub_model.variables.add(names = all_names,
                            types=all_types,obj=all_obj,
                            lb = all_lb,ub = all_ub)
        if var_type=='x':
            self.ub_x_names += all_names
        if var_type=='y':
            self.ub_y_names += all_names
        self.ub_name2idx.update({name:old_inds+j for j,name in enumerate(all_names)})
        self.ub_idx2name.update({old_inds+j:name for j,name in enumerate(all_names)})
        self.ub_var_num = len(self.ub_name2idx)
    def change_variable_name(self,old_var_name,new_var_name):
        self.model.variables.set_names(old_var_name,new_var_name)
        self.idx2name[self.name2idx[old_var_name]] = new_var_name
        self.name2idx[new_var_name] = self.name2idx.pop(old_var_name)
        if new_var_name[0] == 'y':
            self.y_names.append(new_var_name)
            self.y_names.remove(old_var_name)
        else:
            print ("Changing names of non y-variables is currently not supported :(")
    @register_time
    @register_calls
    def solve_model2(self,branches=[]):
        self.model.solve()
    @register_time
    @register_calls
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
                    if j in self.adj_matrix[i]:
                        arc_list.append("x_%d_%d" % (i,j))
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
    def findArcCorrection(self,path,pathTB):
        arc_correction_sp = []
        for i in range(0,len(pathTB)):
            time_delta,bat_delta,load_time = arc_length(path[i+1][1],
                                                  path[i+1][2],
                                                  self.nodes[path[i+2][0]].tw_interval,
                                                  self.nodes[path[i+2][0]].bat_interval,
                                                  self.full_adj_matrix[path[i+1][0]][path[i+2][0]],
                                                  self.battery_from_depot[path[i+1][0]],
                                                  self.battery_from_depot[path[i+2][0]],
                                                  self.loading_speed,
                                                  self.battery_capacity)
            if (path[i+2][0],path[i+1][1]+time_delta,path[i+1][2]+bat_delta) not in pathTB:
                arc_correction_sp.append((path[i+2][0],path[i+1][1]+time_delta,path[i+1][2]+bat_delta))
        return arc_correction_sp
    def findTimedBatteryPath(self,path):
        pathTimeBattery = []
        battery_level = battery_capacity
        t = 0.0
        for i in range(len(path)-1):
            time_delta,bat_delta,load_time = arc_length(t,
                                                  battery_level,
                                                  self.nodes[path[i+1][0]].tw_interval,
                                                  self.nodes[path[i+1][0]].bat_interval,
                                                  self.full_adj_matrix[path[i][0]][path[i+1][0]],
                                                  self.battery_from_depot[path[i][0]],
                                                  self.battery_from_depot[path[i+1][0]],
                                                  self.loading_speed,
                                                  self.battery_capacity)
            
            battery_level += bat_delta
            t += time_delta
            if t <= self.nodes[path[i+1][0]].tw_interval[1]+0.005 and battery_level >= self.nodes[path[i+1][0]].bat_interval[0]-0.005:
                pathTimeBattery.append((path[i+1][0],t,battery_level))
            else:
                return 0,pathTimeBattery
        return 1,pathTimeBattery
    def findSplitPoints(self,sol):#heuristic for additional split points 
        split_point_list = []
        for pathSeed in sol:
            for j in range(len(pathSeed)-2,0,-1):
                path = [pathSeed[0]]+pathSeed[j:]
                pathFeasible,pathTB = self.findTimedBatteryPath(path)
                if not pathFeasible:
                    arc_corrections = self.findArcCorrection(path,pathTB)
                    if len(split_point_list) == 0:
                        print(f"Destroying path of length {len(pathTB)}")
                        #split_point_list += arc_corrections+pathTB
                        split_point_list += pathTB
                    else:
                        if split_point_list[-1][0]!=pathTB[-1][0]:
                            #split_point_list += arc_corrections+pathTB
                            print(f"Destroying path of length {len(pathTB)}")
                            split_point_list += pathTB
        return split_point_list
    def findSplitPointsToBreakCycle(self,cycle):
        split_points = []
        if cycle[0]==cycle[-1]:
            cycle.pop(-1)
        #print( cycle)
        for startIndex in range(len(cycle)):
            bat = self.nodes[cycle[startIndex]].bat_interval[1]
            time = self.nodes[cycle[startIndex]].tw_interval[0]
            curIndex = startIndex
            while time <= self.nodes[cycle[curIndex]].tw_interval[1]+0.005 and bat >= self.nodes[cycle[curIndex]].bat_interval[0]-0.005:
                split_points.append((cycle[curIndex],time,bat))
                nextIndex = (curIndex+1) % (len(cycle))
                #print cycle
                time_delta,bat_delta,load_time = arc_length(time,
                                                  bat,
                                                  self.nodes[cycle[nextIndex]].tw_interval,
                                                  self.nodes[cycle[nextIndex]].bat_interval,
                                                  self.full_adj_matrix[cycle[curIndex]][cycle[nextIndex]],
                                                  self.battery_from_depot[cycle[curIndex]],
                                                  self.battery_from_depot[cycle[nextIndex]],
                                                  self.loading_speed,
                                                  self.battery_capacity)
                bat += bat_delta
                time += time_delta
                curIndex = nextIndex
        return split_points
    def fracSolToPaths(self,yStringList,tol=0.001):
        depot=(self.nodes[0].name,int(round(1000*self.nodes[0].interval_nodes[0].tw_interval[0])),
                       int(round(1000*self.nodes[0].interval_nodes[0].battery_nodes[-1].bat_interval[1])))
        unusedNodeDict ={depot:
                flowGraphNode()
            }
        unusedNodeDict[depot].inflow = self.vehicle_amount
        arcDictFloat = {}
        
        for string,val in yStringList.items():
            arcDictFloat[tuple([float(s) for s in re.split("[_]",string)[1:]])] = val
        arcDict = {tuple([(int(arc[0]),int(round(1000*arc[2])),int(round(1000*arc[3]))),(int(arc[1]),
                               int(round(1000*arc[4])),int(round(1000*arc[5])))]):val for arc,val in arcDictFloat.items()}
        for arcL,val in arcDict.items():
            if not arcL[0] in unusedNodeDict:
                unusedNodeDict[arcL[0]] = flowGraphNode()
            if not arcL[1] in unusedNodeDict:
                unusedNodeDict[arcL[1]] = flowGraphNode()
            unusedNodeDict[arcL[0]].successors[(arcL[1])] = val
            unusedNodeDict[arcL[1]].predecessors[arcL[0]] = val
            unusedNodeDict[arcL[1]].inflow += val
        S = [[] for i in range(self.vehicle_amount)]
        for i in range(self.vehicle_amount):
            succ = depot
            S[i].append((succ[0],succ[1]/1000.0,succ[2]/1000.0))
            while len(unusedNodeDict[succ].successors) != 0:
                prev = succ
                succ = unusedNodeDict[succ].maxSucc()
                S[i].append((succ[0],succ[1]/1000.0,succ[2]/1000.0))
                if unusedNodeDict[prev].inflow <0.001:
                    for pred in unusedNodeDict[prev].predecessors:
                        unusedNodeDict[pred].successors.pop(prev)
                    unusedNodeDict.pop(prev)
                else:
                    unusedNodeDict[succ].inflow -= unusedNodeDict[prev].successors[succ]
                    unusedNodeDict[prev].successors.pop(succ)
                    unusedNodeDict[succ].predecessors.pop(prev)
        #print S
        #raw_input()
        return S
    def solToCyclesYopt(self,yStringList,tol=0.001):
        arcListFloat = []
        for string in yStringList:
            arcListFloat.append([float(s) for s in re.split("[_]",string)[1:]])
        oS = [[(int(arc[0]),arc[2],arc[3]),(int(arc[1]),arc[4],arc[5])] for arc in arcListFloat]
        S = [[(self.nodes[0].name,self.nodes[0].interval_nodes[0].tw_interval[0],self.nodes[0].interval_nodes[0].battery_nodes[-1].bat_interval[1]),
              ] for i in range(self.vehicle_amount)
            ]
        insertPos=[1 for i in range(self.vehicle_amount)]
        #tailPos=[1 for i in range(self.vehicle_amount)]
        appendedArcL = 0
        insertedCustomers = [set() for i in range(self.vehicle_amount)]
        while appendedArcL<len(oS):
            arcL = oS.pop(0)
            tail = arcL[0]
            head = arcL[-1]
            added = 0
            for i in range(self.vehicle_amount):
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
        for k in range(self.vehicle_amount):
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
                    print( "Error customer tracking predicted cycle, but no cycle found")
                    print( path)
                    time.sleep(20)
        return S,oS
    def testNodeGraph(self):
        for vrp_node in self.nodes:
            for interval_node in vrp_node.interval_nodes:
                for battery_node in interval_node.battery_nodes:
                    for arc in battery_node.ingoing_arcs+battery_node.outgoing_arcs:
                        time_delta,bat_delta,lt = arc_length(arc.tail.interval_node.tw_interval[0],
                                                          arc.tail.bat_interval[1],
                                                          arc.head.interval_node.vrp_node.tw_interval,
                                                          arc.head.interval_node.vrp_node.bat_interval,
                                                          self.adj_matrix[arc.tail.name][arc.head.name],
                                                          self.battery_from_depot[arc.tail.name],
                                                          self.battery_from_depot[arc.head.name],
                                                          self.loading_speed,
                                                          self.battery_capacity)
                        if abs(time_delta-arc.arc_time)> 0.1:
                            print ("Error: arcs time length is not correctly stored")
                        if abs(bat_delta-arc.arc_bat)> 0.1:
                            print ("Error: arcs bat length is not correctly stored")
                        if not arc.head.is_maximal_reachable_battery_node(arc.tail,bat_delta):
                            print ("Error: arc is not maximal reachable")
                        if not arc.head.interval_node.is_lowest_reachable(arc.tail.interval_node,time_delta):
                            print ("Tail interval: [%d,%d]" % tuple(arc.tail.interval_node.tw_interval))
                            print ("Head interval: [%d,%d]" % tuple(arc.head.interval_node.tw_interval))
                            print ("Heads complete TW: [%d,%d]" % tuple(arc.head.interval_node.vrp_node.tw_interval))
                            print ("Pure travel time: %d" % self.adj_matrix[arc.tail.name][arc.head.name])
                            print ("Travel + loading time: %d" % time_delta)
                            print ("Errror: arcs interval nodes are not lowest reachable")
    def testModel(self):
        return

            
                    
#class for arcs between interval nodes adding itself to the respective in- and outgoing lists
#and also to the arc dictionary of the vrp class
class Arc():
    def __init__(self,head,tail,is_for_ub=0):
        self.head = head
        
        self.tail = tail
        vrp = tail.interval_node.vrp_node.vrp
        if not is_for_ub:
            self.head.ingoing_arcs.append(self)
            self.tail.outgoing_arcs.append(self)
            self.arc_time,self.arc_bat,lt = arc_length(tail.interval_node.tw_interval[0],
                                                          tail.bat_interval[1],
                                                          head.interval_node.vrp_node.tw_interval,
                                                          head.interval_node.vrp_node.bat_interval,
                                                          vrp.adj_matrix[tail.name][head.name],
                                                          vrp.battery_from_depot[tail.name],
                                                          vrp.battery_from_depot[head.name],
                                                          vrp.loading_speed,
                                                          vrp.battery_capacity)
            self.tail.interval_node.vrp_node.vrp.arc_dict[tail.name,head.name].append(self)
        else:
            self.head.ingoing_o_arcs.append(self)
            self.tail.outgoing_o_arcs.append(self)
            self.arc_time,self.arc_bat,lt = arc_length(tail.interval_node.tw_interval[1],
                                                          tail.bat_interval[0],
                                                          head.interval_node.vrp_node.tw_interval,
                                                          head.interval_node.vrp_node.bat_interval,
                                                          vrp.full_adj_matrix[tail.name][head.name],
                                                          vrp.battery_from_depot[tail.name],
                                                          vrp.battery_from_depot[head.name],
                                                          vrp.loading_speed,
                                                          vrp.battery_capacity)
            self.tail.interval_node.vrp_node.vrp.o_arc_dict[tail.name,head.name].append(self)


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
    def split_ub_node(self,split_point_tw,split_point_bat):
        if self.name == self.vrp.goal:
            return 1
        tw_idx,split_tw = self.find_tw_index_ub(split_point_tw)
        if split_tw:
            old_tw_idx = tw_idx+1
        else:
            old_tw_idx = tw_idx
        #print tw_idx
        #print self
        
        bat_idx,split_bat = self.find_bat_index_ub(split_point_bat,tw_idx)
        
        
        if split_bat:
            new_bat_idx = bat_idx+1
        else:
            new_bat_idx = bat_idx
        if split_bat+split_tw < 1:
            return 0
            print( "Error neither battery window nor time_window split")
            
        old_lb_tw = self.interval_nodes[tw_idx].tw_interval[0]
        old_ub_bat = self.interval_nodes[tw_idx].battery_nodes[bat_idx].bat_interval[1]
        
        if split_tw:
            self.interval_nodes.insert(tw_idx,Interval_node(self.name,[old_lb_tw,split_point_tw],self,self.interval_nodes[tw_idx].is_tw_lb(),0))
            for bat_node in self.interval_nodes[old_tw_idx].battery_nodes:
                self.interval_nodes[tw_idx].battery_nodes.append(Battery_node(self.name,
                                   list(bat_node.bat_interval),self.interval_nodes[tw_idx],bat_node.is_bat_lb(),bat_node.is_bat_ub()))
        if split_bat:
            self.interval_nodes[tw_idx].battery_nodes.insert(new_bat_idx,
                           Battery_node(self.name,[split_point_bat,old_ub_bat],self.interval_nodes[tw_idx],
                                        0,self.interval_nodes[tw_idx].battery_nodes[bat_idx].is_bat_ub()))
        if split_tw:
            self.interval_nodes[old_tw_idx].is_lb=0
            self.interval_nodes[old_tw_idx].tw_interval[0] = split_point_tw
        if split_bat:
            self.interval_nodes[tw_idx].battery_nodes[bat_idx].is_ub = 0
            self.interval_nodes[tw_idx].battery_nodes[bat_idx].bat_interval[1] = split_point_bat
    def split_node_both(self,split_point_tw,split_point_bat,dual_value_locations=[]):
        self.vrp.ub_nodes[self.name].split_ub_node(split_point_tw,split_point_bat)
        if self.name == self.vrp.goal:
            return 1
        tw_idx,split_tw = self.find_tw_index(split_point_tw)
        if split_tw:
            new_tw_idx = tw_idx+1
        else:
            new_tw_idx = tw_idx
        bat_idx,split_bat = self.find_bat_index(split_point_bat,tw_idx)
        
        
        if split_bat:
            old_bat_idx = bat_idx+1
        else:
            old_bat_idx= bat_idx
        if split_bat+split_tw < 1:
            return 0
            print( "Error neither battery window nor time_window split")
            
        old_ub_tw = self.interval_nodes[tw_idx].tw_interval[1]
        old_lb_bat = self.interval_nodes[tw_idx].battery_nodes[bat_idx].bat_interval[0]
        
        
        if split_tw:
            self.interval_nodes.insert(new_tw_idx,Interval_node(self.name,[split_point_tw,old_ub_tw],self,0,self.interval_nodes[tw_idx].is_tw_ub()))
            for bat_node in self.interval_nodes[tw_idx].battery_nodes:
                self.interval_nodes[new_tw_idx].battery_nodes.append(Battery_node(self.name,
                                   list(bat_node.bat_interval),self.interval_nodes[new_tw_idx],bat_node.is_bat_lb(),bat_node.is_bat_ub()))
        if split_bat:
            self.interval_nodes[new_tw_idx].battery_nodes.insert(bat_idx,
                           Battery_node(self.name,[old_lb_bat,split_point_bat],self.interval_nodes[new_tw_idx],
                                        self.interval_nodes[new_tw_idx].battery_nodes[bat_idx].is_bat_lb(),0))
        if split_tw:
            self.interval_nodes[tw_idx].is_ub=0
            self.interval_nodes[tw_idx].tw_interval[1] = split_point_tw
        if split_bat:
            self.interval_nodes[new_tw_idx].battery_nodes[old_bat_idx].is_lb = 0
            self.interval_nodes[new_tw_idx].battery_nodes[old_bat_idx].bat_interval[0] = split_point_bat
        
        node_counter = 0
        bat_idx_reached = 0
        if self.vrp.adapt_model:
            if self.name!=self.vrp.goal:
                if split_tw:
                    for bat_node in self.interval_nodes[new_tw_idx].battery_nodes:
                        names = ["flow_%d_%.3f_%.3f" %(self.name,bat_node.id[0],bat_node.id[1])]
                        lin_expr=[cplex.SparsePair([],[] )]
                        self.vrp.add_constraints(names,lin_expr,["E"],[0.0])
                        if self.vrp.update_duals:
                            id1 = self.interval_nodes[tw_idx].battery_nodes[ node_counter-bat_idx_reached ].id[0]
                            id2 = self.interval_nodes[tw_idx].battery_nodes[ node_counter-bat_idx_reached ].id[1]
                            dual_value_locations += [self.vrp.const_name2idx["flow_%d_%.3f_%.3f"%(self.name,id1,id2)]]
                        if split_bat:
                            if node_counter == bat_idx:
                                bat_idx_reached = 1
                        node_counter += 1
                else:
                    if split_bat:
                        bat_node = self.interval_nodes[new_tw_idx].battery_nodes[bat_idx]
                        names = ["flow_%d_%.3f_%.3f" %(self.name,bat_node.id[0],bat_node.id[1])]
                        lin_expr=[cplex.SparsePair([],[] )]
                        self.vrp.add_constraints(names,lin_expr,["E"],[0.0])
                        if self.vrp.update_duals:
                            id1 = self.interval_nodes[tw_idx].battery_nodes[old_bat_idx].id[0]
                            id2 = self.interval_nodes[tw_idx].battery_nodes[old_bat_idx].id[1]
                            dual_value_locations += [self.vrp.const_name2idx["flow_%d_%.3f_%.3f"%(self.name,id1,id2)]]
                

        
        for idx in range(len(self.interval_nodes[tw_idx].battery_nodes)):
            pop_indices = []
            arc_index = 0
            for arc in self.interval_nodes[tw_idx].battery_nodes[idx].ingoing_arcs:
                head_is_lowest_reachable = arc.head.interval_node.is_lowest_reachable(arc.tail.interval_node,arc.arc_time)
                head_is_maximal_bat =  arc.head.is_maximal_reachable_battery_node(arc.tail,arc.arc_bat)
                if (split_tw and not head_is_lowest_reachable) or (
                            ((not split_tw) and not head_is_maximal_bat)):
                    if self.vrp.adapt_model:
                        old_var_name = "y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (arc.tail.name,arc.head.name,
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
                            print (arc.tail.name,arc.tail.bat_interval,arc.tail.interval_node.tw_interval)
                            print(arc.arc_time,arc.arc_bat)
                            print(self.name,new_interval_node.tw_interval)
                            print([bate_node.bat_interval for bate_node in new_interval_node.battery_nodes])
                            print ("Error no battery node found")
                            input()
                            print (self.vrp.nodes[old_head.name].bat_interval)
                    else:
                        print( "Error no interval node found")
                        #print "Tail interval" + str(arc.tail.interval_node.tw_interval)
                        #print "Shifted by: " + str(arc.arc_time)
                        #print "Original arc time: " + str(self.vrp.adj_matrix[arc.tail.name][arc.head.name])
                        #print "Head interval:" + str(self.tw_interval)
                        #time.sleep(10)

                    

                    arc.head.ingoing_arcs.append(arc)
                    if self.vrp.adapt_model:
                        #print "new_arc: %d,%d,%d,%d" %(arc.tail.name, arc.head.name,arc.tail.id,arc.head.id)
                        new_var_name = "y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (arc.tail.name,arc.head.name,
                                               arc.tail.id[0],arc.tail.id[1],
                                                              arc.head.id[0],arc.head.id[1] )
                        if self.name!=self.vrp.goal:
                            const_name = "flow_%d_%.3f_%.3f" % (self.name,old_head.id[0],old_head.id[1])
                            self.vrp.model.linear_constraints.set_coefficients([(const_name,old_var_name,0.0)]) 
                            
                            const_name = "flow_%d_%.3f_%.3f" % (self.name,arc.head.id[0],arc.head.id[1])
                            
                            self.vrp.model.linear_constraints.set_coefficients([(const_name,old_var_name,1.0)]) 
                            self.vrp.change_variable_name(old_var_name,new_var_name)
                arc_index += 1
            for i in pop_indices:
                self.interval_nodes[tw_idx].battery_nodes[idx].ingoing_arcs.pop(i)
                    
        new_arc_list=[]
        #ToDo maybe optimize breaks for not testing too many nodes
        for node_index in self.vrp.adj_matrix[self.name]:
            vrp_node_head = self.vrp.nodes[node_index]
            if self.interval_nodes[new_tw_idx].tw_interval[0]+self.vrp.adj_matrix[self.name][node_index]>vrp_node_head.tw_interval[1]+0.001:
                continue
            if split_tw:
                for bat_node in self.interval_nodes[new_tw_idx].battery_nodes:
                    #print bat_node.interval_node.tw_interval
                    arc_time,arc_bat,lt = arc_length(bat_node.interval_node.tw_interval[0],
                                                          bat_node.bat_interval[1],
                                                          vrp_node_head.tw_interval,
                                                          vrp_node_head.bat_interval,
                                                          self.vrp.adj_matrix[bat_node.name][vrp_node_head.name],
                                                          self.vrp.battery_from_depot[bat_node.name],
                                                          self.vrp.battery_from_depot[vrp_node_head.name],
                                                          self.vrp.loading_speed,
                                                          self.vrp.battery_capacity)
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
                    arc_time,arc_bat,lt = arc_length(bat_node.interval_node.tw_interval[0],
                                                          bat_node.bat_interval[1],
                                                          vrp_node_head.tw_interval,
                                                          vrp_node_head.bat_interval,
                                                          self.vrp.adj_matrix[bat_node.name][vrp_node_head.name],
                                                          self.vrp.battery_from_depot[bat_node.name],
                                                          self.vrp.battery_from_depot[vrp_node_head.name],
                                                          self.vrp.loading_speed,
                                                          self.vrp.battery_capacity)
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
                new_var_name = "y_%d_%d_%.3f_%.3f_%.3f_%.3f" % (new_arc.tail.name,new_arc.head.name,
                                                  new_arc.tail.id[0],new_arc.tail.id[1],new_arc.head.id[0],new_arc.head.id[1])
                self.vrp.add_variables([new_var_name],['C'],[0.0],[0.0],[1.0],'y')
                if new_arc.head.name != self.vrp.goal:
                    const_name = "flow_%d_%.3f_%.3f" % (new_arc.head.name,new_arc.head.id[0],new_arc.head.id[1])
                    self.vrp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)])
                if new_arc.tail.name != self.vrp.depot:
                    const_name = "flow_%d_%.3f_%.3f" % (new_arc.tail.name,new_arc.tail.id[0],new_arc.tail.id[1])
                    self.vrp.model.linear_constraints.set_coefficients([(const_name,new_var_name,-1.0)]) 
                    
                const_name = "arcuse_%d_%d" % (new_arc.tail.name,new_arc.head.name)
                self.vrp.model.linear_constraints.set_coefficients([(const_name,new_var_name,1.0)]) 
        if self.vrp.break_path:
            if new_tw_idx < len(self.interval_nodes)-2:
                self.split_node_both(self.interval_nodes[new_tw_idx+1].tw_interval[0],split_point_bat,dual_value_locations)
        return 1
    def find_tw_index(self,split_point,tol=0.001):
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
    def find_tw_index_ub(self,split_point,tol=0.001):
        if split_point < self.tw_lb-tol:
            print ("Error: split point outside of time window (less)")
            return 0,0
        else:
            if split_point > self.tw_ub+tol:
                print ("Error: split point outside of time window (larger)")
            for i,i_node in enumerate(self.interval_nodes):
                if i_node.tw_interval[1]+tol > split_point and i_node.tw_interval[1]-tol < split_point:
                    return i,0
                if i_node.tw_interval[1]+tol > split_point and i_node.tw_interval[0]+tol-2*tol*(i_node.is_tw_lb())< split_point:
                    return i,1
            return -1,0
        
    def find_bat_index(self,split_point,tw_index,tol=0.001):
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
                if i_node.bat_interval[1]+tol > split_point and i_node.bat_interval[0]+tol-2*tol*(i_node.is_bat_lb()) < split_point :
                    return i,1
    def find_bat_index_ub(self,split_point,tw_index,tol=0.001):
        if split_point < self.bat_lb-tol:
            print ("Error: split point outside of battery window (less)")
            return 0,0
        else:
            if split_point > self.bat_ub+tol:
                print ("Error: split point outside of battery window (larger)")
                return 0,0
            for i,i_node in enumerate(self.interval_nodes[tw_index].battery_nodes):
                if  i_node.bat_interval[0]+tol > split_point and i_node.bat_interval[0]-tol < split_point:
                    return i,0
                if i_node.bat_interval[0]+tol < split_point and i_node.bat_interval[1]-tol+2*tol*(i_node.is_bat_ub()) > split_point:
                    return i,1
            return -1,0
    def find_lowest_reachable(self,node,step):
        for interval_node in self.interval_nodes:
            if interval_node.is_lowest_reachable(node,step):
                return 1,interval_node
        return 0,-1
    def find_lowest_guaranteed_reachable(self,node,step):
        for interval_node in self.interval_nodes:
            if interval_node.is_lowest_guaranteed_reachable(node,step):
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
    def __init__(self,name,tw_interval,VRP_node,is_tw_lb,is_tw_ub,tol=0.001,bat_interval=[]):
        self.name = name
        self.id = tw_interval[0]
        self.id_ub = tw_interval[1]
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
    def is_reachable(self,node,shift_tw,tol=0.001):
        interv_tw1= [node.tw_interval[0]+shift_tw,node.tw_interval[1]+shift_tw-2*tol+4*tol*node.is_tw_ub()]
        interv_tw2= [(not self.is_tw_lb())*self.tw_interval[0]-tol,self.tw_interval[1]-2*tol+4*tol*self.is_tw_ub()]
        #print interv_tw1
        #print interv_tw2
        return has_intersection(interv_tw1,interv_tw2)
    def is_reachable_from_ub(self,node,shift_tw,tol=0.001):
        return node.tw_interval[1]+shift_tw < self.tw_interval[1]+tol
    def is_tw_lb(self,tol=0.001):
        return self.is_lb
    def is_tw_ub(self,tol=0.001):
        return self.is_ub
    def is_lowest_reachable(self,node,step_tw,tol=0.001):
        interv_tw=(node.tw_interval[0]+step_tw,node.tw_interval[1]-2*tol+4*tol*node.is_tw_ub()+step_tw)
        return (self.is_reachable(node,step_tw) and (
                self.is_tw_lb() or not has_intersection((0,self.tw_interval[0]-tol),interv_tw)))
    def is_lowest_guaranteed_reachable(self,node,step_tw,tol=0.001):
        arrTime = node.tw_interval[1] +step_tw
        #interv_tw=(node.tw_interval[0]+step_tw,node.tw_interval[1]-2*tol+4*tol*node.is_tw_ub()+step_tw)
        #return (self.is_reachable_from_ub(node,step_tw) and (
        #        self.is_tw_lb() or not has_intersection((0,self.tw_interval[0]-tol),interv_tw)))
        return (self.is_reachable_from_ub(node,step_tw) and (
                self.is_tw_lb() or arrTime > self.tw_interval[0]+tol))
    def find_maximal_reachable_battery_node(self,battery_node,bat_step):
        for bat_node in self.battery_nodes:
            if bat_node.is_maximal_reachable_battery_node(battery_node,bat_step):
                return 1,bat_node
        return 0,-1
    def find_maximal_guaranteed_reachable_battery_node(self,battery_node,bat_step):
        for bat_node in self.battery_nodes:
            if bat_node.is_maximal_guaranteed_reachable_battery_node(battery_node,bat_step):
                return 1,bat_node
        return 0,-1
"""
for node in vrp.nodes:
    node.interval_nodes[0].is_lb = 0
    node.interval_nodes.insert(0,Interval_node(node.name,
    [node.tw_interval[0],node.tw_interval[0]],node,0,1))
    for bat_node in node.interval_nodes[1].battery_nodes:
        node.interval_nodes[0].battery_nodes.append(Battery_node(node.name,
        list(bat_node.bat_interval),
        node.interval_nodes[0],bat_node.is_bat_lb,
        bat_node.is_bat_ub()))
    for inode in node.interval_nodes:
        inode.battery_nodes[0].is_bat_lb = 0
        inode.battery_nodes.append(Battery_node(node.name,[node.bat_interval[1],
                        node.bat_interval[1]],inode,0,1))
"""
class Battery_node():
    def __init__(self,name,bat_interval,interval_node,is_bat_lb,is_bat_ub,tol=0.001):
        self.name = name
        self.id = (interval_node.id,bat_interval[-1])
        self.id_ub = (interval_node.id_ub,bat_interval[0])
        self.bat_interval = bat_interval
        self.interval_node = interval_node
        self.ingoing_arcs = []
        self.outgoing_arcs = []
        self.ingoing_o_arcs = []
        self.outgoing_o_arcs = []
        self.is_ub = is_bat_ub
        self.is_lb = is_bat_lb
        self.big_num = 100000
    def is_reachable(self,node,shift_bat,tol=0.001):
        #print self.bat_interval
        interv_bat1=[node.bat_interval[0]+shift_bat+2*tol-4*tol*node.is_bat_lb(),node.bat_interval[1]+shift_bat]
        interv_bat2=[self.bat_interval[0]+2*tol-4*tol*self.is_bat_lb(),(not self.is_bat_ub())*self.big_num+self.bat_interval[1]+tol]
        return  has_intersection(interv_bat1,interv_bat2)
    def is_reachable_from_lb(self,node,shift_bat,tol=0.001):
        #print self.bat_interval
        return node.bat_interval[0]+shift_bat > self.bat_interval[0]-tol
    def is_reachable_from_ub(self,node,shift_bat,tol=0.001):
        return node.bat_interval[1]+shift_bat > self.bat_interval[0]+tol-2*tol*self.is_lb

    def is_bat_lb(self,tol=0.001):
        return self.is_lb
    def is_bat_ub(self,tol=0.001):
        return self.is_ub
    def is_maximal_reachable_battery_node_old(self,node,step_bat,tol=0.001):
        interv_bat=(node.bat_interval[0]+step_bat+2*tol-4*tol*node.is_bat_lb(),node.bat_interval[1]+step_bat)
        return self.is_reachable(node,step_bat) and (
                        self.is_bat_ub() or not has_intersection((self.bat_interval[1]+tol,self.big_num),interv_bat)
                        )
    def is_maximal_reachable_battery_node(self,node,step_bat,tol=0.001):
        arrBat = node.bat_interval[1]+ step_bat
        return self.is_reachable_from_ub(node,step_bat) and (
                        self.is_bat_ub() or arrBat < self.bat_interval[1]+tol)
    def is_maximal_guaranteed_reachable_battery_node(self,node,step_bat,tol=0.001):
        arrBat = node.bat_interval[0]+ step_bat
        return self.is_reachable_from_lb(node,step_bat) and (
                        self.is_bat_ub() or arrBat < self.bat_interval[1]-tol)
        
class Tree_node():
    def __init__(self,tree,branches,dual_values_model=0,dual_values_branches=0,primal_values = 0,slacks = 0,red_costs = 0):
        self.tree = tree
        self.descendents = {}
        self.branches = branches
        self.branch_names = [branch.name for branch in self.branches]
        self.branch_lin_exprs = [branch.lin_expr for branch in self.branches]
        self.branch_senses = [branch.sense for branch in self.branches]
        self.branch_rhs = [branch.rhs for branch in self.branches]
        self.branch_ubs = [(branch.var,0) for branch in self.branches if abs(branch.rhs) < 0.001]
        self.branch_lbs = [(branch.var,1) for branch in self.branches if abs(branch.rhs-1) < 0.001]
        self.restore_ubs = [(branch.var,1) for branch in self.branches if abs(branch.rhs) < 0.001]
        self.restore_lbs = [(branch.var,0) for branch in self.branches if abs(branch.rhs) < 0.001]
        self.dual_values_model = dual_values_model
        self.dual_values_branches = dual_values_branches
        self.primal_values = primal_values
        self.slacks = slacks
        self.red_costs = red_costs
        self.lower_bound = 0
        self.fractionals = {}
        if self.solve_lp_relaxation() and self.lower_bound < self.tree.ub:
            self.feasible = 1
        else:
            self.feasible = 0
    def get_one_branches(self):
        retval = 0
        for rhs in self.branch_rhs:
            if abs(rhs-1) < 0.0001:
                retval += 1
        return retval
                
    def update_dual_values(self,dual_value_locations,extra_values=[]):
        for ind in dual_value_locations:
            self.dual_values_model.append(self.dual_values_model[ind])
        if extra_values != []:
            self.dual_values_model += extra_values
    def solve_lp_relaxation(self,tol=0.001):
        vrp = self.tree.vrp
        model = vrp.model
        oldLen = len(self.branches)
        if 0:
            model.linear_constraints.add(names = self.branch_names,lin_expr = self.branch_lin_exprs,senses = self.branch_senses,rhs = self.branch_rhs)
            if self.tree.vrp.update_duals and self.dual_values_model != 0:
                model.parameters.advance.set(1)
                model.start.set_start(col_status=[],row_status=[],
                                      row_dual=self.dual_values_model+self.dual_values_branches,col_primal=[],row_primal=[],col_dual=[])
                feas = vrp.solve_model()
                #raw_input()
            else:
                #raw_input()
                feas = vrp.solve_model()
        else:
            if len(self.branch_lbs)>0:
                model.variables.set_lower_bounds(self.branch_lbs)
            if len(self.branch_ubs)>0:  
                model.variables.set_upper_bounds(self.branch_ubs)
            if self.tree.vrp.update_duals and self.dual_values_model != 0:
                model.parameters.advance.set(1)
                model.start.set_start(col_status=[],row_status=[],
                                      row_dual=self.dual_values_model,col_primal=[],row_primal=[],col_dual=[])
                feas = vrp.solve_model()
                    #raw_input()
            else:
                #raw_input()
                feas = vrp.solve_model()
        #model.parameters.preprocessing.presolve.set(0)
        
        self.updated_lp_relaxation = 1
        model.parameters.advance.set(0)
        if not feas:
            self.feasible = 0
            self.lower_bound = 1000000
            model.linear_constraints.delete(range(model.linear_constraints.get_num()-oldLen,model.linear_constraints.get_num()))
            return 0
        else:
            solution = model.solution
            if self.tree.vrp.adapt_model:
                dual_values = solution.get_dual_values()
            else:
                dual_values = []
            self.dual_values_model = dual_values[:self.tree.vrp.const_num]
            self.dual_values_branches = dual_values[self.tree.vrp.const_num:]
            self.primal_values = solution.get_values()
            self.lower_bound = solution.get_objective_value()
            
            self.primal_x_values = {name:self.primal_values[self.tree.vrp.name2idx[name]] for name in self.tree.vrp.x_names 
                                    if self.primal_values[self.tree.vrp.name2idx[name]]>tol}
            self.primal_x_values = { name:val for name,val in zip(self.tree.vrp.x_names,self.primal_values[0:self.tree.vrp.x_num])
                        if self.primal_values[self.tree.vrp.name2idx[name]]>tol }
            self.fractionals = {key:val for key,val in self.primal_x_values.items() if abs(0.5-val)<0.5-tol}
            #if len(self.fractionals) == 0:
            self.primal_y_values = {name:self.primal_values[self.tree.vrp.name2idx[name]] for name in self.tree.vrp.y_names
                        if self.primal_values[self.tree.vrp.name2idx[name]]>tol}
            #else:
            #    self.primal_y_values = {}
        model.linear_constraints.delete(range(model.linear_constraints.get_num()-oldLen,model.linear_constraints.get_num()))
        self.feasible = 1
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
        for var_name,var_val in self.fractionals.items():
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
        for var_name,var_val in self.fractionals.items():
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
            print (self.fractionals)
            print ("Error")
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
            return self.tree.psi_avg,0.0



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
        self.print_interval = 30
        self.add_all_split_points=1
        self.lbs = []
        #self.print_switch = 0
        self.psi_avg = 0.0
        self.reduced_cost_fixing = 0
        self.lp_times=[]
        self.node_count = 1
        self.refinement_count = 0
        self.cut_count = 0
        self.lp_time = 0.0
        self.lp_no_dual_times=[]
        self.simp_iteras=[]
        self.simp_nd_iteras=[]
        self.start_control = start_control
        self.closed_nodes = []
        self.vrp = vrp
        self.vrp.tree = self
        self.ub = 100000.0
        self.lb = -50.0
        self.root = Tree_node(self,[])
        self.open_nodes = [self.root]
        self.branch_history = {key:[] for key in vrp.x_names}
        self.time_limit = 7200
        self.count = 0
        self.root_count = 0
        self.refinement = 0  
    def conditional_print(self,string):
        if self.count % self.print_interval == 0:
            print (string)
    @register_time
    @register_calls
    def choose_node(self,selection=2):
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
        if selection == 2:
            minInd = 0
            minVal= 100000000
            minLb = 100000000
            for i,node in enumerate(self.open_nodes):
                if node.lower_bound - 0.1*node.get_one_branches() < minVal:
                    minInd = i
                    minVal = node.lower_bound - 0.1*node.get_one_branches() 
                if node.lower_bound < minLb:
                    minLb = node.lower_bound
            if self.lb < minLb:
                self.lb = minLb
        return self.open_nodes.pop(minInd)
    @register_time
    @register_calls
    def branch(self,node):
        branch_var,branch_val = node.choose_branch_var()
        #print "branching"
        f_1 = 1.0-branch_val
        f_0 = branch_val
        if self.vrp.update_duals:
            new_node_list = [Tree_node(node.tree,node.branches+[Branch(branch_var,'E',0.0)],
                                                                node.dual_values_model,
                                                                node.dual_values_branches+[0.0]),
                             Tree_node(node.tree,node.branches+[Branch(branch_var,'E',1.0)],
                                                                node.dual_values_model,
                                                                node.dual_values_branches+[0.0])]
        else:
            new_node_list = [Tree_node(node.tree,node.branches+[Branch(branch_var,'E',0.0)]),
                             Tree_node(node.tree,node.branches+[Branch(branch_var,'E',1.0)])]  
        if new_node_list[0].feasible:
            c_0 = (new_node_list[0].lower_bound-node.lower_bound)/f_0
        else:
            c_0 = self.psi_avg/f_0
        if new_node_list[1].feasible:
            c_1 = (new_node_list[1].lower_bound-node.lower_bound)/f_1
        else:
            c_1 = self.psi_avg/f_1
        self.branch_history[branch_var].append((c_0,c_1))
        for new_node1 in new_node_list:
            if new_node1.feasible:
                if new_node1.lower_bound < self.ub-0.99:
                    self.open_nodes.append(new_node1)
    @register_time
    @register_calls
    def remove_shortCycles(self,shortCycles):
        split_points = []
        #print "shortCycles before"
        #print shortCycles
        for S in shortCycles:
            cy= [s[0] for s in S]
            #print "extracted cycle"
            #print cy
            split_points += self.vrp.findSplitPointsToBreakCycle(cy)
            #self.vrp.add_ste_cut(cy)
        self.adapt_vrp_graph(split_points)
        
    def clean_up_and_adapt_duals(self,dual_value_locations,extra_values):
        pop_indices=[]
        for i,node2 in enumerate(self.open_nodes):
            if self.vrp.update_duals:
                node2.update_dual_values(dual_value_locations,extra_values)
            if len(extra_values)+len(dual_value_locations)>0:
                node2.updated_lp_relaxation = 0
            if not node2.feasible or node2.lower_bound >= self.ub-0.99:#TODO: Adapt this to non integer objective
                pop_indices.append(i)
        while (len(pop_indices)>0):
            self.open_nodes.pop(pop_indices.pop(-1))
    def adapt_vrp_graph(self,split_points):
        dual_value_locations=[]
        actually_split = 0
        if self.vrp.update_duals:
            for i,t,b in split_points:
                bat=b
                time=t
                actually_split += self.vrp.nodes[i].split_node_both(time,bat,dual_value_locations)
        else:
            for i,t,b in split_points:
                bat=b
                time=t
                actually_split += self.vrp.nodes[i].split_node_both(time,bat)
        return dual_value_locations,actually_split
    def splitAtRoot(self):
        prev_root_lb = 0
        root = self.open_nodes[0]
        root_splits = 0
        if root.updated_lp_relaxation == 0:
            root.solve_lp_relaxation()
        while not floatEqual(prev_root_lb,root.lower_bound):
            if root.updated_lp_relaxation == 0:
                root.solve_lp_relaxation()
            prev_root_lb = root.lower_bound
            sol,shortCycles = self.vrp.solToCyclesYopt(root.primal_y_values)
            if len(shortCycles)>0:
                i = 0
                while i < len(shortCycles):
                    if shortCycles[i][0][0] != shortCycles[i][-1][0]:
                        shortCycles.pop(i)
                    else:
                        i += 1
            if len(shortCycles)>0:
                print ("Cycles removal")
                self.remove_shortCycles(shortCycles)
                self.open_nodes=[Tree_node(self,[])]
                root  = self.open_nodes[0]
                prev_root_lb = -51.0
            else:
                sol = self.vrp.fracSolToPaths(root.primal_y_values)
                split_points = self.vrp.findSplitPoints(sol)
                if len(split_points)>0:
                    root_splits += 1
                    prev_root_lb = root.lower_bound
                    print( "Splitting nodes at ", len(split_points), "points")
                    dual_value_locations,split_successfull = self.adapt_vrp_graph(split_points)
                    self.clean_up_and_adapt_duals(dual_value_locations,[])
                    if split_successfull == 0:
                        print ("Error no node split lb: " + str(root.lower_bound))
                        print (str(sol))
                        print (split_points)
                        print (root.primal_y_values)
                        time.sleep(10) 
                
                self.open_nodes=[Tree_node(self,[])]
                root = self.open_nodes[0]
        print ("Root Splits: %d" %root_splits)
        #time.sleep(3)
    @register_time
    @register_calls
    def branch_and_refine(self):
        if self.use_best_heuristic:
            self.ub = self.heuristic_ub
        t0=time.time()
        self.count=0
        prev_root_lb = self.root.lower_bound
        if self.vrp.refine_heuristic:
            self.splitAtRoot()
            heur_sol_val = self.vrp.solve_heuristic()
            if heur_sol_val < self.ub:
                print ("Heuristic found new upper bound: %.2f" % heur_sol_val)
                self.ub = heur_sol_val
        #return
        while ( len(self.open_nodes)>0) and (time.time()-t0 < self.time_limit):
            self.count+=1
            self.conditional_print("Current lower bound: %f" % (self.lb))
            self.conditional_print("Current best upper Bound: %f " % (self.ub))
            self.conditional_print("Number of open nodes: %d" % len(self.open_nodes))
            node = self.choose_node()
            if node.updated_lp_relaxation == 0:
                node.solve_lp_relaxation()
            if not node.feasible or node.lower_bound >= self.ub-0.99:
                continue
            if len(node.fractionals)>0:
                self.branch(node)
                continue
                #ab hier ist fuer split auf fraktionalen vars    
                sol,shortCycles = self.vrp.solToCyclesYopt(node.primal_y_values)
                if len(shortCycles)>0:
                    i = 0
                    while i < len(shortCycles):
                        if shortCycles[i][0][0] != shortCycles[i][-1][0]:
                            shortCycles.pop(i)
                        else:
                            i += 1
                if len(shortCycles)>0:
                    print( "Cycles removal")
                    self.remove_shortCycles(shortCycles)
                else:
                    split_points = self.vrp.findSplitPoints(sol)
                    if len(split_points)>0:
                        print (sol)
                        print( "Splitting nodes by lp relaxation")
                        self.open_nodes.append(node)
                        dual_value_locations,split_successfull = self.adapt_vrp_graph(split_points)
                        self.clean_up_and_adapt_duals(dual_value_locations,[])
                        if split_successfull == 0:
                            print ("Error no node split lb: " + str(node.lower_bound))
                            print (str(split_points))
                            print (split_points)
                            print (node.primal_y_values)
                            time.sleep(10)
                        heur_sol_val = self.vrp.solve_heuristic()
                        if heur_sol_val < self.ub:
                            print ("Heuristic found new upper bound: %.2f" % heur_sol_val)
                            self.ub = heur_sol_val
                    else:
                        self.branch(node)
                    continue
            #this part should only be reached for integer solutions
            #Segment to add STE Cuts, if a cut is added loop is continued
            #sol,shortCycles = solToCyclesMultiVisit(node.primal_x_values,node.primal_y_values)
            sol,shortCycles = self.vrp.solToCyclesYopt(node.primal_y_values)
            
            if len(shortCycles)>0:
                self.open_nodes.append(node)
                self.remove_shortCycles(shortCycles)
                self.clean_up_and_adapt_duals([],[0.0]*len(shortCycles))
                continue
            
            #Segment to split nodes, if nodes are split loop is continued
            #sol = [[v[0] for v in path] for path in sol]
            #split_points = self.vrp.findSplitPointsInfPathRef(sol)#splitPoints pruft, ob die Wege in
            split_points = self.vrp.findSplitPoints(sol)
            if len(split_points)>0:
                print( "Splitting nodes at ", len(split_points), "points")
                self.open_nodes.append(node)
                dual_value_locations,split_successfull = self.adapt_vrp_graph(split_points)
                self.clean_up_and_adapt_duals(dual_value_locations,[])
                if split_successfull == 0:
                    print ("Error no node split lb: " + str(node.lower_bound))
                    print (str(sol))
                    print (split_points)
                    print (node.primal_y_values)
                    time.sleep(10)
                    heur_sol_val = self.vrp.solve_heuristic()
                    if heur_sol_val < self.ub:
                        print ("Heuristic found new upper bound: %.2f" % heur_sol_val)
                        self.ub = heur_sol_val
                new_root = Tree_node(self,[])
                print ("current root lb = %.2f" % new_root.lower_bound)
                if abs(new_root.lower_bound-self.lb)<0.001 or (abs(new_root.lower_bound-prev_root_lb)>0.5) :
                    print ("Root node has same lower bound as branch tree, restarting from root")
                    self.open_nodes=[new_root]
                    self.lb = new_root.lower_bound
                    prev_root_lb = new_root.lower_bound
                continue
            #segment if no cut was added and nodes were not split
            if node.lower_bound < self.ub-0.99:
                #self.solution_path = self.vrp.solToPaths(sol)
                print( "Integer feasible solution found, objective: %f" %node.lower_bound)
                self.ub = node.lower_bound
                self.solution_node = node
                node.sol = sol
                self.clean_up_and_adapt_duals([],[])
    @register_time
    @register_calls
    def dynamic_discovery(self):
        if self.use_best_heuristic:
            self.ub = self.heuristic_ub
        t0 = time.time()
        while time.time()-t0 < self.time_limit:
            while len(self.open_nodes)>0 and time.time()-t0 < self.time_limit:
                self.count+=1
                self.conditional_print("Current lower bound: %f" % (self.lb))
                self.conditional_print("Current best upper Bound: %f " % (self.ub))
                self.conditional_print("Number of open nodes: %d" % len(self.open_nodes))
                node = self.choose_node()
                if node.updated_lp_relaxation == 0:
                    node.solve_lp_relaxation()
                if not node.feasible or node.lower_bound >= self.ub-0.99:
                    continue
                if len(node.fractionals)>0:
                    self.branch(node)
                    continue
                
                sol,shortCycles = self.vrp.solToCyclesYopt(node.primal_y_values)
                if len(shortCycles)>0:
                    self.open_nodes.append(node)
                    self.remove_shortCycles(shortCycles)
                    self.clean_up_and_adapt_duals([],[0.0]*len(shortCycles))
                    continue
                

                #segment if no cut was added and nodes were not split
                if node.lower_bound < self.ub-0.99:
                    self.sol = sol
                    #self.solution_path = self.vrp.solToPaths(sol)
                    print( "Integer feasible solution found, objective: %f" %node.lower_bound)
                    self.ub = node.lower_bound
                    self.solution_node = node
                    self.clean_up_and_adapt_duals([],[])
    
            split_points = self.vrp.findSplitPoints(self.sol)
            if len(split_points)>0:
                self.ub = 0
                self.lb = -51
                print( "Splitting nodes")
                self.open_nodes.append(node)
                dual_value_locations,split_successfull = self.adapt_vrp_graph(split_points)
                self.clean_up_and_adapt_duals(dual_value_locations,[])
                if split_successfull == 0:
                    print( "Error no node split lb: " + str(node.lower_bound))
                    print (str(node.primal_y_values))
                self.root = Tree_node(self,[])
                self.open_nodes = [self.root]
            else:
                break
    @register_time
    @register_calls
    def dynamic_discovery_cplex(self):
        if self.use_best_heuristic:
            self.ub = self.heuristic_ub
        t0 = time.time()
        while time.time()-t0 < self.time_limit and self.lb<self.ub-0.99:
            no_cycles = 0
            while no_cycles == 0:
                self.vrp.create_model('B')            #model.parameters.preprocessing.presolve.set(0)
                self.vrp.solve_model2()
                model = self.vrp.model
                solution = model.solution
                primal_values = solution.get_values()
                lower_bound = solution.get_objective_value()
                tol = 0.001
                primal_y_values = {name:primal_values[self.vrp.name2idx[name]] for name in self.vrp.y_names
                            if primal_values[self.vrp.name2idx[name]]>tol}
                sol,shortCycles = self.vrp.solToCyclesYopt(primal_y_values)
                if len(shortCycles)>0:
                    self.remove_shortCycles(shortCycles)
                else:
                    no_cycles = 1
            split_points = self.vrp.findSplitPoints(sol)
            if len(split_points)>0:
                self.ub = self.heuristic_ub
                self.lb = lower_bound
                print( "Splitting nodes")
                dual_value_locations,split_successfull = self.adapt_vrp_graph(split_points)
            else:
                self.ub = lower_bound
                break

def process_adj_matrix(adj_matrix,TWs,bat_intervals,battery_matrix,loading_speed,battery_capacity):
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
    print ("Total number of arcs before second preprocessing step: %d" % oldArcAmount)
    #return
    for i in adj_matrix:
        toRemove = []
        for j in adj_matrix[i]:
            for k in adj_matrix[i]:
                if j in adj_matrix[k].keys():
                    time_delta_ik,bat_delta_ik,lt = arc_length(TWs[i][1],
                                                          bat_intervals[i][0],
                                                          TWs[k],
                                                          bat_intervals[k],
                                                          adj_matrix[i][k],
                                                          battery_matrix[0][i],
                                                          battery_matrix[0][k],
                                                          loading_speed,
                                                          battery_capacity)
                    
                    if TWs[i][1]+time_delta_ik+adj_matrix[k][j]+(battery_capacity-
                          (bat_intervals[i][0]+bat_delta_ik))/load_fac <TWs[j][0]:
                        toRemove.append(j)
                        #print "Removing arc (%d,%d) because %d can be visited in between at no cost" % (i,j,k)
                        #print "Previous time: %f" % adj_matrix[i][j]
                        #print "Extra travel time: %f" % ((adj_matrix[i][k]+adj_matrix[k][j]-adj_matrix[i][j])*(1+1/load_fac))
                        break
        for j in toRemove:
            adj_matrix[i].pop(j)

def write_line(filename,instance_name,data):
    file = open(filename, "a")
    #file.write("Instance " + instance_name + ":\nSolve_time: %.2f, Nodes explored: %d, Branches: %d, Obj: %.0f\n" %data)
    file.write(instance_name + "\n%.2f %d %d %.0f\n" %data)
    file.close()
        


#1:[[0,18,3,47,5,11,29,48,13,51],[0,32,14,10,26,42,7,36,51],[0,35,50,15,33,45,6,2,51]]
#2:[[0,2,41,21,9,3,11,12,51],[0,5,19,39,36,34,50,46,51],[0,10,13,15,23,47,28,37,51]]
#7: [[0,1,32,30,40,5,46,36,39,51],[0,20,4,22,12,50,15,11,51],[0,27,8,43,17,6,42,7,29,51]]
instance_names = {
"SimCologne_C_50_twLength_30_i_1_equalProfits.txt":-22,
#"SimCologne_C_50_twLength_30_i_2_equalProfits.txt":-21,
#"SimCologne_C_50_twLength_30_i_3_equalProfits.txt":-26,
#"SimCologne_C_50_twLength_30_i_4_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_30_i_5_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_30_i_6_equalProfits.txt":-24,
#"SimCologne_C_50_twLength_30_i_7_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_30_i_8_equalProfits.txt":-26,
#"SimCologne_C_50_twLength_30_i_9_equalProfits.txt":-24,
#"SimCologne_C_50_twLength_30_i_10_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_30_i_11_equalProfits.txt":-22,
#"SimCologne_C_50_twLength_30_i_12_equalProfits.txt":-22,
#"SimCologne_C_50_twLength_30_i_13_equalProfits.txt":-24,
#"SimCologne_C_50_twLength_30_i_14_equalProfits.txt":-24,
#"SimCologne_C_50_twLength_30_i_15_equalProfits.txt":-21,             
                  }
instance_names = {
#"SimCologne_C_50_twLength_60_i_1_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_60_i_2_equalProfits.txt":-22,
#"SimCologne_C_50_twLength_60_i_3_equalProfits.txt":-27,
#"SimCologne_C_50_twLength_60_i_4_equalProfits.txt":-25,
#"SimCologne_C_50_twLength_60_i_5_equalProfits.txt":-25,
#"SimCologne_C_50_twLength_60_i_6_equalProfits.txt":-25,
#"SimCologne_C_50_twLength_60_i_7_equalProfits.txt":-24,
"SimCologne_C_50_twLength_60_i_8_equalProfits.txt":-27,
#"SimCologne_C_50_twLength_60_i_9_equalProfits.txt":-25,
#"SimCologne_C_50_twLength_60_i_10_equalProfits.txt":-25,
#"SimCologne_C_50_twLength_60_i_11_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_60_i_12_equalProfits.txt":-23,
#"SimCologne_C_50_twLength_60_i_13_equalProfits.txt":-25,
#"SimCologne_C_50_twLength_60_i_14_equalProfits.txt":-25,
#"SimCologne_C_50_twLength_60_i_15_equalProfits.txt":-22,             
                  }
instance_names = {
"SimCologne_C_50_twLength_60_i_1_equalProfits.txt":-23,
"SimCologne_C_50_twLength_60_i_2_equalProfits.txt":-22,
"SimCologne_C_50_twLength_60_i_3_equalProfits.txt":-27,
"SimCologne_C_50_twLength_60_i_4_equalProfits.txt":-25,
"SimCologne_C_50_twLength_60_i_5_equalProfits.txt":-25,
"SimCologne_C_50_twLength_60_i_6_equalProfits.txt":-25,
"SimCologne_C_50_twLength_60_i_7_equalProfits.txt":-24,
"SimCologne_C_50_twLength_60_i_8_equalProfits.txt":-27,
"SimCologne_C_50_twLength_60_i_9_equalProfits.txt":-25,
"SimCologne_C_50_twLength_60_i_10_equalProfits.txt":-25,
"SimCologne_C_50_twLength_60_i_11_equalProfits.txt":-23,
"SimCologne_C_50_twLength_60_i_12_equalProfits.txt":-23,
"SimCologne_C_50_twLength_60_i_13_equalProfits.txt":-25,
"SimCologne_C_50_twLength_60_i_14_equalProfits.txt":-25,
"SimCologne_C_50_twLength_60_i_15_equalProfits.txt":-22,             
                  }
start_paramsens = [
                {"vehicle_amount":3,
                "update_duals" : 1,
                "multivisit_model" :1,
                "add_priority_cuts_at_start":1,
                "remove_infeasible_paths_at_start":0,
                "remove_cycles_at_start":0,
                "refine_heuristic":1
                        },
#                 {"vehicle_amount":3,
#                "update_duals" : 1,
#                "multivisit_model" :1,
#                "add_priority_cuts_at_start":0,
#                "remove_infeasible_paths_at_start":1,
#                "remove_cycles_at_start":1,
#                        },
#                  {"vehicle_amount":3,
#                "update_duals" : 1,
#                "multivisit_model" :1,
#                "add_priority_cuts_at_start":1,
#                "remove_infeasible_paths_at_start":0,
#                "remove_cycles_at_start":1,
#                        },
#                  {"vehicle_amount":3,
#                "update_duals" : 1,
#                "multivisit_model" :1,
#                "add_priority_cuts_at_start":1,
#                "remove_infeasible_paths_at_start":1,
#                "remove_cycles_at_start":1,
#                        },
                ]
for dynamic_discovery in [0]:
    for start_params in start_paramsens:
        startHeurIter = 0
        cheat = 0
        if dynamic_discovery:
            saveFileName = "Results_BAT_dyn_disc"+str(dynamic_discovery)
        else:
            saveFileName = "Results_BAT_BNT"+str(cheat)
        
        now = datetime.now()
        start_line = now.strftime("%H:%M:%S\n:"+str(start_params)+"\n")
        file = open(saveFileName, "a")
        file.write("\n______________________________________\n")
        file.write(start_line)
        file.close()
        use_best_heuristic = 0
        bat_unit = 1.0
        load_fac = bat_unit*2.5
        loading_speed = load_fac
        service_time = 5
        battery_capacity = bat_unit*120
        speed = 60
        time_horizon = 720.0
        inst_num = 0
        
        calc_cplex_time = 0
        
        for instance_name in sorted(instance_names.keys()):
            function_times = {}
            function_calls = {}
            function_times = {'branch':0.0,'solve_model':0.0,'choose_node':0}
            function_calls = {'branch':0,'solve_model':0,'choose_node':0}
            error_messsages = []
            inst_num += 1
            
            vert_num,TWs,adj_matrix,profits,depotDists = readData(instance_name,"BatVRP")
            
            full_adj_matrix = {i:{j:val for j,val in adj_matrix[i].items()} for i in adj_matrix}
            
            bat_intervals = [[bat_unit*depotDists[i],battery_capacity-bat_unit*depotDists[i]] for i in adj_matrix]
            bat_intervals[0]=[battery_capacity,battery_capacity]
            battery_matrix = {i : { j : -bat_unit*(depotDists[i]+depotDists[j]) for j in adj_matrix} for i in adj_matrix}
            
            process_adj_matrix(adj_matrix,TWs,bat_intervals,battery_matrix,loading_speed,battery_capacity)
            
            
            #break
            vrp = VRP(TWs,bat_intervals,full_adj_matrix,adj_matrix,profits,battery_matrix,0,len(TWs)-1,load_fac,battery_capacity,start_params)
            vrp.full_adj_matrix=full_adj_matrix
            vrp.update_duals=0
            vrp.adapt_model=0
            #time.sleep(3)
            #for i in vrp.nodes:
            #    if i.name == 0:
            #        continue
            #    i.split_node_both(i.tw_interval[0],i.bat_interval[0],0)
            #    i.split_node_both(i.tw_interval[1],i.bat_interval[0],0)
            vrp.update_duals=1
            vrp.adapt_model=1
            vrp.create_model()
            tree = Tree(vrp,0)
            tree.service_time = service_time
            tree.use_best_heuristic = use_best_heuristic
            tree.heuristic_ub = instance_names[instance_name]
            tree.time_limit = 3600
            #break
            if use_best_heuristic:
                tree.ub = instance_names[instance_name]
            if dynamic_discovery==0:
                tree.branch_and_refine()
            if dynamic_discovery==1:
                tree.dynamic_discovery()
            if dynamic_discovery==2:
                tree.dynamic_discovery_cplex()
            if cheat:
                function_times = {'branch':0.0}
                function_calls = {'branch':0}
                error_messsages = []
                if use_best_heuristic:
                    tree.ub = instance_names[instance_name]
                else:
                    tree.ub = 0
                tree.lb=-51
                vrp.create_model()
                tree.open_nodes=[Tree_node(tree,[])]
                tree.open_nodes[0].solve_lp_relaxation()
                tree.branch_and_refine()
                vrp.create_model('B')
            if calc_cplex_time:
                t0 = time.time()
                vrp.model.solve()
                cplex_time = time.time()-t0
            print( str(function_times))
            print (str(function_calls))
        
            lineData = (function_times['solve_model'], function_calls['choose_node'], function_calls['branch'],tree.ub)
            write_line(saveFileName,instance_name,lineData)
            file = open(saveFileName, "a")
            if calc_cplex_time:
                file.write(" %.2f\n" % cplex_time)
            file.close()
        file = open(saveFileName, "a")
        file.write("______________________________________\n")
        file.close()
        #break