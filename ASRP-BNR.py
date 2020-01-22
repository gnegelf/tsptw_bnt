#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 10 13:49:13 2020

@author: fabiangnegel
"""
import cplex

class AirportGraph():
    def __init__(self,adj_matrix):
        self.adj_matrix = adj_matrix
    def add_copy(self,node):
        self.nodes[node].append(len(self.nodes[node]))


class MILPHandler():
    def __init__(self,milp):
        self.milp = milp
        self.var2idx = {}
        self.const2idx = {}
        self.var2flowconstList = {}
        self.idx2var = {}
        self.idx2const = {}
        self.dual_values = []
        self.original_constraints = {}#this dictionary should contain another dictionary for each node
        #                       which contains the constraints as keys (which will be the exceptions) and a list of variables to copy as value
        self.branchable_vars = []
    def add_variable(self,varname,vartype,lb=0,ub=1):
        self.var2idx[varname] = self.milp.variables.get_num()
        self.idx2var[self.milp.variables.get_num()] = varname
        self.var2flowconstList[varname] = {}
        self.milp.variables.add(name=varname,lb=lb,ub=ub,var_type=vartype)
        if vartype in ["I","B"]:
            self.branchable_vars.append(varname)
    def add_constraint(self,variables,coeffs,name,rhs,const_type):
        self.const2idx[name] = self.milp.linear_constraints.get_num()
        self.idx2const[self.milp.linear_constraints.get_num()] = name
        for var,coeff in zip(variables,coeffs):
            self.var2constList[var][name]=coeff
        self.milp.linear_constraints.add(names=[name],lin_exp=[cplex.SparsePair(variables,coeffs)],rhs=rhs,const_type=[const_type])
    def add_node_copy(self, node):
        for constraint,variables in self.original_constraints[node].iteritems():
            for variable in variables:
                #it is necessary to track the coefficient and the added variables somehow
                self.add_var_copy(variable[0],variable[1],variable[2],variable[3],exceptions=constraint)#take care of correct lower and upper bounds
        self.add_constraint(adapted_constraint_name,thevars,thecoeffs)
    def add_var_copy(self,varname,newvarname,exceptions=[],newtype="",newlb=0,newub=1):
        if newtype=="":
            newtype=self.milp.variables.get_type(self.var2idx[varname])
        self.add_variable(newvarname,newtype,newlb,newub)
        for const,coeff in self.var2constList[varname].iteritems():
            if const not in exceptions:
                self.milp.linear_constraints.change_coeff(newvarname,const,coeff)
                self.dual_values.append(self.dual_values[self.const2idxconst])
                
                
#jede node sollte zusätzlich eine Liste der branches auf Originalvariablen haben für schnelleres anpassen der branches
class Tree_node():
    def __init__(self):
        self.lower_bound = 0

class Tree():
    def __init__(self):
        self.open_nodes = []
        
    def branch_and_refine(self):
        while self.open_nodes != []:
            node = self.get_node()
            node.calc_lb()
            if node.lb>self.ub:
                continue
            if node.is_integer():
                if node.check_feasibility():
                    self.ub = node.lb
                else:
                    for node in node.too_aggregated_nodes():
                        self.milp_handler.add_node_copy(node)
                    self.open_nodes.append()
            else:
                branch_var,branch_val = node.get_branch_spec()
                self.add_branch(branch_var,branch_val)
                