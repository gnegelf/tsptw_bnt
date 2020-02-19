#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 18 16:27:35 2020

@author: fabiangnegel
"""

import cplex

import random
zahl = 300
A=[[random.randint(0,1) for j in range(zahl)]for i in range(zahl)]
b=[random.randint(80,85) for j in range(zahl)]
c=[-1 for i in range(zahl)]

model = cplex.Cplex()

for i,row in enumerate(A):
    model.variables.add(names=["x%d"%i],obj=[c[i]],lb=[0.0],ub=[1.0],types=['C'])

for i,row in enumerate(A):
    thevars = ["x%d"%j for j,row2 in enumerate(A)]
    thecoefs = row
    rhs = [ b[i] ]
    #print thecoefs
    #print thevars
    #print rhs
    model.linear_constraints.add(lin_expr=[cplex.SparsePair(thevars,thecoefs)],rhs=rhs,senses=["L"])

model.parameters.simplex.display.set(2)
model.set_problem_type(0)
model.solve()
print model.solution.get_values()
Bcol,Brow = model.solution.basis.get_basis()
dual_values = model.solution.get_dual_values()
reduced_costs = model.solution.get_reduced_costs()
slacks = model.solution.get_linear_slacks()
primals = model.solution.get_values()
model.parameters.advance.set(1)
model.parameters.lpmethod.set(2)
model.parameters.simplex.display.set(2)
newlinexpr = cplex.SparsePair(["x%d"%j for j in range(1)],[1 for j in range(1)])
model.linear_constraints.add(lin_expr=[newlinexpr],rhs=[0.0],senses=["L"])
model.variables.set_upper_bounds(0,0)
model.solve()

"""
#print model.solution.get_values()
model2 = cplex.Cplex()

for i,row in enumerate(A):
    model2.variables.add(names=["x%d"%i],obj=[c[i]],lb=[0.0],ub=[1.0],types=['C'])

for i,row in enumerate(A):
    thevars = ["x%d"%j for j,row2 in enumerate(A)]
    thecoefs = row
    rhs = [ b[i] ]
    #print thecoefs
    #print thevars
    #print rhs
    model2.linear_constraints.add(lin_expr=[cplex.SparsePair(thevars,thecoefs)],rhs=rhs,senses=["L"])
model2.parameters.simplex.display.set(2)
model2.set_problem_type(0)
#model2.solve()
#print model2.solution.get_values()
#Bcol,Brow = model2.solution.basis.get_basis()
#dual_values = model2.solution.get_dual_values()
model2.parameters.advance.set(1)
model2.parameters.lpmethod.set(2)
#model2.parameters.simplex.display.set(2)

model2.linear_constraints.add(lin_expr=[newlinexpr],rhs=[0.0],senses=["L"])
#dual_values = [random.randint(0,1) for j in dual_values]
model2.start.set_start(col_status=Bcol,row_status=Brow+[1], row_dual=dual_values+[0],col_primal=primals,row_primal=[0],col_dual=reduced_costs)
#model2.start.set_start(col_status=Bcol,row_status=Brow+[0], row_dual=[1,8,9,4],col_primal=[],row_primal=[],col_dual=[])
#model2.start.set_start(col_status=[],row_status=[], row_dual=dual_values+[0],col_primal=[],row_primal=[],col_dual=[])

model2.solve()
#print model2.solution.get_values()

model22 = cplex.Cplex()

for i,row in enumerate(A):
    model22.variables.add(names=["x%d"%i],obj=[c[i]],lb=[0.0],ub=[1.0],types=['C'])

for i,row in enumerate(A):
    thevars = ["x%d"%j for j,row2 in enumerate(A)]
    thecoefs = row
    rhs = [ b[i] ]
    #print thecoefs
    #print thevars
    #print rhs
    model22.linear_constraints.add(lin_expr=[cplex.SparsePair(thevars,thecoefs)],rhs=rhs,senses=["L"])
model22.parameters.simplex.display.set(2)
model22.set_problem_type(0)
#model22.solve()
#print model22.solution.get_values()
#Bcol,Brow = model22.solution.basis.get_basis()
#dual_values = model22.solution.get_dual_values()
model22.parameters.advance.set(1)
model22.parameters.lpmethod.set(2)
#model22.parameters.simplex.display.set(2)

model22.linear_constraints.add(lin_expr=[newlinexpr],rhs=[0.0],senses=["L"])
#dual_values = [random.randint(0,1) for j in dual_values]
#model22.start.set_start(col_status=Bcol,row_status=Brow+[1], row_dual=dual_values+[0],col_primal=[],row_primal=[],col_dual=[])
#model22.start.set_start(col_status=Bcol,row_status=Brow+[0], row_dual=[1,8,9,4],col_primal=[],row_primal=[],col_dual=[])
model22.start.set_start(col_status=[],row_status=[], row_dual=dual_values+[0],col_primal=[],row_primal=[],col_dual=[])

model22.solve()
#print model22.solution.get_values()"""