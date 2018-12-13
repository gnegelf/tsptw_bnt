import cplex
import random

model = cplex.Cplex()
size=2199
A= [[0 for i in range(size)] for j in range(size) ]

for i in range(400):
    for j in range(400):
        A[random.randint(0,size-1)][random.randint(0,size-1)] = random.randint(0,12)
b =[random.randint(0,30) for i in range(size)]
chosenRows = [i for i in range(1500)]
chosenCols = [i for i in range(1500)]
obj=[random.randint(-10,0) for d in chosenCols]
model.variables.add(names = ["%d" %d for d in chosenCols],ub = [10 for d in chosenCols],
                             lb=[0 for d in chosenCols],obj=[obj[d] for d in chosenCols])


allrhs= b
allsenses = ['L']*len(b)
allvars = [cplex.SparsePair([i for i in chosenCols],[A[j][i] for i in chosenCols]) for j in chosenRows]
model.linear_constraints.add(names = ["%d"%j for j in chosenRows],lin_expr = allvars, 
                                                     senses = ['L' for i in chosenRows], rhs = [b[i] for i in chosenRows])

model.solve()
chosenRows = [i for i in range(1500,1510)]
#chosenCols = [i for i in range(1500,1510)]
print model.solution.get_objective_value()


allvars = [cplex.SparsePair([i for i in chosenCols],[A[j][i] for i in chosenCols]) for j in chosenRows]
model.linear_constraints.add(names = ["%d"%j for j in chosenRows],lin_expr = allvars, 
                                                     senses = ['L' for i in chosenRows], rhs = [b[i] for i in chosenRows])
model.linear_constraints.delete(["%d" %j for j in range(100)])

model.solve()

model2=cplex.Cplex()
chosenRows = [i for i in range(100,1510)]
chosenCols = [i for i in range(1500)]
model2.variables.add(names = ["%d" %d for d in chosenCols],ub = [10 for d in chosenCols],
                             lb=[0 for d in chosenCols],obj=[obj[d] for d in chosenCols])



allvars = [cplex.SparsePair([i for i in chosenCols],[A[j][i] for i in chosenCols]) for j in chosenRows]
model2.linear_constraints.add(names = ["%d"%j for j in chosenRows],lin_expr = allvars, 
                                                     senses = ['L' for i in chosenRows], rhs = [b[i] for i in chosenRows])

model2.solve()
print model.solution.get_objective_value()
print model2.solution.get_objective_value()