


import re
import random

def readData(fileName,directoryName ="BatVRP"):
    
    print("reading "+fileName)

    file = open(directoryName+"/"+fileName, "r")
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
    unwantedChars = ['\r','']
    listsToClean=[depotDists,TWlbs,TWubs,profits]
    for cleanMe in listsToClean:
        for charac in unwantedChars:
            if charac in cleanMe:
                cleanMe.remove(charac)
    #ToDo: Dieses korrigieren
    timeWindows = [(TWlbs[i],TWubs[i]) for i in range(len(TWlbs))]
    depotDists = [i for i in depotDists]
    profits = [i for i in profits]
    return depotDists,timeWindows,profits
instanceNames = {
"SimCologne_C_50_twLength_30_i_1_equalProfits.txt":-22,
"SimCologne_C_50_twLength_30_i_2_equalProfits.txt":-21,
"SimCologne_C_50_twLength_30_i_3_equalProfits.txt":-26,
"SimCologne_C_50_twLength_30_i_4_equalProfits.txt":-23,
"SimCologne_C_50_twLength_30_i_5_equalProfits.txt":-23,
"SimCologne_C_50_twLength_30_i_6_equalProfits.txt":-24,
"SimCologne_C_50_twLength_30_i_7_equalProfits.txt":-23,
"SimCologne_C_50_twLength_30_i_8_equalProfits.txt":-26,
"SimCologne_C_50_twLength_30_i_9_equalProfits.txt":-24,
"SimCologne_C_50_twLength_30_i_10_equalProfits.txt":-23,
"SimCologne_C_50_twLength_30_i_11_equalProfits.txt":-22,
"SimCologne_C_50_twLength_30_i_12_equalProfits.txt":-22,
"SimCologne_C_50_twLength_30_i_13_equalProfits.txt":-24,
"SimCologne_C_50_twLength_30_i_14_equalProfits.txt":-24,
"SimCologne_C_50_twLength_30_i_15_equalProfits.txt":-21,             
                  }
instNum = 15
cNum = 25
depotDists,timeWindows,profits = [],[],[]
for instanceName in instanceNames:
    ndepotDists,ntimeWindows,nprofits  = readData(instanceName)
    depotDists += ndepotDists
    timeWindows += ntimeWindows
    profits += nprofits
for j in range(instNum):
    usedIndices = []
    for i in range(cNum):
        ind = random.randint(0, len(timeWindows))
        while ind in usedIndices:
            ind = random.randint(0, len(timeWindows))
        usedIndices.append(ind)
    fileName=f"SimCologne_C_{cNum}_twLength_30_i_{j+1}_equalProfits.txt"
    file = open("BatVRP/"+fileName, "w")
    for index in usedIndices:
        file.write(f"{depotDists[index]} ")
    file.write("\n")
    for index in usedIndices:
        file.write(f"{timeWindows[index][0]} ")
    file.write("\n")
    for index in usedIndices:
        file.write(f"{timeWindows[index][1]} ")
    file.write("\n")
    for index in usedIndices:
        file.write(f"{profits[index]} ")
    file.close()