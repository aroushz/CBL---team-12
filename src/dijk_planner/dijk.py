def dijkNodes(dijkAdj):
    Nodes = {}
    for node in dijkAdj:
        Nodes[node] = {"Distance":999999, "Path":[]}
    return Nodes
def dijkSmallest(dijkNodes):
    Smallest = None
    SmallestDist = 999999
    for i in dijkNodes:
        if(dijkNodes[i]["Distance"]<SmallestDist):
            SmallestDist = dijkNodes[i]["Distance"]
            Smallest = i
    return Smallest
def dijkOperate(dijkNodes, dijkAdj, Smallest):
    smallestDist = dijkNodes[Smallest]["Distance"]
    for adj in dijkAdj[Smallest]:
        if(adj[0] in dijkNodes):
            oldDist = dijkNodes[adj[0]]["Distance"]
            newDist = smallestDist + adj[1]
            if(newDist<oldDist):
                dijkNodes[adj[0]]["Path"] = dijkNodes[Smallest]["Path"] + [[Smallest, adj[0]]]
                dijkNodes[adj[0]]["Distance"] = newDist
    return dijkNodes
def dijk(start, dijkAdj):
    nodes = dijkNodes(dijkAdj)
    finalNodes = {}
    nodes[start]["Distance"] = 0
    while(len(nodes)>0):
        small = dijkSmallest(nodes)
        nodes = dijkOperate(nodes, dijkAdj, small)
        finalNodes[small] = nodes[small]
        nodes.pop(small)
    return finalNodes

def makeCoef(length, traffic, safetyRisk, roadCondition, busDamage, safetyThres):
    if(safetyRisk > safetyThres):
        return 9999999
    return traffic + length + safetyRisk + roadCondition * busDamage

Adj = {
    "A":[["B",1],["D",10]],
    "B":[["A",1],["C",1]],
    "C":[["B",1],["D",1]],
    "D":[["C",1],["A",10]],
}

print(dijk("A",Adj)["D"])

