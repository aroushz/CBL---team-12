import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult

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



#print(dijk("A",Adj)["D"])



class GoToGoalNode(Node):
    def __init__(self, coords):
        super().__init__('go_to_goal_node')
        self.navigator = BasicNavigator()
        self.goal_pose = PoseStamped()

        # Define your goal position (Point B)
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = coords[0]  # Adjust these values to your target
        self.goal_pose.pose.position.y = coords[1]
        self.goal_pose.pose.orientation.w = 1.0  # Facing forward

        self.navigator.waitUntilNav2Active()

        # Send the goal
        self.navigator.goToPose(self.goal_pose)

        # Monitor progress
        self.check_goal_completion()

    def check_goal_completion(self):
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and feedback.distance_remaining:
                self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} meters")
            rclpy.spin_once(self, timeout_sec=1.0)

        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info("Goal reached successfully!")
        elif result == NavigationResult.CANCELED:
            self.get_logger().warn("Goal was canceled.")
        elif result == NavigationResult.FAILED:
            self.get_logger().error("Goal failed!")

        # Stop the robot gracefully
        self.navigator.cancelTask()



def main(args=None):

    currentlyAt = "A"
    currentGoal = "D"

    Adj = {
        "A":[["B",1],["D",10]],
        "B":[["A",1],["C",1]],
        "C":[["B",1],["D",1]],
        "D":[["C",1],["A",10]],
    }
    pointCoordinates = {
        "A":[1,1],
        "B":[2,2],
        "C":[3,3],
        "D":[4,4]
    }

    path = dijk(currentlyAt,Adj)[currentGoal]["Path"]

    
    rclpy.init(args=args)
    for i in path:

        print("Going to "+i[1])
        node = GoToGoalNode(pointCoordinates[i[1]])
        node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()