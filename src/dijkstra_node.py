#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator

# ----------  your functions unchanged  ----------
def dijkNodes(adj): return {n:{"Distance":999999,"Path":[]} for n in adj}
def dijkSmallest(nodes): return min(nodes, key=lambda k:nodes[k]["Distance"])
def dijkOperate(nodes, adj, s):
    for nxt,c in adj[s]:
        if nxt in nodes and nodes[s]["Distance"]+c < nodes[nxt]["Distance"]:
            nodes[nxt]["Distance"]=nodes[s]["Distance"]+c
            nodes[nxt]["Path"]=nodes[s]["Path"]+[[s,nxt]]
    return nodes
def dijk(start, adj):
    nodes, done = dijkNodes(adj), {}
    nodes[start]["Distance"]=0
    while nodes:
        s=dijkSmallest(nodes)
        nodes=dijkOperate(nodes,adj,s)
        done[s]=nodes.pop(s)
    return done
# -----------------------------------------------

# toy graph & **POSES you measure in RViz** -------------
ADJ = {"A":[["B",1],["D",10]],
       "B":[["A",1],["C",1]],
       "C":[["B",1],["D",1]],
       "D":[["C",1],["A",10]]}
POSES = {"A":(0.0,0.0,0.0),
         "B":(1.0,0.0,0.0),
         "C":(2.0,0.0,0.0),
         "D":(2.0,1.0,0.0)}  # <‑‑ update for your map
# --------------------------------------------------------

class DijkstraPlanner(Node):
    def __init__(self):
        super().__init__("dijkstra_planner")
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()     # blocks until Nav2 lifecycle is ACTIVE :contentReference[oaicite:5]{index=5}
        self.plan_and_go()

    def pose(self, nid):
        x,y,th = POSES[nid]
        ps=PoseStamped()
        ps.header.frame_id="map"
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.w = 1.0              # yaw 0 for demo
        return ps

    def plan_and_go(self):
        result = dijk("A", ADJ)["D"]
        self.get_logger().info(f"Cost to D: {result['Distance']}")

        path = Path()
        path.header.frame_id = "map"
        path.poses = [self.pose("A")] + [self.pose(p[1]) for p in result["Path"]] + [self.pose("D")]

        self.navigator.followPath(path)          # sends a FollowPath action to Nav2
        while not self.navigator.isTaskComplete():
            rclpy.sleep(0.1)
        self.get_logger().info("Task finished")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    DijkstraPlanner()
    rclpy.spin()
