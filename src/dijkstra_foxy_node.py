#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
import time

# Dijkstra functions
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

# Graph and poses
ADJ = {"A":[["B",1],["D",10]],
       "B":[["A",1],["C",1]],
       "C":[["B",1],["D",1]],
       "D":[["C",1],["A",10]]}
POSES = {"A":(0.0,0.0,0.0),
         "B":(1.0,0.0,0.0),
         "C":(2.0,0.0,0.0),
         "D":(2.0,1.0,0.0)}  # Update for actual map

class DijkstraPlannerFoxy(Node):
    def __init__(self):
        super().__init__('dijkstra_planner_foxy')
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self.get_logger().info("Waiting for 'follow_path' action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("'follow_path' action server available.")

    def pose(self, nid):
        if nid not in POSES:
            self.get_logger().error(f"Node {nid} not in POSES")
            return None
        x, y, th = POSES[nid]
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0  # no rotation, yaw=0
        return ps

    def plan_and_go(self):
        result = dijk("A", ADJ)["D"]
        self.get_logger().info(f"Cost to D: {result['Distance']}")

        path = Path()
        path.header.frame_id = "map"
        poses_list = [self.pose("A")] + [self.pose(p[1]) for p in result["Path"]] + [self.pose("D")]
        path.poses = [p for p in poses_list if p is not None]

        self.send_path_goal(path)

    def send_path_goal(self, path):
        goal_msg = FollowPath.Goal()
        goal_msg.path = path

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal completed')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    planner = DijkstraPlannerFoxy()
    planner.plan_and_go()
    rclpy.spin()

if __name__ == '__main__':
    main()
