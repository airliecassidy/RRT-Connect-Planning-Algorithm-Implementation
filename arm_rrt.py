import numpy as np
from tqdm import tqdm
from scipy.spatial import cKDTree
import random
import argparse
import robots


class Node:
    def __init__(self, point, parent=None):
        self.point = np.array(point)
        self.parent = parent


class Tree:
    def __init__(self, node_list, kdtree):
        self.node_list = node_list
        self.kdtree = kdtree

    def add(self, new_point):
        self.node_list.append(new_point)

        if len(self.node_list) % 50 == 0:
            data = [n.point for n in self.node_list]
            self.kdtree = cKDTree(data)


class RRT:
    def __init__(self, step_size=0.1, max_iter=20, env_name="Free"):
        self.controller = robots.Gen3LiteArmController(env_name=env_name)

        self.start = Node(self.controller.getCurrentJointAngles())
        self.goal = Node(self.controller.goal_angles)
        self.rand_ranges = self.controller.getRanges()
        self.step_size = step_size
        self.max_iter = max_iter
        self.start_tree = Tree([self.start], cKDTree([self.start.point]))
        self.goal_tree = Tree([self.goal], cKDTree([self.goal.point]))
        self.path_to_goal = []

    # This a "stub" planning function. It adds random nodes always
    # to the root of a start and goal tree. It shows many of the
    # syntax elements and helper functions you could use, but
    # you will have to fix and extend this to make it an RRT or
    # RRT-Connect planner that can solve the harder environments.
    def plan(self):

        """
        This RRT-Connect algorithm implementation follows LaValle, Planning Algorithms.

        The algorithm alternates between growing two trees (one from the start,
        one from the goal) and attempts to connect them. This bidirectional approach
        significantly improves efficiency over single-tree RRT.

        """

        # Add max_iter nodes to each start and goal tree
        for k in tqdm(range(self.max_iter)):
            q_rand = self.sample()
            extension_result = self.extend_tree(self, start_tree, q_rand)

            if extension_result is not None:
                connection_node = self.connect_trees(self.goal_tree, extension_result.point)
                if connection_node is not None:
                    self.path_to_goal = self.extract_path(extension_result, connection_node)
                    return True
            self.start_tree, self.goal_tree = self.goal_tree, self.start_tree
        return False

    def extend_tree(self, tree, q_target):
        """
        Extend operation which attempts to grow the tree one step towards q_target

        """
        q_near = self.nearest_node(q_target, tree)
        q_new = self.steer(q_near, q_target)

        if self.collision_free(q_near.point, q_new.point):
            tree.add(q_new)
            return q_new
        return None

    def connect_trees(self, tree, q_target):
        """
        Repeatedly extending towards torward target until reaching q_target or hitting
        an obstacle.
        This is the key difference between 'extend' and 'connect' in RRT-Connect. As
        'connect' tries to reach the target in multiple steps, not just one.
        Returns the node that reached q_target if successful, None otherwise.
        """
        while True:
            q_near = self.nearest_node(q_target, tree)

            if np.linalg.norm(q_near.point - q_target) < self.step_size:
                if self.collision_free(q_near.point, q_target):
                    q_final = Node(q_target, q_near)
                    tree.add(q_final)
                    return q_final
                else:
                    return None  # cant connect due to a collision

    def sample(self):
        point = []
        for i in range(0, len(self.rand_ranges[0])):
            point.append(random.uniform(self.rand_ranges[0][i], self.rand_ranges[1][i]))
        return np.array(point)

    def nearest_node(self, point, tree):
        _, idx = tree.kdtree.query(point)
        return tree.node_list[idx]

    def steer(self, from_node, to_point):
        direction = to_point - from_node.point
        distance = np.linalg.norm(direction)

        if distance < self.step_size:
            new_point = to_point
        else:
            direction = direction / distance
            new_point = from_node.point + self.step_size * direction

        new_node = Node(new_point)
        new_node.parent = from_node
        return new_node

    def collision_free(self, p1, p2):
        return self.controller.collision_free(p1, p2)

    def reached_goal(self, node, goal=None):
        if goal is None:
            goal = self.goal

        return np.linalg.norm(node.point - goal.point) < self.step_size and self.collision_free(node.point, goal.point)

    def extract_path(self, start_node, goal_node):
        # Build the start-tree path, leaf to root (backwards)
        start_tree_path = []
        while start_node is not None:
            start_tree_path.append(start_node.point)
            start_node = start_node.parent

        # Build the goal-tree path, leaf to root (forwards)
        goal_tree_path = []
        while goal_node is not None:
            goal_tree_path.append(goal_node.point)
            goal_node = goal_node.parent

            # First add the start path, reversing
        overall_path = start_tree_path[::-1]
        # Add the goal path
        overall_path.extend(goal_tree_path)
        return overall_path


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        prog='arm_rrt',
        description='Plans and executes paths for arms around obstacles.')
    parser.add_argument('--filename', default='rrt_path.npy')
    parser.add_argument('-e', '--environment', default='Easiest')
    parser.add_argument('-p', '--plan', action='store_true', default=True)
    parser.add_argument('-r', '--run', action='store_true', default=False)
    args = parser.parse_args()

    rrt = RRT(env_name=args.environment)

    if args.plan:
        success = rrt.plan()

        if success:
            print("Tree planning reached the goal.")
            np.save(args.filename, rrt.path_to_goal)
        else:
            print("Failed to find a path to the goal.")

        rrt.controller.visTreesAndPaths(
            [rrt.start_tree, rrt.goal_tree],
            [rrt.path_to_goal],
            rgbas_in=[[0.5, 0.0, 0.5, 1.0], [0.902, 0.106, 0.714, 1.0]]
        )

    if args.run:
        path_to_goal = np.load(args.filename)
        rrt.controller.execPath(path_to_goal)
