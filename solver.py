import threading

import numpy as np


class Solver:
    def __init__(self, maze) -> None:
        self.Maze = maze

        # Entry points are values where the Maze is 2
        self.EntryPoints = [(x, y) for x, y in np.array(np.where(self.Maze == 2)).transpose()]

        # Exit points are values where the Maze is 3
        self.ExitPoints = [(x, y) for x, y in np.array(np.where(self.Maze == 3)).transpose()]

        # Traversable points are values where the Maze is 1
        self.Traversable = [(x, y) for x, y in np.array(np.where(self.Maze == 1)).transpose()]
        print(len(self.Traversable))
        # The following loop is implemented to reduce the complexity of the path finding algorithm by removing the traversal points leading to the wall
        while True:
            c = True
            for point in self.Traversable:
                if self.IsTraversable(point) and self.getTraversableAdjacentCount(point) <= 1:
                    self.setAsWall(point)
                    c = False
            if c:
                self.Traversable = [(x, y) for x, y in np.array(np.where(self.Maze == 1)).transpose()]
                break
        self.solution = [[], []]

        print(len(self.Traversable))
        quit()

    def getTraversableAdjacentCount(self, point) -> int:
        return [self.IsTraversable(p) for p in getAdjacentPoints(point)].count(True)

    def IsTraversable(self, point) -> bool:
        return self.Maze[point[0]][point[1]] > 0

    def setAsWall(self, point) -> None:
        self.Maze[point[0]][point[1]] = 0

    def parse(self) -> None:
        """
        Parses the Maze into data structures which may help in solving the maze.
        """
        pass

    def solve(self) -> bool:
        """
        Returns True if a solution was found, False otherwise.
        A valid solution is a complete set of paths from each entry point
        to a distinct exit point where no two paths intersect with each other
        and no path intersects with a wall.
        A path is an ordered list of adjacent Traversable points.
        """
        for i in range(len(self.EntryPoints)):
            for j in range(len(self.ExitPoints)):
                path_finder1 = PathFinder(self.Maze, self.Traversable, self.EntryPoints[i], self.ExitPoints[j])
                if path_finder1.Dijkstra():
                    sol = path_finder1.Solution
                    forbidden_points = sol[1:len(sol) - 1]
                    path_finder2 = PathFinder(self.Maze, self.Traversable, self.EntryPoints[1 - i], self.ExitPoints[1 - j], forbidden_points)
                    if path_finder2.Dijkstra():
                        self.solution = [path_finder1.Solution, path_finder2.Solution]
                        return True
        for i in range(len(self.EntryPoints)):
            for j in range(len(self.ExitPoints)):
                path_finder1 = PathFinder(self.Maze, self.Traversable, self.EntryPoints[i], self.ExitPoints[j])
                Solutions = []
                path_finder1.Stop = False
                path_finder1.Condition = threading.Condition()
                t=threading.Thread(target=path_finder1.ReccursiveAlgorithm, args=(self.EntryPoints[i], self.ExitPoints[j], [], Solutions,))
                t.start()
                index = 0
                while t.is_alive() or index < len(Solutions):
                    with path_finder1.Condition:
                        if index >= len(Solutions):
                            path_finder1.Condition.wait()
                    sol = Solutions[index]
                    forbidden_points = sol[1:len(sol) - 1]
                    path_finder2 = PathFinder(self.Maze, self.Traversable, self.EntryPoints[1 - i], self.ExitPoints[1 - j], forbidden_points)
                    if path_finder2.Dijkstra():
                        self.solution = [sol, path_finder2.Solution]
                        path_finder1.Stop = True
                        t.join()
                        return True
                    index += 1
        return False


class PathFinder:
    def __init__(self, maze, traversable, entry_point, exit_point, forbidden_points=None) -> None:
        self.Maze = maze
        self.Traversable = traversable
        self.EntryPoint = entry_point
        self.ExitPoint = exit_point
        self.ForbiddenPoints = forbidden_points
        self.IndexesMatrix = [[0 for _ in range(len(self.Maze[i]))] for i in range(len(self.Maze))]
        for index, p in enumerate(self.Traversable):
            self.IndexesMatrix[p[0]][p[1]] = index + 2
        self.IndexesMatrix[self.ExitPoint[0]][self.ExitPoint[1]] = 1

    def Dijkstra(self) -> bool:
        """
        A feasible path is sought for using Dijkstra
        IF the first path is found already, it is considered as forbidden for the second path search
        :return:
        """
        Length = len(self.Traversable) + 2
        previous = [i for i in range(Length)]
        self.Labels = [0 if i == 0 else float('inf') for i in range(Length)]
        self.VisitedPoints = [False for _ in range(Length)]
        if self.ForbiddenPoints is not None:
            for forbidden_point in self.ForbiddenPoints:
                self.VisitedPoints[self.getIndex(forbidden_point)] = True
        self.Solution = None
        while True:
            min_index = self.FindMinIndex()
            if min_index < 0:
                break
            if min_index == 1:
                sol = []
                index = min_index
                while index != previous[index]:
                    sol.insert(0, self.ExitPoint if index == 1 else self.Traversable[index - 2])
                    index = previous[index]
                if index == 0:
                    sol.insert(0, self.EntryPoint)
                    self.Solution = sol
                break
            else:
                self.VisitedPoints[min_index] = True
                p_min = self.EntryPoint if min_index == 0 else self.Traversable[min_index - 2]
                if self.Labels[1] == float('inf') or self.Labels[min_index] + getDistance(p_min, self.ExitPoint) < \
                        self.Labels[1]:
                    for p in getAdjacentPoints(p_min):
                        index = self.getIndex(p)
                        if index > 0 and (min_index != 0 or index != 1) and not self.VisitedPoints[index]\
                                and self.Labels[min_index] + 1 < self.Labels[index]:
                            self.Labels[index] = self.Labels[min_index] + 1
                            previous[index] = min_index
        return self.Solution is not None

    def FindMinIndex(self) -> int:
        Min = float('inf')
        index = -1
        for i, label in enumerate(self.Labels):
            if label < Min and not self.VisitedPoints[i]:
                Min = label
                index = i
        return index

    def ReccursiveAlgorithm(self, new_point, exit_point, black_list, Solutions):
        if self.Stop:
            return
        new_black_list = []
        for p in black_list:
            if areEquals(p, new_point):
                return
            new_black_list.append(p)
        new_black_list.append(new_point)
        if areEquals(exit_point, new_point):
            Solutions.append(new_black_list)
            with self.Condition:
                self.Condition.notify()
            return
        for p in getAdjacentPoints(new_point):
            if self.getIndex(p) > 0:
                self.ReccursiveAlgorithm(p, exit_point, new_black_list, Solutions)

    def getIndex(self, point) -> int:
        return self.IndexesMatrix[point[0]][point[1]] if point[0] < len(self.IndexesMatrix) and point[1] < len(
            self.IndexesMatrix[point[0]]) else 0


def getDistance(point1, point2) -> int:
    return abs(point1[0] - point2[0]) + abs(point1[1] - point2[1])


def areEquals(point1, point2) -> bool:
    return getDistance(point1, point2) == 0


def getAdjacentPoints(point):
    return [(point[0], point[1] + 1), (point[0], point[1] - 1), (point[0] + 1, point[1]), (point[0] - 1, point[1])]
