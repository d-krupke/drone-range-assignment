
import typing
import itertools
import math


class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __hash__(self):
        return hash((self.x, self.y))

    def __eq__(self, other):
        if not isinstance(other, Point):
            return False
        return self.x == other.x and self.y == other.y

    def distance(self, other: "Point") -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    def __str__(self) -> str:
        return "(" + str(self.x) + ", " + str(self.y) + ")"

    def __repr__(self) -> str:
        return str(self)


class Agent:
    def is_terminal(self) -> bool:
        return False

    def is_drone(self) -> bool:
        return False

    def __repr__(self) -> str:
        return str(self)


class Terminal(Agent):
    def __init__(self, index: int, position: Point):
        self.index = index
        self.position = position

    def __hash__(self) -> int:
        return hash((self.index, self.position))

    def __eq__(self, other: typing.Any) -> bool:
        if not isinstance(other, Terminal):
            return False
        return self.index == other.index and self.position == other.position

    def distance(self, other: typing.Union["Terminal", Point]) -> float:
        if isinstance(other, Terminal):
            return self.position.distance(other.position)
        else:
            return self.position.distance(other)

    def __str__(self) -> str:
        return "Terminal(" + str(self.index) + ", " + str(self.position) + ")"

    def is_terminal(self) -> bool:
        return True


class Drone(Agent):
    def __init__(self, index: int):
        self.index = index

    def __hash__(self) -> int:
        return hash(self.index)

    def __eq__(self, other: typing.Any) -> bool:
        if not isinstance(other, Drone):
            return False
        return self.index == other.index

    def __str__(self) -> str:
        return "Drone(" + str(self.index) + ")"

    def is_drone(self) -> bool:
        return True


class Instance:
    def __init__(self, terminals: typing.List[Point], num_drones: int) -> None:
        self.terminals = [Terminal(i, p) for i, p in enumerate(terminals)]
        self.drones = [Drone(i) for i in range(num_drones)]

    def get_num_terminals(self) -> int:
        return len(self.terminals)

    def bounding_box(
        self,
    ) -> typing.Tuple[typing.Tuple[float, float], typing.Tuple[float, float]]:
        min_x = min(t.position.x for t in self.terminals)
        min_y = min(t.position.y for t in self.terminals)
        max_x = max(t.position.x for t in self.terminals)
        max_y = max(t.position.y for t in self.terminals)
        return (min_x, min_y), (max_x, max_y)

    def terminal_indices(self) -> typing.List[int]:
        return list(range(self.get_num_terminals()))

    def items(self) -> typing.List[typing.Union[Terminal, Drone]]:
        return self.terminals + self.drones
    
    def diameter(self):
        return max(
            t1.distance(t2)
            for t1, t2 in itertools.combinations(self.terminals, 2)
        )