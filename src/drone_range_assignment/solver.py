import gurobipy as gp
from gurobipy import GRB
import typing
import itertools
import networkx as nx
from .instance import Point, Agent, Terminal, Drone, Instance
from .solution import Solution


class PositionVars:
    def __init__(self, model: gp.Model, instance: Instance) -> None:
        self.model = model
        self.instance = instance
        (min_x, min_y), (max_x, max_y) = instance.bounding_box()
        self._x: typing.Dict[Agent, typing.Any] = {
            d: model.addVar(
                lb=min_x, ub=max_x, vtype=GRB.CONTINUOUS, name=f"x_{d.index}"
            )
            for d in instance.drones
        }
        self._x.update({t: t.position.x for t in instance.terminals})
        self._y: typing.Dict[Agent, typing.Any] = {
            d: model.addVar(
                lb=min_y, ub=max_y, vtype=GRB.CONTINUOUS, name=f"y_{d.index}"
            )
            for d in instance.drones
        }
        self._y.update({t: t.position.y for t in instance.terminals})

    def x(self, agent: Agent, resolve: bool = False) -> typing.Any:
        if resolve:
            return self._x[agent].x
        return self._x[agent]

    def y(self, agent: Agent, resolve: bool = False) -> typing.Any:
        if resolve:
            return self._y[agent].x
        return self._y[agent]

    def get_positions(self) -> typing.Dict[Drone, Point]:
        return {
            d: Point(self.x(d, True), self.y(d, True)) for d in self.instance.drones
        }


class DiffVars:
    """
    Differences between positions. Needed for computing the distance between
    two agents. The difference between two terminals is a constant and is
    never negative. The other differences are variables and can be negative.
    The square sum in the SOCP formulation does not care about the sign and
    it would be expensive to compute the absolute value of the difference.
    """

    def __init__(self, positions: PositionVars) -> None:
        self.model = positions.model
        self.instance = positions.instance
        agents = positions.instance.items()
        (min_x, min_y), (max_x, max_y) = positions.instance.bounding_box()
        max_diff = max(max_x - min_x, max_y - min_y)
        self._x_vars: typing.Dict[typing.Tuple[Agent, Agent], typing.Any] = {
            (a, b): positions.model.addVar(
                lb=-max_diff, ub=max_diff, vtype=GRB.CONTINUOUS
            )
            for a, b in itertools.combinations(agents, 2)
            if self.is_variable(a, b)
        }
        for (a, b), v in self._x_vars.items():
            positions.model.addConstr(v == positions.x(a) - positions.x(b))
        self._y_vars: typing.Dict[typing.Tuple[Agent, Agent], typing.Any] = {
            (a, b): positions.model.addVar(
                lb=-max_diff, ub=max_diff, vtype=GRB.CONTINUOUS
            )
            for a, b in itertools.combinations(agents, 2)
            if self.is_variable(a, b)
        }
        for (a, b), v in self._y_vars.items():
            positions.model.addConstr(v == positions.y(a) - positions.y(b))

    def x_diff(self, a: Agent, b: Agent) -> typing.Any:
        if self.is_variable(a, b):
            if (a, b) in self._x_vars:
                return self._x_vars[(a, b)]
            return self._x_vars[(b, a)]
        assert isinstance(a, Terminal) and isinstance(b, Terminal)
        return abs(a.position.x - b.position.x)

    def y_diff(self, a: Agent, b: Agent) -> typing.Any:
        if self.is_variable(a, b):
            if (a, b) in self._y_vars:
                return self._y_vars[(a, b)]
            return self._y_vars[(b, a)]
        assert isinstance(a, Terminal) and isinstance(b, Terminal)
        return abs(a.position.y - b.position.y)

    def is_variable(self, a: Agent, b: Agent) -> bool:
        return a.is_drone() or b.is_drone()


class EdgeVars:
    def __init__(self, model: gp.Model, instance: Instance) -> None:
        self.model = model
        self.instance = instance
        self._vars: typing.Dict[typing.Tuple[Agent, Agent], typing.Any] = {
            (v, w): model.addVar(vtype=GRB.BINARY, name=f"e_{v}_{w}")
            for v, w in itertools.permutations(instance.items(), 2)
        }
        # trivial constraint to get a better start
        if len(instance.terminals) > 1:
            self.model.addConstr(
                sum(
                    self._vars[(v, w)]
                    for v in instance.terminals
                    for w in instance.items()
                    if v != w
                )
                >= len(instance.terminals)
            )

    def used(self, a: Agent, b: Agent) -> typing.Any:
        return self._vars[(a, b)]

    def get_graph(self, in_callback: bool = False) -> nx.DiGraph:
        G = nx.DiGraph()
        G.add_nodes_from(self.instance.items())
        for (v, w), var in self._vars.items():
            if in_callback:
                val = self.model.cbGetSolution(var) > 0.5
            else:
                val = var.x > 0.5
            if val:
                G.add_edge(v, w)
        return G

    def lazily_enforce_strongly_connected(self) -> int:
        # enforce strongly connected graph
        G = self.get_graph(in_callback=True)
        add_constraints = 0
        for t in self.instance.terminals:
            reachable_comp = list(nx.dfs_preorder_nodes(G, t))
            assert t in reachable_comp
            if any(t_ not in reachable_comp for t_ in self.instance.terminals):
                # not all terminals are reachable
                self.model.cbLazy(
                    sum(
                        self._vars[(v, w)]
                        for v in reachable_comp
                        for w in self.instance.items()
                        if w not in reachable_comp
                    )
                    >= 1
                )
                add_constraints += 1
        return add_constraints

    def items(self):
        yield from self._vars.items()


class SquaredDistVars:
    def __init__(self, diffs: DiffVars) -> None:
        self.model = diffs.model
        self.instance = diffs.instance
        self._vars: typing.Dict[typing.Tuple[Agent, Agent], typing.Any] = {
            (a, b): self.model.addVar(lb=0.0, vtype=GRB.CONTINUOUS)
            for a, b in itertools.combinations(self.instance.items(), 2)
            if diffs.is_variable(a, b)
        }
        for (a, b), v in self._vars.items():
            x_diff = diffs.x_diff(a, b)
            y_diff = diffs.y_diff(a, b)
            # SOCP of form a**2 + b**2 <= c
            self.model.addQConstr(x_diff * x_diff + y_diff * y_diff <= v)

    def ub(self, a: Agent, b: Agent) -> typing.Any:
        if isinstance(a, Terminal) and isinstance(b, Terminal):
            return a.distance(b)
        if (a, b) in self._vars:
            return self._vars[(a, b)]
        return self._vars[(b, a)]

    def get_distance_assignments(self):
        distances = {(a, b): v.x for (a, b), v in self._vars.items()}
        distances.update(
            {
                (a, b): a.distance(b) ** 2
                for a, b in itertools.combinations(self.instance.terminals, 2)
            }
        )
        return distances


class PowerVars:
    def __init__(self, sqdist: SquaredDistVars, edge_vars: EdgeVars) -> None:
        self.instance = sqdist.instance
        self.model = sqdist.model
        max_power = self.instance.diameter() ** 2
        self._vars: typing.Dict[Agent, typing.Any] = {
            n: self.model.addVar(
                lb=0.0, ub=max_power, vtype=GRB.CONTINUOUS, name=f"r_{n}"
            )
            for n in self.instance.items()
        }
        # enforce minimal power based on topology
        for (a, b), var in edge_vars.items():
            if isinstance(a, Terminal) and isinstance(b, Terminal):
                # fixed distance allows for a tighter bound
                self.model.addConstr(self._vars[a] >= var * (a.distance(b) ** 2))
            else:
                # disable constraint if edge is not used
                self.model.addConstr(
                    self._vars[a]
                    >= sqdist.ub(a, b) - (1 - edge_vars.used(a, b)) * max_power
                )

    def power(self, a: Agent) -> typing.Any:
        return self._vars[a]

    def get_power_assignments(self) -> typing.Dict[Agent, float]:
        return {a: self._vars[a].x for a in self.instance.items()}


class Solver:
    def __init__(self, instance: Instance) -> None:
        self.model = gp.Model()
        self.instance = instance
        self.positions = PositionVars(self.model, instance)
        self.diffs = DiffVars(self.positions)
        self.sqdist = SquaredDistVars(self.diffs)
        self.edges = EdgeVars(self.model, instance)
        self.power = PowerVars(self.sqdist, self.edges)
        self.model.setObjective(
            sum(self.power.power(a) for a in self.instance.items()), GRB.MINIMIZE
        )

    def solve(self, timelimit: float = 60.0) -> typing.Optional[Solution]:
        def callback(model: gp.Model, where: int) -> None:
            if where == GRB.Callback.MIPSOL:
                self.edges.lazily_enforce_strongly_connected()

        self.model.setParam("LazyConstraints", 1)
        self.model.setParam("TimeLimit", timelimit)
        self.model.optimize(callback)
        lb = self.model.ObjBound
        if self.model.numSolutions > 0:
            solution = Solution(
                self.instance,
                self.power.get_power_assignments(),
                self.positions.get_positions(),
                lb,
            )
            return solution
        return None
