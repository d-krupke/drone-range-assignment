from .instance import Instance, Agent, Drone, Point, Terminal
import typing
import networkx as nx


class Solution:
    def __init__(
        self,
        instance: Instance,
        power: typing.Dict[Agent, float],
        positions: typing.Dict[Drone, Point],
        lb: float = 0.0,
    ) -> None:
        self.instance = instance
        self.power = power
        self.positions = positions
        self.lb = lb

    def objective(self) -> float:
        return sum(self.power.values())

    def get_position(self, agent: Agent) -> Point:
        if isinstance(agent, Drone):
            return self.positions[agent]
        assert isinstance(agent, Terminal)
        return agent.position

    def get_range(self, agent: Agent) -> float:
        return self.power[agent] ** 0.5

    def get_power(self, agent: Agent) -> float:
        return self.power[agent]

    def get_topology(self, eps=0.001) -> nx.DiGraph:
        topology = nx.DiGraph()
        topology.add_nodes_from(self.instance.items())
        for v in self.instance.items():
            r = self.get_range(v) + eps
            for w in self.instance.items():
                if v == w:
                    continue
                if self.get_position(v).distance(self.get_position(w)) <= r:
                    topology.add_edge(v, w)
        return topology

    def is_feasible(self, eps=0.001):
        # solution is feasible if terminals are strongly connected
        topology = self.get_topology(eps)
        for t in self.instance.terminals:
            reachable = list(nx.dfs_preorder_nodes(topology, t))
            if any(w not in reachable for w in self.instance.terminals):
                print(t, reachable)
                return False
        return True
