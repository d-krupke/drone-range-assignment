import matplotlib.pyplot as plt
from .solution import Solution


def plot(ax: plt.Axes, solution: Solution, eps=0.001):
    # plot terminals
    instance = solution.instance
    for t in instance.terminals:
        ax.plot([t.position.x], [t.position.y], "ks")
        ax.add_artist(
            plt.Circle(
                [t.position.x, t.position.y],
                solution.get_range(t),
                color="blue",
                fill=True,
                alpha=0.1,
            )
        )
    # plot drones
    for d in instance.drones:
        p = solution.get_position(d)
        ax.plot([p.x], [p.y], "o", color="blue")
        ax.add_artist(
            plt.Circle(
                [p.x, p.y], solution.get_range(d), color="blue", fill=True, alpha=0.1
            )
        )
    # plot arcs
    topology = solution.get_topology(eps)
    for v, w in topology.edges():
        v_pos = solution.get_position(v)
        w_pos = solution.get_position(w)
        ax.arrow(
            v_pos.x,
            v_pos.y,
            w_pos.x - v_pos.x,
            w_pos.y - v_pos.y,
            color="grey",
            head_width=0.5,
            length_includes_head=True,
            lw=0.5,
        )
    ax.set_aspect("equal", adjustable="box")
    # plt.xlim(-10, 30)
    # plt.ylim(-10, 30)
    min_x = min(t.position.x - solution.get_range(t) for t in instance.terminals)
    min_y = min(t.position.y - solution.get_range(t) for t in instance.terminals)
    max_x = max(t.position.x + solution.get_range(t) for t in instance.terminals)
    max_y = max(t.position.y + solution.get_range(t) for t in instance.terminals)
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
