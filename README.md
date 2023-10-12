# Range Assignment Optimizer for Networks with Drones

_Dominik Krupke, 2023_

This repository contains experimental code for an exact solver for a range assignment problem with drones.
This problem was proposed by Aaron Becker.
The problem definition may differ slightly from the original problem statement.

![example](./docs/figures/example.png)

## Problem Description

**Objective:**
Determine the optimal placement of drones and range assignments for both terminals and drones to ensure a strongly connected network among the terminals, with the goal of minimizing the overall power consumption.

**Input:**

1. **Terminals:** A predefined set of terminal nodes $T$ positioned at fixed locations within a 2D plane.
2. **Drones:** An allocation of \( k \) drones $D$ available for strategic placement within the plane.

$A=T\cup D$ is the set of all agents.

**Constraints:**

1. **Strong Connectivity:** The network configuration must guarantee a directed path between any pair of terminal nodes, whether through direct connections or intermediated by drones.
2. **Range-based Connectivity:** One node is directly connected to another if their distance does not exceed the range of the originating node.
3. **Power Consumption Model:** A node's power consumption is directly proportional to the square of its transmission range.

**Optimization Metric:**
Minimize the collective power consumption, which is represented by the sum of the squared ranges of all terminals and drones in the network.

## Insights

The problem can be efficiently solved via a Second Order Cone Program, if the topology of the network is known.

## Solver

Currently, there is only a rather inefficient solver that uses a lot of Big-M to compute the topology.
There are many opportunities for improvement.

### Model

**Variables:**

* $p_i=(x_i, y_i) \in \mathbb{R}^2$: Position of drone or terminal $i\in A$. For terminals $T$, this is fixed.
* $r_i \in \mathbb{R}$: Squared range (power) of terminal or drone $i$.
* $d_{ij} \in \mathbb{R}$: Squared distance between terminal or drone $i$ and terminal or drone $j$.
* $z_{ij} \in \{0, 1\}$: Binary variable indicating whether terminal $i$ is (directly) connected to terminal $j$.

**Constraints:**

* $d_{ij} \geq squared_dist(p_i, p_j)$ for all $i, j \in A$. This can be implemented via a Second Order Cone Constraint using some auxiliary variables. Second Order Cone Constraints can be solved efficiently.
* $r_i \geq d_{ij} - (1-z_{ij})\cdot M$ for all $i, j \in A$. This enforces the necessary power to establish the connection if the solver decides to connect $i$ to $j$. $M$ is a large constant such that $r_i \geq d_{ij}$ if $z_{ij}=1$. This is an application of the Big-M method and the most critical part of the model. $M$ can be upper bounded by the maximal squared distance.
* $\sum_{i \in A', j \in A\setminus A'} z_{ij} \geq 1$ for all $A'\subset A, T\cap A\not=\emptyset, T\cap A\setminus A'\not= \emptycap$. This ensures a strongly connected network and is implemented via callbacks.

**Objective:**

Sum of all power assignments $\sum_{i\in A} r_i$

#### Potential Improvements

There are already some smaller improvements implements, e.g.,:

* Lowering the big-M constants between terminals, as we now the maximum necessary range between terminals.
* Enforcing every terminal to have at least one incoming and one outgoing connection.
* Enforcing at least $|T|$ edges, if there is more than one terminal.

Further improvements could be:

* Using lower bounds on the power of individual agents.
* Using better upper bounds on the power of individual agents, allowing to reduce the big-M constants.
* Adding additional constraint to enforce the network to be connected.
* Only using the SOCP and implementing the branch and bound yourself. This would require a smart way to iteratively build the network, such that every subnetwork is a lower bound on the overall network.

## Installation

You can install the package via

```bash
pip install .
```

See the notebooks-folder for examples.