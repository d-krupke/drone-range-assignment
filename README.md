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

1. **Terminals:** A predefined set of terminal nodes positioned at fixed locations within a 2D plane.
2. **Drones:** An allocation of \( k \) drones available for strategic placement within the plane.

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
