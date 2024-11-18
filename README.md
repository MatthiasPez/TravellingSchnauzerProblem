# The Travelling Schnauzer Problem

A Schnauzer has a set of favorite spots at the park. Every time it goes for a walk on the leash of its owner, it will refuse to go home before visiting all its favorite spots. The dog runs faster than the owner and the leash has a limited length. Which is the optimal trajectory for the dog and the owner that minimizes the time of their walk?

From the mathematical point of view, this everyday problem is the same as the engineeristic problem where a fast vehicle (also referred to as mission vehicle) has to visit a set of points of interest while remaining within a certain distance of a slow support vehicle (also referred to as base station). This problem arises for instance in searching or monitoring tasks in ocean applications, where the fast vehicle is a UAV or AUV and the slow vehicle is a support ship. 

Interestingly, the optimal trajectories of the vehicles are characterized only by the positions of the slow vehicle when the fast vehicle reaches and departs from each point of interest. Based on this structural property of the optimal trajectory, we can formalize the Travelling Schnauzer Problem as the Mixed-Integer Second-Order Cone Programming problem
```math
\begin{aligned}
&\min_{x,t,q,u} \sum\nolimits_{i=1}^{n-1} t_i \\ 
&\sum\nolimits_{i=1}^{n}q_{1i}=\sum\nolimits_{j=1}^{n}q_{jn}=1 \\
&\sum\nolimits_{i=1}^{n}q_{i1}=\sum\nolimits_{j=1}^{n}q_{nj}=0 \\
&\sum\nolimits_{i=1}^{n}q_{ik}=\sum\nolimits_{j=1}^{n}q_{kj} = 1 \quad \forall k=2, \dots, n-1 \\
&2 \leq u_{i} \leq n \quad \forall i=2, \dots, n \\
&u_i - u_j + 1 \leq (n-1) (1-q_{ij}) \quad \forall i,j =2, \dots, n \\
&\| \mathbf{x_{b2,i}}-\mathbf{x_{b1,i}} \| \leq v_b t_{p,i}  \quad \forall i = 1 \dots n \\
&\| \mathbf{p_i}-\mathbf{x_{b1,i}} \| \leq r \quad \forall i = 1 \dots n, \\
&\| \mathbf{p_i}-\mathbf{x_{b2,i}} \| \leq r \quad \forall i = 1 \dots n. \\
&(1-q_{ij}) \| \mathbf{p_i}-\mathbf{p_{j}} \| \leq v_v t_i  \quad \forall i,j = 1 \dots n, \\
&(1-q_{ij}) \| \mathbf{x_{b2,i}}-\mathbf{x_{b1,j}} \| \leq v_b t_i \quad \forall i,j = 1 \dots n
\end{aligned}
```
where $p_i$ is the point $i$ to visit, $n$ is the total number of points to visit, $\mathbf{x_{b1,i}}$ and $\mathbf{x_{b2,i}}$ are the positions of the base at the vehicle's arrival and departure from point $i$, $t_i$ is the travel time between the vehicle's departure from point $i$ and the arrival at point $i+1$, $q_{ij}$ is the selection variable for the link between point $i$ and point $j$, $u_i$ is the visiting order of the point $i$, $t_{p,i}$ is the visiting time of point $i$, $v_b$ is the maximum speed of the base station, $v_v$ is the maximum speed of the mission vehicle, and $r$ is the maximum distance. 

In order to overcome the complexity of the MISOCP problem, we propose a fast heuristic to find a suboptimal solution in a small amount of time. The proposed heuristic consists of two stages: first, a Euclidean TSP problem is solved to obtain the path of the fast vehicle and the associated visiting order; then, a simple SOCP problem is solved to obtain the optimal trajectory of the base for the ordered sequence of points. This heuristic is fast (instances of hundreds of points can be solved in a few seconds using efficient E-TSP solvers) and efficient (overall mission time is increased by less than 1% in many practical cases). 

This repository contains the MATLAB functions to get the optimal solution of the Travelling Schnauzer Problem (using solvetschp.m) and to get a fast and efficient sub-optimal solution with the proposed heuristic (using solvetschphh.m or using first solvetschphh_tsp.m and then solvetschphh_socp.m). A function to get an improved sub-optimal solution with a ruin-and-recreate approach (using solvetschprr.m) and an executable script (main.m) to test the algorithms are also provided. 
