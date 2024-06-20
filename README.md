# ENAE681: Engineering Optimization Final Project
This is a final project completed for Engineering Optimization at the University of Maryland

### Overview
The goal of this work was to directly apply optimization algorithm to an engineering problem of interest. In this work, I used off-the-shelf optimization algorithm to plan a minimum-travel time path for a rover traversing a hilly terrain. The complete report and complementary presentations are found in "Finalized report and presentation."

### Methodology
I first consider the problem of minimum-time path planning for a four-wheeled rover traveling over a hilly terrain. The rover's equations of motion are first built from first principles and integrated to get the total travel time as a function of path. This integration is performed numerically by solving the line integral over the path. Finally, 5 different optimizers are compared for 1) their ability to find an optimal solution in the space and 2) their solution times. Discrete points along the line are the design variables that the optimizer reasons over. The path is then formed by linearly connecting the points of the line. 

![main_fig_with_wheel](https://github.com/JMocklerUMD/ENAE681-Final-Project/assets/150191399/5186ad6e-4e6d-48c9-bef8-b1b7b73cb2a2)
*Fig. 1 - Simplified rover model showing the primary forces acting on the body. The forces are then used to derive the equations of motion*

### Results
The optimal results under the 5 different solvers are compared below. Because the objective is smooth and closely convex, the gradient-based solvers find solutions as effectively as the nonlinear, gradient-free solvers. 
![first_pass_fig_redo jpg](https://github.com/JMocklerUMD/ENAE681-Final-Project/assets/150191399/1343db89-572f-4ce4-83d8-1d240d7953ec)
*Fig. 2 - A comparison of the optimal path found by the different solvers. Each solver finds an almost identical path through the space*

For deeper intuition in the solution process, the intermediate solutions of the unconstrained, quasi-Newton solver are plotted below. As the solver marches towards the optimum, the line shifts to find a flat path that attempts to avoid sharp changes in height. ![intermediate_pts](https://github.com/JMocklerUMD/ENAE681-Final-Project/assets/150191399/6a2f790e-951f-46a6-8ca2-f12ac5e7d2a3)
*Fig. 2 - A few intermediate solutions from an unconstrained, quasi-Newton solver as the optimizer approaches the minimum-time path*
