# Description
This code does convex optimization based tensegrity generation, with several interesting constraints. 

# Code 
1. experiment_runner.m - main file, runs all experiment and here you can tune parameters(total number of nodes, whether to use certain constraint or not etc)
2. run_experiment.m - runs the whole optimization procedure
3. visualize_2.m - does the visualization and saves resulting structure

+ several other files, which are used in visualization only

# Requirements
The code does MICP optimization, thus some framework which will do all the optimization is needed. In this work we are using Gurobi_2.

# Running
Just run the experiment_runner.m 
