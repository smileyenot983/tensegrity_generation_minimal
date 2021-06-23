clc;
close all;
clear all;
cvx_clear;
cvx_solver Gurobi_2;


seed = 1;
n_nodes = 20;

strut_constraint = false;
strut_max_length = 15;

cable_constraint = false;
cable_max_length = 15;

projection_constraint = false;
projection_axis = 'x';
projection_max_length = 15;

filename = 'random_filename';
plot_title = 'random_plot_title';

sol = run_experiment(seed,n_nodes,strut_constraint,strut_max_length,cable_constraint,cable_max_length,...
                projection_constraint,projection_axis,projection_max_length);

visualize_2(sol,3,filename,plot_title);
            
     

