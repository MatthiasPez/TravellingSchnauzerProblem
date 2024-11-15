%%% Test the algorithms to solve the TSchP problem

close all
clear all


%% Parameters
r=40;   % maximum distance between vehicles
t_p=1;  % visiting time (constant)
v_m=60; % maximum velocity of the mission vehicle
v_b=24; % maximum velocity of the base station
n=10;   % number of points of interest
w=500;  % width of the area



%% Generate a set of points of interest (from TSPLib or uniform random variables)
file_name='berlin52'; % name of TSPLib 
p=readTSPLib(file_name,n,w);  % locations of poi from TSPLib

p=unifrnd(0,w,2,n); % random locations of poi
p=[p,p(:,1)];  % p(:,1) is assumed to be both the initial and the final point


%% Get the optimal solution to the TSCHP problem
[xb,xm, q,u,xb_a,xb_d,t, opt_cost,ex_time] = solvetschp(r,v_m,v_b,t_p,p);

figure
hold on
grid on
box on
axis equal
plot(xb(1,:),xb(2,:),'*:b','LineWidth',1.5) % base
plot(xm(1,:),xm(2,:),'o-k','LineWidth',1.5) % vehicle
title('Optimal solution')
legend('Base','Vehicle')


%% Get a suboptimal solution to the TSCHP problem through the ETSP-driven heuristic (step by step)
[xm_tsp, q_tsp,u_tsp, opt_cost_tsp,ex_time_tsp] = solvetschphh_tsp(v_m,p);

figure
hold on
grid on
box on
axis equal
plot(xm_tsp(1,:),xm_tsp(2,:),'o-k','LineWidth',1.5) % vehicle
title('TSP solution')


[xb_socp,xm_socp, xb_a_socp,xb_d_socp,t_socp, opt_cost_socp,ex_time_socp]=solvetschphh_socp(r,v_m,v_b,t_p,xm_tsp);
figure
hold on
grid on
box on
axis equal
plot(xb_socp(1,:),xb_socp(2,:),'*:b','LineWidth',1.5) % base
plot(xm_socp(1,:),xm_socp(2,:),'o-k','LineWidth',1.5) % vehicle
title('Heuristic solution')
legend('Base','Vehicle')


%% Get a suboptimal solution to the TSCHP problem through the ETSP-driven heuristic (step by step)
[xb_hh,xm_hh, q_hh,u_hh,xb_a_hh,xb_d_hh,t_hh, opt_cost_hh,ex_time_hh] = solvetschphh(r,v_m,v_b,t_p,p);

figure
hold on
grid on
box on
axis equal
plot(xb_hh(1,:),xb_hh(2,:),'*:b','LineWidth',1.5) % base
plot(xm_hh(1,:),xm_hh(2,:),'o-k','LineWidth',1.5) % vehicle
title('Heuristic solution')
legend('Base','Vehicle')


%% Get a suboptimal solution to the TSCHP problem through the ETSP-driven heuristic with Ruin-and-Recreate correction
[xb_rr,xm_rr, q_rr,u_rr,xb_a_rr,xb_d_rr,t_rr, opt_cost_rr,ex_time_rr] = solvetschprr(r,v_m,v_b,t_p,p);

figure
hold on
grid on
box on
axis equal
plot(xb_rr(1,:),xb_rr(2,:),'*:b','LineWidth',1.5) % base
plot(xm_rr(1,:),xm_rr(2,:),'o-k','LineWidth',1.5) % vehicle
title('Heuristic solution')
legend('Base','Vehicle')




