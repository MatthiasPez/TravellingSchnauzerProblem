function [xb,xm, q,u,xb_a,xb_d,t, opt_cost,sim_time]=solvetschprr(r,v_m,v_b,t_p,p)
% Solve the TSchP by searching for single swaps in the point sequence that 
% decrease the cost starting from the solution provided by the heuristic.
% Once the point sequence is modified, a complete feasible solution to the
% TSchP is obtained by solving the SOCP problem for the given ordered seq
% Yalmip and Gurobi required

%%% Input
% r: maximum distance between vehicles
% t_p: visiting time (constant)
% v_m: maximum velocity of the mission vehicle
% v_b: maximum velocity of the base station
% p: locations of poi (2xN matrix)

%%% Output
% q: link variable (NxN matrix)
% u: visiting order (1XN vector)
% xb_d: position of the base at the arrival of the mission vehicle at the poi (2xN matrix)
% xb_r: position of the base at the departure of the mission vehicle at the poi (2xN matrix)
% t: time elapsed between the departure from a poi and the arrival to the next poi (1xN vector)
% xb: sequence of positions of the base (2x2*N matrix)
% xm: sequence of positions of the mission vehicle (2xN matrix)

% q(i,j)=1 if poi j is visited by the mission vehicle after poi i, 0 otherwise
% u(i) is the visiting order of poi i
% xb_d(:,i) is the position of the base at the arrival of the mission vehicle at the poi i
% xb_r(:,i) is the position of the base at the departure of the mission vehicle at the poi i
% t(i) is the time elapsed between the departure from poi i and the arrival to the next poi


N = size(p,2); % number of points

[~,p_opt, ~,~,~,~,~, opt_cost,ex_time]=solvetschphh(r,v_m,v_b,t_p,p);

tic
for ii=2:N-1
    p_pivot=p_opt(:,ii);
    p_aux=p_opt;
    for jj=2:N-1
        if jj~=ii
            p_temp=p_opt;
            p_temp(:,ii)=[]; % remove p_pivot from position ii
            p_ij=[p_temp(:,1:jj-1), p_pivot, p_temp(:,jj:end)]; % insert p_pivot in position jj
    
            [~,~,~,~,~, opt_cost_ij,ex_time_ij]=solvetschphh_socp(r,v_m,v_b,t_p,p_ij);
            if opt_cost_ij < opt_cost
                p_aux=p_ij;
                opt_cost=opt_cost_ij;
            end
            ex_time=ex_time+ex_time_ij;
            yalmip('clear')
        end
    end
end
p_opt=p_aux;
sim_time=toc;



[xb,xm, xb_a,xb_d,t, ~,~]=solvetschphh_socp(r,v_m,v_b,t_p, p_opt);

q = zeros(N,N); % place holder
for ii=1:N-1
    q(ii,ii+1)=1;
end
u = 1:1:N; % place holder