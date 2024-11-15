function [xb,xm, xb_a,xb_d,t, opt_cost,ex_time]=solvetschphh_socp(r,v_m,v_b,t_p,ps)
% Solve the TSchP for a given oredered sequence
% Yalmip and Gurobi required

%%% Input
% r: maximum distance between vehicles
% t_p: visiting time (constant)
% v_m: maximum velocity of the mission vehicle
% v_b: maximum velocity of the base station
% ps: sorted locations of poi (2xN matrix)

%%% Output
% xb_d: position of the base at the arrival of the mission vehicle at the poi (2xN matrix)
% xb_r: position of the base at the departure of the mission vehicle at the poi (2xN matrix)
% t: time elapsed between the departure from a poi and the arrival to the next poi (1xN vector)

% xb_d(:,i) is the position of the base at the arrival of the mission vehicle at the poi i
% xb_r(:,i) is the position of the base at the departure of the mission vehicle at the poi i
% t(i) is the time elapsed between the departure from poi i and the arrival to the next poi

N = size(ps,2); % number of points


%%% Decision variables
xb_as = sdpvar(2,N,'full'); % position of the base at the arrival of the mission vehicle at the poi
xb_ds = sdpvar(2,N,'full'); % position of the base at the departure of the mission vehicle at the poi
ts = sdpvar(1,N-1,'full');  % time between the departure from poi i and the arrival to the next poi


%%% Constraints and cost
objective = 0; % travelling time

constr_t=[];
for ii=1:N-1
    constr_t=[constr_t, ts(ii)>=0];
end

constr_rad=[];
constr_vel=[];
for ii = 1:N
    constr_rad = [constr_rad, norm(xb_as(:,ii)-ps(:,ii)) <= r];
    constr_rad = [constr_rad, norm(xb_ds(:,ii)-ps(:,ii)) <= r];
    if (ii<N)
        objective = objective + ts(ii); 
        constr_vel = [constr_vel, norm(xb_ds(:,ii)-xb_as(:,ii+1))<= v_b*ts(ii)];
        constr_vel = [constr_vel, norm(xb_as(:,ii)-xb_ds(:,ii))<= v_b*t_p];
        constr_vel = [constr_vel, norm(ps(:,ii)-ps(:,ii+1))<= v_m*ts(ii)]; 
    end

end

x0 = ps(:,1);   % initial point
xf = ps(:,end); % final point 

constraints = [xb_as(:,1) == x0, xb_ds(:,1) == x0, xb_as(:,N) == xf, xb_ds(:,N) == xf]; 
constraints = [constraints, constr_rad, constr_vel, constr_t];


%%% Solve the optimization problem
opt = sdpsettings('solver','gurobi','verbose', 0); 
result=optimize(constraints,objective,opt);
socp_ex_time=result.solvertime;


%%% Process the results
ex_time=socp_ex_time;
opt_cost=double(objective);

xb_as=double(xb_as);
xb_ds=double(xb_ds);
ts=double(ts);

xm=ps;
xb=zeros(2,N);
for ii=1:N
    xb(:,2*ii-1)=xb_as(:,ii); 
    xb(:,2*ii)=xb_ds(:,ii);  
end

xb_a=zeros(2,N);
xb_d=zeros(2,N);
for ii=1:N
    xb_a(:,ii)=xb_as(:,ii);
    xb_d(:,ii)=xb_ds(:,ii);
end
t=zeros(1,N-1);
for ii=1:N-1
    t(ii)=double(ts(ii));
end