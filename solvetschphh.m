function [xb,xm, q,u,xb_a,xb_d,t, opt_cost,ex_time]=solvetschphh(r,v_m,v_b,t_p,p)
% Solve the TSchP problem through the heuristic
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


%% SOLVE TSP

%%% Decision variables
q = binvar(N,N,'full'); % link variable
u = intvar(N,1); % visiting order


%%% Constraints
Adj=ones(N); % adjacency matrix
Adj_W=ones(N); % weighted adjacency matrix
for ii=1:N
    for jj=1:N
        Adj_W(ii,jj)=norm(p(:,ii)-p(:,jj));
        if(ii==jj)
            Adj_W(ii,jj)=0;
            Adj(ii,jj)=0;
        end        
    end
end

constr_q=[];
constr_q=[constr_q, Adj(1,:)*q(1,:)'==1]; 
constr_q=[constr_q, Adj(:,N)'*q(:,N)==1];
for ii=2:N-1
    constr_q=[constr_q, Adj(ii,:)*q(ii,:)'- Adj(:,ii)'*q(:,ii)==0, Adj(ii,:)*q(ii,:)'==1, Adj(:,ii)'*q(:,ii)==1];
end
for ii=2:N
    for jj=2:N
        if (Adj(ii,jj)~=0)
            constr_q=[constr_q,(u(ii)-u(jj))+1-((N-1)*(1-q(ii,jj)))<=0];
        end
    end
end

constr_u=[];
constr_u=[u(1)==1, u(N)==N];
for ii=2:N-1
    constr_u=[constr_u, u(ii)>=2, u(ii)<=N-1];    
end

constraints = [constr_q, constr_u];


%%% Cost
objective = 0;
for jj=1:N 
    for ii=1:N
        objective = objective + Adj_W(ii,jj)*q(jj,ii) ;
    end
end



%%% Solve the optimization problem
opt = sdpsettings('solver','gurobi','verbose', 0); 
result=optimize(constraints,objective,opt);
tsp_ex_time=result.solvertime;


u=uint8(double(u));
q=uint8(double(q));



%% SOLVE SOCP

ps=zeros(2,N); % sort poi according to the tsp solution
for ii = 1:N
    ps(:,u(ii))=p(:,ii);
end

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

x0 = p(:,1);   % initial point
xf = p(:,end); % final point 

constraints = [xb_as(:,1) == x0, xb_ds(:,1) == x0, xb_as(:,N) == xf, xb_ds(:,N) == xf]; 
constraints = [constraints, constr_rad, constr_vel, constr_t];


%%% Solve the optimization problem
opt = sdpsettings('solver','gurobi','verbose', 0); 
result=optimize(constraints,objective,opt);
socp_ex_time=result.solvertime;


%%% Process the results
ex_time=socp_ex_time + tsp_ex_time;
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
    xb_a(:,ii)=xb_as(:,u(ii));
    xb_d(:,ii)=xb_ds(:,u(ii));
end
t=zeros(1,N-1);
for ii=1:N-1
    t(ii)=double(ts(u(ii)));
end
