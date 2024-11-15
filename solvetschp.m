function [xb,xm, q,u,xb_a,xb_d,t, opt_cost,ex_time]=solvetschp(r,v_m,v_b,t_p,p)
% Solve the TSchP problem
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


%%% Decision variables
q = binvar(N,N,'full'); % link variable
u = intvar(N,1); % visiting order
xb_a = sdpvar(2,N,'full'); % position of the base at the arrival of the mission vehicle at the poi
xb_d = sdpvar(2,N,'full'); % position of the base at the departure of the mission vehicle at the poi
t = sdpvar(1,N-1,'full');  % time between the departure from poi i and the arrival to the next poi


%%% Constraints and cost
objective = 0; % travelling time

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
M=max(Adj_W,[],'all')+2*r+1000; % big-M 

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

constr_t=[];
for ii=1:N-1
    constr_t=[constr_t, t(ii)>=0];
end

constr_rad=[];
constr_vel=[];
for ii = 1:N
    constr_rad = [constr_rad, norm(xb_a(:,ii)-p(:,ii)) <= r];
    constr_rad = [constr_rad, norm(xb_d(:,ii)-p(:,ii)) <= r];
    if (ii<N)
        objective = objective + t(ii); 
        for jj=1:N
            if(ii~=jj)
                constr_vel = [constr_vel, norm(xb_d(:,ii)-xb_a(:,jj))<= v_b*t(ii)+(1-q(ii,jj))*M]; 
                constr_vel = [constr_vel, norm(xb_a(:,ii)-xb_d(:,ii))<= v_b*t_p];
                constr_vel = [constr_vel, norm(p(:,ii)-p(:,jj))<= v_m*t(ii)+(1-q(ii,jj))*M]; 
            end
        end
    end

end

x0 = p(:,1);   % initial point
xf = p(:,end); % final point 

constraints = [xb_a(:,1) == x0, xb_d(:,1) == x0,  xb_a(:,N) == xf, xb_d(:,N) == xf];
constraints = [constraints, constr_q, constr_u, constr_rad, constr_vel, constr_t];


%%% Solve the optimization problem
opt = sdpsettings('solver','gurobi', 'verbose',0); 
%opt = sdpsettings('solver','gurobi', 'gurobi.MIPGap',0.2, 'gurobi.TuneTimeLimit', 100);
result=optimize(constraints,objective,opt);
misocp_ex_time=result.solvertime;


%%% Process the results
ex_time=misocp_ex_time;
opt_cost=double(objective);

xb_a=double(xb_a);
xb_d=double(xb_d);
u=uint8(double(u));
q=uint8(double(q));
t=double(t);

xm=zeros(2,N);
xb=zeros(2,2*N);
for ii=1:N
    xb(:,2*u(ii)-1)=xb_a(:,ii); 
    xb(:,2*u(ii))=xb_d(:,ii);  
    xm(:,u(ii))=p(:,ii);
end

