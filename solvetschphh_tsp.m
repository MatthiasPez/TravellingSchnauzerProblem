function [xm, q,u, opt_cost,ex_time]=solvetschphh_tsp(v_m,p)
% Solve Euclidian TSP problem
% Yalmip and Gurobi required

%%% Input
% v_m: maximum velocity of the mission vehicle
% p: locations of poi (2xN matrix)

%%% Output
% q: link variable (NxN matrix)
% u: visiting order (1XN vector)
% xm: sequence of positions of the mission vehicle (2xN matrix)

% q(i,j)=1 if poi j is visited by the mission asset after poi i, 0 otherwise
% u(i) is the visiting order of poi i

N = size(p,2); % number of points

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


%%% Process the results
ex_time=tsp_ex_time;
opt_cost=double(objective)/v_m;

xm=zeros(2,N);
for ii=1:N
    xm(:,u(ii))=p(:,ii);
end