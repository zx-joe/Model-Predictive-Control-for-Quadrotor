function [ctrl,traj] = ctrl_NMPC(quad)

import casadi.*

opti = casadi.Opti(); % Optimization problem

N =10; % MPC horizon [SET THIS VARIABLE]

% decision variables 
X = opti.variable(12,N+1); % state trajectory variables 
U = opti.variable(4, N); % control trajectory (throttle, brake)

X0 = opti.parameter(12,1); % initial state 
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]


%%%%%%%%%%%%%%%%%%%%%%%% 
%%%% YOUR CODE HERE %%%% 
Xs = opti.variable(12,1);
Us = opti.variable(4, 1);
h=0.22;
f=@(a,b) quad.f(a,b);
f_discrete = @(x,u) RK4(x,u,h,f);

cost=0;
for i=1:4
    cost=cost+(U(i,:)-Us(i))*(U(i,:)-Us(i))';
end
for i=1:12
    cost=cost+(X(i,:)-Xs(i))*(X(i,:)-Xs(i))';
end
opti.minimize(cost)

for k=1:N % loop over control intervals
  opti.subject_to(X(:,k+1) == f_discrete(X(:,k), U(:,k)));
end

opti.subject_to(0 <= U <= 1.5);
opti.subject_to(-0.035 <= X(4,:) <= 0.035);
opti.subject_to(-0.035 <= X(5,:) <= 0.035);
opti.subject_to(X(:,1)==X0)


opti.subject_to(-0.035<=Xs(4)<=0.035);
opti.subject_to(-0.035<=Xs(5)<=0.035);
opti.subject_to(Xs(6)==REF(4));
opti.subject_to(Xs(10)==REF(1));
opti.subject_to(Xs(11)==REF(2));
opti.subject_to(Xs(12)==REF(3));
opti.subject_to(Xs == f_discrete(Xs, Us));
opti.subject_to(0 <= Us <= 1.5);

%%%%%%%%%%%%%%%%%%%%%%%%

ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);

end

function u = eval_ctrl(x, ref, opti, X0, REF, X, U) 
%  Set the initial state and reference 
opti.set_value(X0, x); 
opti.set_value(REF, ref);

% Setup solver NLP 
ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false); 
opti.solver('ipopt', ops);

% Solve the optimization problem 
sol = opti.solve(); 
assert(sol.stats.success == 1, 'Error computing optimal input');
u = opti.value(U(:,1));

% Use the current solution to speed up the next optimization 
opti.set_initial(sol.value_variables()); 
opti.set_initial(opti.lam_g, sol.value(opti.lam_g)); 
end

function [x_next] = RK4(X,U,h,f)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%

% Runge-Kutta 4 integration
% write your function here
   k1 = f(X,         U);
   k2 = f(X+h/2*k1, U);
   k3 = f(X+h/2*k2, U);
   k4 = f(X+h*k3,   U);
   x_next = X + h/6*(k1+2*k2+2*k3+k4);

end
