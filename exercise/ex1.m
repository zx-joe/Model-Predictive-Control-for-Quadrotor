clc 
close all
clear all

% System dynamics
A = [4/3 -2/3;1 0];
B = [1;0];
C = [-2/3 1];

% Cost matrices
Q = C'*C+0.001*eye(2);
R = 0.001;

%% Prob1 
% Horizon
N = 8;

% Bellman/Riccati recursion
H = Q;
for i = N:-1:1
	K = -(R+B'*H*B)\B'*H*A;
	H = Q + K'*R*K + (A+B*K)'*H*(A+B*K);
		
	% Store the complete time-varying feedback law
    % (necessary to later plot the predictions)
	Kcomp{i} = K;
end

%% Prob2

% Initial condition
x0 = [10; 10];
x = x0;

figure(1); clf; 
hold on; grid on;
axis([0 11 0 11]);

hClosedLoop = plot(x(1,:),x(2,:),'k-o','linewidth',2,'markersize',8,'markerfacecolor','w');
hPred = plot(x(1,:),x(2,:),'-d','LineWidth',2,'markersize',8,'markerfacecolor','w');
hInitial = plot(x0(1),x0(2),'sk', 'markerfacecolor', 'k', 'markersize', 15);

legend([hInitial, hPred, hClosedLoop], ...
  {'Initial condition', 'Prediction', 'Closed-loop trajectory'}, ...
  'Location', 'NorthWest');
title('State-Space');

% Simulation
tmax = 20;

%
for t = 1:tmax
    
    % Always apply the first controller gain 
    % ("receiding horizon fashion")
    u(t) = Kcomp{1}*x(:,t); 
    
    x(:,t+1) = A*x(:,t) + B*u(t);
    y(t) = C*x(:,t);
    
    % Check if the system is diverging ("instability")
    if norm(x(:,t+1),2) > 3*norm(x0,2), error('===> System unstable!'); break; end
    
    t = t+1;

    % Plot the state evolution
    hClosedLoop.XData = x(1,:); hClosedLoop.YData = x(2,:);

    % Calculating and plotting the prediction (from current x)
    xpred(:,1) = x(:,t);
    for tt=1:N
      xpred(:,tt+1) = A*xpred(:,tt) + B*(Kcomp{tt}*xpred(:,tt)); 
    end
    hPred.XData = xpred(1,:); hPred.YData = xpred(2,:);
    
    pause(0.5)
end


% % Calculating and plotting the prediction (from x0)
% xpred(:,1) = x0;
% for t=1:N, xpred(:,t+1) = A*xpred(:,t) + B*(Kcomp{t}*xpred(:,t)); end
% plot(xpred(1,:),xpred(2,:),'m-o','LineWidth',1.2);


%% Prob3
% Computing the infinite horizon LQR
[Kih,~,~] = dlqr(A,B,Q,R,[]);
% Reversing its sign (see help dlqr)
Kih = -Kih; 

% Comparing costs
tmax = 1000;
xfh(:,1) = x0;
xih(:,1) = x0;
for t = 1:tmax
    
    ufh(t) = Kcomp{1}*xfh(:,t);
    uih(t) = Kih*xih(:,t);
    
    xfh(:,t+1) = A*xfh(:,t) + B*ufh(t);
    xih(:,t+1) = A*xih(:,t) + B*uih(t);
    
    lfh(t) = xfh(:,t)'*Q*xfh(:,t) + ufh(t)'*R*ufh(t);
    lih(t) = xih(:,t)'*Q*xih(:,t) + uih(t)'*R*uih(t);
    
    t = t + 1;
end
lfh(t+1) = xfh(:,t)'*Q*xfh(:,t);
lih(t+1) = xih(:,t)'*Q*xih(:,t);

% Final costs = sum of stage costs
Vfh = sum(lfh);
Vih = sum(lih);

disp(['Finite horizon cost (with N=' num2str(N) '): ' num2str(Vfh)])
disp(['Infinite horizon cost: ' num2str(Vih)])

% EOF