classdef MPC_Control_z < MPC_Control
  properties
    A_bar, B_bar, C_bar % Augmented system for disturbance rejection    
    L                   % Estimator gain for disturbance rejection
  end
  
  methods
    function mpc = MPC_Control_z(sys, Ts)
      mpc = mpc@MPC_Control(sys, Ts);
      
      [mpc.A_bar, mpc.B_bar, mpc.C_bar, mpc.L] = mpc.setup_estimator();
    end
    
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   d_est  - disturbance estimate
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.3)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);

      % SET THE HORIZON HERE
      N = 10; 
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
      
     
      Fu=[1;-1];
      fu=[0.3;0.2];
      
      Q=eye(n)*1; % 不确定
      R=eye(m)*1; % 不确定
      
      % max invariant set for terminal LQR controller
      [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
      %Qf
      %P = dlyap(mpc.A,Q);
      K = -K; 
      Xf = polytope([Fu*K],[fu]);
      Ac = [mpc.A+mpc.B*K];
      while 1
          prevXf = Xf;
          [T,t] = double(Xf);
          preXf = polytope(T*Ac,t);
          Xf = intersect(Xf, preXf);
          if isequal(prevXf, Xf)
              break
          end
      end
      [Ff,ff] = double(Xf);
      
      con=[];
      obj=0;
      for i = 1:N-1
          con = [con, x(:,i+1) == mpc.A*x(:,i) + mpc.B*u(:,i) + mpc.B*d_est]; 
          con = con + (Fu*u(:,i) <= fu);
          obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us);
      end
      %con = con + (Ff*x(:,N) <= ff);
      obj = obj + (x(:,N)-xs)'*Qf*(x(:,N)-xs);
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us, d_est}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      %   d_est  - disturbance estimate
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.3)
      ref = sdpvar;
            
      % Disturbance estimate (Ignore this before Part 5)
      d_est = sdpvar(1);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      u_M=[1,-1];
      u_m=[0.3,0.2];
      
      obj = (mpc.C * xs - ref)' * (mpc.C * xs - ref);
      
      con = (mpc.B * d_est == (eye(n) - mpc.A) * xs - mpc.B * us)...
          + (u_M * us <= u_m);
      %obj = (mpc.C * xs - ref)' * (mpc.C * xs - ref);
      

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), {ref, d_est}, {xs, us});
    end
    
    
    % Compute augmented system and estimator gain for input disturbance rejection
    function [A_bar, B_bar, C_bar, L] = setup_estimator(mpc)
      
      %%% Design the matrices A_bar, B_bar, L, and C_bar
      %%% so that the estimate x_bar_next [ x_hat; disturbance_hat ]
      %%% converges to the correct state and constant input disturbance
      %%%   x_bar_next = A_bar * x_bar + B_bar * u + L * (C_bar * x_bar - y);
      
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D


      [n,m] = size(mpc.B); p = m;
      
      A_bar = [mpc.A, mpc.B; zeros(p,n), eye(p)];
      B_bar = [mpc.B; zeros(p,m)];
      C_bar = [mpc.C, zeros(1,p)];
      
      %L = -place(A_bar',C_bar',[0.45;0.5;0.55])';
      L = -place(A_bar',C_bar',[0.5,0.6,0.7])';

      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    
  end
end
