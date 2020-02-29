classdef MPC_Control_x < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % SET THE HORIZON HERE
      %N = ...不确定
      N=20;
      
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      Fx = [0 1 0 0 ; 0 -1 0 0];
      fx=[0.035; 0.035];
      Fu=[1;-1];
      fu=[0.3;0.3];
      
      Q=diag([1 5 1 5]); % 不确定
      R=eye(m)*1; % 不确定
      
      % max invariant set for terminal LQR controller
      [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
      %Qf
      %P = dlyap(mpc.A,Q);
      K = -K; 
      Xf = polytope([Fx;Fu*K],[fx;fu]);
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
      con = [];
      obj = 0;
      %con = (x(:,2) == sys_x.A*x(:,1) + sys_x.B*u(:,1)) + (M*u(:,1) <= m);
      %obj = u(:,1)'*R*u(:,1);
      for i = 1:N-1
          con = [con, x(:,i+1) == mpc.A*x(:,i) + mpc.B*u(:,i)]; 
          con = con + (Fx*x(:,i) <= fx) + (Fu*u(:,i) <= fu);
          %obj = obj + x(:,i)'*Q*x(:,i) + u(:,i)'*R*u(:,i);
          obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us);
      end
      %con = con + (Ff*x(:,N) <= ff);
      %obj = obj + x(:,N)'*Qf*x(:,N);
      obj = obj + (x(:,N)-xs)'*Qf*(x(:,N)-xs);
    

      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %ctrl_opt = optimizer(con, obj,  sdpsettings('verbose',0),{x(:,1), xs, us}, u(:,1));
      
      ctrl_opt = optimizer(con, obj,  sdpsettings('solver','gurobi'),{x(:,1), xs, us}, u(:,1));
        
      
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;  
      
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      
      %con = [];
      %obj = 0;
      
      con=[-0.3 <= us <= 0.3 ,  -0.035<=xs(2)<=0.035 , ...
                xs == mpc.A*xs + mpc.B*us    ,...
                ref == mpc.C*xs + mpc.D*us   ];
      obj   = us'*us;
      
      
      %con=[-0.3 <= us <= 0.3 , xs == mpc.A*xs + mpc.B*us, -0.035<=xs(2)<=0.035 ];
      %obj  = (ref -mpc.C*xs -mpc.D*us)'*(ref -mpc.C*xs -mpc.D*us) ;
      
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
