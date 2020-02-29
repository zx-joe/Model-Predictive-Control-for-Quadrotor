function draw = draw(tag)
    draw=0;
    Ts = 1/5; 
    quad = Quad(Ts); 
    [xs, us] = quad.trim(); 
    sys = quad.linearize(xs, us); 
    [sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys, xs, us);
    if tag == 'x'
        mpc = MPC_Control_x(sys_x, Ts);
        x0 = [0;0;0;2];
        ref = -2;
        target = 4;
    elseif tag == 'y'
        mpc = MPC_Control_y(sys_y, Ts);
        x0 = [0;0;0;2];
        ref = -2;
        target = 4;
    elseif tag == 'z'
        mpc = MPC_Control_z(sys_z, Ts);
        x0 = [0;2];
        ref = -2;
        target = 2;
    elseif tag == "yaw"
        mpc = MPC_Control_yaw(sys_yaw, Ts);
        x0 = [0; 0];
        ref = pi/4;
        target=2;
        
    end
    
    sol.x(:,1) = x0;

    i = 1;
    while norm(ref - sol.x(target,end)) > 1e-5 % Simulate until convergence
      uopt = mpc.get_u(sol.x(:,end),ref);
      %if infeasible == 1, error('Error in optimizer - could not solve the problem'); end
      % Extract the optimal input
      sol.u(:,i) = uopt;
      % Apply the optimal input to the system
      sol.x(:,i+1) = mpc.A*sol.x(:,i) + mpc.B*sol.u(:,i);
      i = i + 1;
      sol.i = i;
    end

    sol.t = 0:Ts:(sol.i-1)*Ts;

    if tag == 'x'
      figure;plot(sol.t,sol.x(1,:),'-b','markersize',10,'linewidth',1);title("vel pitch");ylabel('rad/s');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_x/vel_pitch.jpg");
      figure; plot(sol.t,sol.x(2,:),'-b','markersize',10,'linewidth',1);title("pitch");ylabel('rad');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_x/pitch.jpg");
      figure; plot(sol.t,sol.x(3,:),'-b','markersize',20,'linewidth',1);title("vel "+tag);ylabel('m/s');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_x/vel_x.jpg");
      figure; plot(sol.t,sol.x(4,:),'-b','markersize',20,'linewidth',1);title(tag);ylabel('m');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_x/x.jpg");
    elseif tag == 'y'
      figure;plot(sol.t,sol.x(1,:),'-b','markersize',10,'linewidth',1);title("vel roll");ylabel('rad/s');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_y/vel_roll.jpg");
      figure; plot(sol.t,sol.x(2,:),'-b','markersize',10,'linewidth',1);title("roll");ylabel('rad');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_y/roll.jpg");
      figure; plot(sol.t,sol.x(3,:),'-b','markersize',20,'linewidth',1);title("vel "+tag);ylabel('m/s');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_y/vel_y.jpg");
      figure; plot(sol.t,sol.x(4,:),'-b','markersize',20,'linewidth',1);title(tag);ylabel('m');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_y/y.jpg");
    elseif tag == 'z'
      figure;plot(sol.t,sol.x(1,:),'-b','markersize',10,'linewidth',1);title("vel z");ylabel('m/s');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_z/vel_z.jpg");
      figure; plot(sol.t,sol.x(2,:),'-b','markersize',10,'linewidth',1);title("z");ylabel('m');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_z/z.jpg");
    elseif tag == "yaw"
      figure;plot(sol.t,sol.x(1,:),'-b','markersize',10,'linewidth',1);title("vel yaw");ylabel('rad/s');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_yaw/vel_yaw.jpg");
      figure; plot(sol.t,sol.x(2,:),'-b','markersize',10,'linewidth',1);title("yaw");ylabel('rad');xlabel("time(s)");hold on;%saveas(gcf,deliverable+"_yaw/yaw.jpg");
    end
          
          

      


end