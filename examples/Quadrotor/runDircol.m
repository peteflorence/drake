function [utraj,xtraj,prog,r] = runDircol

% simple planning demo which takes the quadrotor from hover at x=0m to a new hover at
% x=2m with minimal thrust.

r = Quadrotor(); % Quadrotor constructor

N = 11;
minimum_duration = .1;
maximum_duration = 4;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros
x0.base_z = .5; % lift the quad off the ground
u0 = double(nominalThrust(r));

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1); % DirectTrajectoryOptimization method
prog = prog.addInputConstraint(ConstantConstraint(u0),1); % DirectTrajectoryOptimization method

xf = x0;                       % final conditions: translated in x
xf.base_x = 2;                 % translate x by 2
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N); % DirectTrajectoryOptimization method
prog = prog.addInputConstraint(ConstantConstraint(u0),N); % DirectTrajectoryOptimization method

prog = prog.addRunningCost(@cost); % note: DirCol not Direct!  DircolTrajectoryOptimization method
prog = prog.addFinalCost(@finalCost); % DirectTrajectoryOptimization method

tf0 = 2;                      % initial guess at duration 
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)])); % traj.init.x is a PPTrajectory < Trajectory < DrakeSystem
traj_init.u = ConstantTrajectory(u0); % traj_init.u is a ConstantTrajectory < Trajectory < DrakeSystem

info=0;
while (info~=1)
  tic
  [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init); % DirectTrajectoryOptimization method
  toc
end

if (nargout<1)
  v = constructVisualizer(r);
  v.playback(xtraj,struct('slider',true));
end

end

function [g,dg] = cost(dt,x,u)

R = eye(4);
g = u'*R*u;
%g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
%dg = zeros(1, 1 + size(x,1) + size(u,1));

end

function [h,dh] = finalCost(t,x)

h = t;
dh = [1,zeros(1,size(x,1))];

end

