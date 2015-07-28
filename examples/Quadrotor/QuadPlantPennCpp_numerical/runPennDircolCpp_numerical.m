function [utraj,xtraj,prog,r] = runPennDircolCpp_numerical

% Simple trajectory optimization which shows trajectory optimization of a
% quadrotor with:
%    - Hand-specified dynamics
%    - Analytical gradients computed via generateGradients.m util


% Add parent directory to path
path = pwd;
addpath(fileparts(path));

r_temp = Quadrotor();                 % Leverages urdf visualizer from the Quadrotor class
v = constructVisualizer(r_temp);      
r = QuadPlantPennCpp_numerical();
r.setOutputFrame(r_temp.getStateFrame());


N = 21;
minimum_duration = .1;
maximum_duration = 8;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);

x0 = Point(getStateFrame(r));  
x0.x = 1.0;
x0.z = 1.0;                     
u0 = double(nominalThrust(r));
prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1); 
prog = prog.addInputConstraint(ConstantConstraint(u0),1); 

x1 = x0;
x1.x = -1.0;
x1.z = 0.5;
prog = prog.addStateConstraint(ConstantConstraint(double(x1)),round(N/4));
x2 = x1;
x2.x = 0;
x2.y = -1.5;
x2.z = 1.5;
prog = prog.addStateConstraint(ConstantConstraint(double(x2)),round(N/2));
x3 = x2;
x3.x = 3;
x3.z = 1;
prog = prog.addStateConstraint(ConstantConstraint(double(x3)),round(3*N/4));
xf = x0;                       
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);
prog = prog.addInputConstraint(ConstantConstraint(u0),N); 

prog = prog.addRunningCost(@cost); 
prog = prog.addFinalCost(@finalCost); 

tf0 = 2;
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0); 

info=0;
while (info~=1)
  tic
  [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
  toc
end

if (nargout<1)
  xtraj = xtraj.setOutputFrame(r_temp.getStateFrame());
  v.playback(xtraj,struct('slider',true));
end

end

function [g,dg] = cost(dt,x,u)

R = eye(4);
g = u'*R*u;
dg = [zeros(1,1+size(x,1)),2*u'*R];

end

function [h,dh] = finalCost(t,x)

h = t;
dh = [1,zeros(1,size(x,1))];

end

