classdef QuadPlantPennCpp_numerical < DrakeSystem
  % Modified from D. Mellinger, N. Michael, and V. Kumar,
  % "Trajectory generation and control for precise aggressive maneuvers with quadrotors",
  %  In Proceedings of the 12th International Symposium on Experimental Robotics (ISER 2010), 2010.
  
  methods
    function obj = QuadPlantPennCpp_numerical()
      obj = obj@DrakeSystem(12,0,4,12,false,1);
      obj = setStateFrame(obj,CoordinateFrame('QuadState',12,'x',{'x','y','z','roll','pitch','yaw','xdot','ydot','zdot','rolldot','pitchdot','yawdot'}));
      obj = obj.setOutputFrame(obj.getStateFrame);
    end
    
    function m = getMass(obj)
      m = obj.m;
    end
    
    function I = getInertia(obj)
      I = obj.I;
    end
    
    function u0 = nominalThrust(obj)
      % each propellor commands -mg/4
      u0 = Point(getInputFrame(obj),getMass(obj)*norm(obj.gravity)*ones(4,1)/4);
    end
    
    function [xdot,df] = dynamics(obj,t,x,u)
      
      options = struct();
      options.grad_method = 'numerical';
      
      tempfunc = @(t, x, u) obj.dynamics_no_grad(t, x, u);
      
      [xdot, df] = geval(tempfunc, t, x, u, options);  
      
    end
    
    % dynamics_no_grad() is overloaded by cpp
    
    function y = output(obj,t,x,u)
      y = x;
    end
    
    function x = getInitialState(obj)
      x = zeros(12,1);
    end
    
  end
  properties
    m = .5;
    I = diag([0.0023,0.0023,0.004]);
    gravity = [0; 0; -9.8100]
  end
  
end



