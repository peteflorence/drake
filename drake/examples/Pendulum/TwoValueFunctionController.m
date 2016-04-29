classdef TwoValueFunctionController < DrakeSystem
  
  properties 
    p
    data
  end
  
  methods
    function obj = TwoValueFunctionController(plant)
      obj = obj@DrakeSystem(0,0,2,1,true,true);
      typecheck(plant,'PendulumPlant');
      obj.p = plant;
      obj = setInputFrame(obj,PendulumState);
      obj = setOutputFrame(obj,PendulumInput);
      obj.data = load('ValueFunctionFit.mat');
      
    end
    
    function y = output(obj,t,~,u)
        disp(obj.data);
        y = 0;
%         phi = ones(size(degree,2),1);
%         
%         for i=1:size(degree,2)
%             for j=1:length(basis_generators)
%                 phi = phi.*(basis_generators{j}(u).^degree(j,i));
%             end
%         end
%         
%         % choose which value function
%         
%         alpha1 = (phi(:,V1_indices)')\J(V1_indices);
%         alpha2 = (phi(:,V2_indices)')\J(V2_indices);
%         
%         V1 = alpha1'*phi(:,V1_indices);
%         V2 = alpha2'*phi(:,V2_indices);
%         
%         
%         
%         
%         % take gradient wrt theta_dot
% 
%         y = alpha'*phi;
        
    end
  end 

  methods (Static)
    function run()
      load('ValueFunctionFit.mat');
      pd = PendulumPlant;
      pd = setInputLimits(pd,-inf,inf);
      pv = PendulumVisualizer();
      c = TwoValueFunctionController(pd);

      sys = feedback(pd,c);
      
      for i=1:5
        xtraj = simulate(sys,[0 6]);
        playback(pv,xtraj);
        figure(2); fnplt(xtraj);
      end
    end
  end  
end