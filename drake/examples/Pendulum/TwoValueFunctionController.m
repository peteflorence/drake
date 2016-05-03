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
        
        theta = u(1);
        theta_dot = u(2);
        S_sample = [theta theta_dot]';
        
        % determine my feature vector, phi_sample_gradient
        phi_sample_gradient = obj.generatePhiSampleGradient(S_sample);

        % decide if we should use V1 or V2
        V_closer = obj.computeCloserV(S_sample);
        
        % use weights from correct V
        if V_closer == 1
            gradient_wrt_theta_dot = obj.data.alpha1'*phi_sample; 
        else
            gradient_wrt_theta_dot = obj.data.alpha2'*phi_sample; 
        end
        control_input = -gradient_wrt_theta_dot;
        
        y = control_input;
        if y > 10
            y = 10;
        elseif y <= -10
            y = -10;
        end
        
    end
    
    
    function phi_sample = generatePhiSampleGradient(obj, S_sample)
        phi_sample = ones(size(obj.data.degree,2),1);
        for i=1:size(obj.data.degree,2)
            for j=1:length(obj.data.basis_generators)
                if j == 3  % if j=3, we are looking at a monomial of theta_dot, and need to take the derivative
                    if obj.data.degree(j,i)==0
                        phi_sample(i) = 0;
                    else
                        phi_sample(i) = obj.data.degree(j,i) * phi_sample(i,:).*(obj.data.basis_generators{j}(S_sample).^(obj.data.degree(j,i)-1));
                    end
                else
                    phi_sample(i) = phi_sample(i).*(obj.data.basis_generators{j}(S_sample).^obj.data.degree(j,i));
                end
            end
        end
    end
    
    function phi_sample = generatePhiSample(obj, S_sample)
        phi_sample = ones(size(obj.data.degree,2),1);
        for i=1:size(obj.data.degree,2)
            for j=1:length(obj.data.basis_generators)
                
                phi_sample(i) = phi_sample(i).*(obj.data.basis_generators{j}(S_sample).^obj.data.degree(j,i));
                
            end
        end
    end
    
    function V_closer = computeCloserV(obj, S_sample)
        closest_index = 1;
        smallest_L2_squared = 1e9;
        for i = 1:size(obj.data.S,2)
            L2_squared = (obj.data.S(1,i)-S_sample(1))^2 + (obj.data.S(2,i)-S_sample(2))^2;
            if L2_squared < smallest_L2_squared
                closest_index = i;
                smallest_L2_squared = L2_squared;
            end
        end
        if obj.data.V1_indices(closest_index)
            V_closer = 1;
        else
            V_closer = 2;
        end
    end
        
        
  end 

  methods (Static)
    function run()
      pd = PendulumPlant;
      pd = setInputLimits(pd,-inf,inf);
      pv = PendulumVisualizer();
      c = TwoValueFunctionController(pd);

      sys = feedback(pd,c);
      
%       for i=1:5
        
        xtraj = simulate(sys,[0 6], [0.05 -0.02]');
        playback(pv, xtraj);
%         playbackAVI(pv,xtraj, ['swingup_TwoValueFunction' num2str(i)]);
        figure(2); fnplt(xtraj);
%       end
    end
  end  
end