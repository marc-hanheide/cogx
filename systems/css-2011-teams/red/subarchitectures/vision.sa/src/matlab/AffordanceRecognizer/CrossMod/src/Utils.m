classdef Utils < handle
    
    properties
        
    end
    
    methods (Static = true)
        
        %% --------------------- *** RADIUS *** ---------------------------
        %------------------------------------------------------------------
        function r = radius(type, rini, rfin, train_length, shift, t)
            
            % switch Mod.radius_type, % radius
            %     case 'linear', r = rini + (rfin-rini) * ((t - shift) - 1) / ((train_length / (1/shift))-1);
            % end
            r = rini + (rfin - rini) * ((t - shift) - 1) / (train_length - 1);
            
            if ~r, r=eps; end % zero neighborhood radius may cause div-by-zero error  
            
        end
        
        
        %% ----------------- *** NEIGHBOURHOOD *** ------------------------
        %------------------------------------------------------------------
        function h = neighbourhood(type, Ud, bmu, r)
            
            switch type, % neighborhood function 
                case 'bubble',   h = (Ud(:,bmu) <= r);
                case 'gaussian', h = exp(-(Ud(:,bmu).^2)/(2*r*r)); 
                case 'cutgauss', h = exp(-(Ud(:,bmu).^2)/(2*r*r)) .* (Ud(:,bmu) <= r);
                case 'ep',       h = (1 - (Ud(:,bmu).^2)/(r*r)) .* (Ud(:,bmu) <= r);
            end
            
        end


        %% ----------------- *** LEARNING RATE *** ------------------------
        %------------------------------------------------------------------
        function a = learningrate(type, seed, train_length, shift, t)
                        
            %% Calculate current learning rate...            
            switch type,                        
                case 'constant',...
                     a = seed;
                case 'linear',...
                     a = (1-(t - shift) / train_length) * seed;
                case 'linear_inc',...
                     a = ((t - shift) / train_length) * seed;
                case 'inv',...
                     a = seed / (1 + 99*((t - shift)-1) / (train_length-1));
                case 'power',...
                     a = seed * (0.005/seed)^(((t - shift)-1)/train_length);
                case 'inv_nonnull',...
                     a = t^(-1/2);
                
            end
            
        end
        
    end
    
end
