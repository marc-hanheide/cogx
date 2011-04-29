classdef SOMUpdateRule < UpdateRule
    
    methods (Static = true)
        
        function Mod = update(Algo, Mod, Data, iData, varargin)

            %% Update...
            Mod.SOM.codebook(:,Mod.known) =...
                Mod.SOM.codebook(:,Mod.known) - Mod.a*Mod.h(:,ones(sum(Mod.known),1)).*Mod.Dx;
            
        end
        
    end
    
end