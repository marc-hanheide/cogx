classdef LVQ1UpdateRule < UpdateRule
    
    methods (Static = true)
        
        function Mod = update(Algo, Mod, Data, iData, varargin)

            %% Update...
            if Mod.ClassLabels(Mod.BMUs(1)) == find(Data.ClassLabels(Data.GroundTruthLabelIndices, iData))

                % Update the BMU codebook vector (LVQ)...
                Mod.SOM.codebook(Mod.BMUs(1),Mod.known) =...
                    Mod.SOM.codebook(Mod.BMUs(1),Mod.known) +...
                        Mod.a * (Mod.x(Mod.known) - Mod.SOM.codebook(Mod.BMUs(1),Mod.known));

            else

                % Update the BMU codebook vector (LVQ)...
                Mod.SOM.codebook(Mod.BMUs(1),Mod.known) =...
                    Mod.SOM.codebook(Mod.BMUs(1),Mod.known) -...
                        Mod.a * (Mod.x(Mod.known) - Mod.SOM.codebook(Mod.BMUs(1),Mod.known));

            end
            
        end
        
    end
    
end