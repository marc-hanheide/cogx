classdef HebbianMapping < Mapping
    %HEBBIANMAPPING Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        % Default settings...
        
        % Modality point/node matches...
        CoOccurences = [];
        
        % Modality activations for the point/node matches...
        InActivations = [];
        OutActivations = [];
        
        % Hebbian link weights...
        Weights = [];
        
        % Current timestep...
        t = [];
    end
    
    methods
        
        function obj = HebbianMapping(inModality, outModality)
            
            obj.Weights = zeros(size(inModality.SOM.codebook,1),...
                                    size(outModality.SOM.codebook,1));
                                
            obj.t = 0;
        end
        
        function obj = train(obj, inMatches, outMatches, InActivations, OutActivations, varargin)
            
            %% Store matches...
            if isempty(obj.CoOccurences)
                obj.CoOccurences = [inMatches' outMatches'];
            else
                obj.CoOccurences = [[obj.CoOccurences(:,1)' inMatches]' [obj.CoOccurences(:,2)' outMatches]'];
            end
            
            %% Store activations...
            if isempty(obj.InActivations)
                obj.InActivations = InActivations';
            else
                obj.InActivations = [obj.InActivations' InActivations]';
            end
            
            if isempty(obj.OutActivations)
                obj.OutActivations = OutActivations';
            else
                obj.OutActivations = [obj.OutActivations' OutActivations]';
            end
            
            %% Training...
            for t = (obj.t + 1):size(obj.InActivations,1)

                WeightAdjustments = obj.DeltaFunction(t);
                Divisor = repmat(sqrt( sum((obj.Weights + WeightAdjustments).^2, 2)),1, size(obj.Weights,2));
                Divisor(isnan(Divisor)) = 1;
                Divisor(Divisor==0) = 1;

                obj.Weights = (obj.Weights + WeightAdjustments) ./ Divisor;
                % obj.Weights = (obj.Weights + WeightAdjustments);
            end
            
            obj.t = t;
                
        end
    end
    
    methods (Access = private)
        
        function WeightAdjustments = DeltaFunction(obj, t)
            
            WeightAdjustments = repmat(obj.InActivations(t,:),size(obj.OutActivations(t,:),2),1)' .*...
                repmat(obj.OutActivations(t,:),size(obj.InActivations(t,:),2),1);
            
%             for i = 1:size(obj.InActivations(t,:),2)
%                 for j = 1:size(obj.OutActivations(t,:),2)
%                     WeightAdjustments(i,j) = obj.InActivations(t,i) * obj.OutActivations(t,j);
%                 end
%             end
            
            % Linear...
            a = 1 - ((size(obj.InActivations,1) - t) / size(obj.InActivations,1));
            
            % Inverse...
            % a = 1 / (1 + 99*(t-1)/(size(obj.InActivations,1)-1));
            
            % Power...
            % a = (0.005/1)^((size(obj.InActivations,1)-t-1) / size(obj.InActivations,1));
            
            WeightAdjustments = a * WeightAdjustments;
        end
    end
    
end

