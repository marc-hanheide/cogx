classdef RawMapping < Mapping
    %SOM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
    end
    
    properties (SetAccess = private)
        
        % Default settings...
        
        % CoOccurences...
        CoOccurences = [];
        
    end
    
    methods
        function obj = RawMapping(obj)
            
        end
        
        function obj = train(obj, inMatches, outMatches, varargin)
            
            % Store CoOccurences...
            if isempty(obj.CoOccurences)
                obj.CoOccurences = [inMatches outMatches];
            else
                obj.CoOccurences = [[obj.CoOccurences(:,1)' inMatches']' [obj.CoOccurences(:,2)' outMatches']'];
            end
        end
    end
end