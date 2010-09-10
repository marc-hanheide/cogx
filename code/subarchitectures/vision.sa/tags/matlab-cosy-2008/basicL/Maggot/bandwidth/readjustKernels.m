%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function [ model, H_out ] = readjustKernels( model, H_new, skipSublayer ) 

if nargin < 3
    skipSublayer = 0 ;
end

if iscell(H_new)
    if size(H_new,2)>1
        error('Not made for multiple kernels yet!!!') ;
    end
    H_new = H_new{1} ;
end

H_out = [] ;
if nargout == 2 && ~isempty(model.suffStat.B{1})
    H_out = model.suffStat.B{1} ;
end
 
for i = 1 : length(model.w)
    if ~isempty(model.suffStat.B)
        model.suffStat.B{i} = H_new  ;
        model.Cov{i} = model.suffStat.B{i} + ...
                       model.suffStat.A{i} - model.Mu(:,i)*model.Mu(:,i)' ;
    end
    
    if isfield( model.suffStat, 'subLayer')
        if ~isempty(model.suffStat.subLayer)
            if ~skipSublayer
                for j = 1 : length(model.suffStat.subLayer(i).w)
                    model.suffStat.subLayer(i).B{j} = H_new ;
                    model.suffStat.subLayer(i).Cov{j} = model.suffStat.subLayer(i).B{j} + ...
                        model.suffStat.subLayer(i).A{j} - ...
                        model.suffStat.subLayer(i).Mu(:,j)*model.suffStat.subLayer(i).Mu(:,j)' ;
                end
%             else
%                 warning('Skipping sublayer!') ;
            end
        end
    end
end
 