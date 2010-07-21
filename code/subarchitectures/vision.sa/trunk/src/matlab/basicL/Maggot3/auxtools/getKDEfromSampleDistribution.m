function model = getKDEfromSampleDistribution( model ) 


N = size(model.w,2) ;
model.Cov = {} ;
for i = 1 : N
    model.Cov = horzcat(model.Cov, model.smod.ps.Cov{i}+model.smod.H) ;
end

% rescale BWs if the variable bandwidths are required
if isfield(model.smod,'useVbw') && model.smod.useVbw == 1
    model = recalculateLocalVariableKDE( model ) ;
end