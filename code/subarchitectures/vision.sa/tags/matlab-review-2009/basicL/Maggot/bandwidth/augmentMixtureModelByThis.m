%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function model = augmentMixtureModelByThis( model, obs, H, obs_mixing_weights, mix_weights )

% read dimension and number of components
[ d, N ]= size(model) ;

U = size(obs,2) ;
% augment the model
model.Mu = [model.Mu, obs] ;
if ~isempty(obs)
    model.w = [model.w*mix_weights(1), obs_mixing_weights*mix_weights(2)] ;
end
 
if ( abs(sum(model.w)-1) > 1e-5 )
    error('Weights should sum to one!!') ;
end
model.w = model.w / sum(model.w) ;

for i = 1 : U
    if iscell(H)
        lN = length(H) ;
        if lN > 1
            H0 = H{i} ;
        else
            H0 = H{1} ;
        end
    else
        H0 = H ;
    end
    model.Cov = horzcat(model.Cov, H0) ;    
    model.suffStat.B = horzcat(model.suffStat.B, {H0}) ;  
end

for i = 1 : size(obs,2)
    model.suffStat.A = horzcat(model.suffStat.A, {obs(:,i)*obs(:,i)'}) ;

    sub_pdf.Mu = obs(:,i) ;
    sub_pdf.Cov = {H0} ;
    sub_pdf.w = 1 ;
    sub_pdf.A = {obs(:,i)*obs(:,i)'} ;
    sub_pdf.B = {H0} ;
    model.suffStat.subLayer = horzcat( model.suffStat.subLayer, sub_pdf ) ;    
end