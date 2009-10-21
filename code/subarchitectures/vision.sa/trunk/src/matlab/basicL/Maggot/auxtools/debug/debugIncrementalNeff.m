function debugIncrementalNeff()

N = 100 ;
K=1 ;

w_att = 0.95 ;
histor = [] ;


K_eff_t = 1 ;
C_t = 1 ;
w = [1] ;
W = [1] ;
C = 1 ;

histor = [K_eff_t] ;
for i = 2 : N   
    w0 = rand(1,K) ;
    D = sum(w0) ;
    C_t_1 = C_t ;
    K_eff_t_1 = K_eff_t ;
    
    C_t = C_t_1*w_att + D ;    
    K_eff_t = (w_att*C_t_1/C_t)^2 *K_eff_t_1 + C_t^(-2)*sum(w0.^2) ;
    
    
    histor = [histor, 1/K_eff_t] ;
    C0 = C ;     
    C = C0*w_att + D ;   
    w = [w * C0*w_att , w0] /C ;    
%    t = 1 - exp(-i) ;
    W = [W*w_att,w0] ;    
    w_x = W / sum(W) ;

%    plot(w_x); msg = sprintf('N_{eff}=%g',1/sum(w_x.^2)) ; title(msg) ; drawnow ; %w_x.*[1:i]
    figure(1); plot(w); msg = sprintf('N_{eff}=%g',1/sum(w_x.^2)) ; title(msg) ; drawnow ; %w_x.*[1:i]
    figure(2); plot(histor) ;
    
end 


%%%%% Previous adaptation 
% function ikdeParams = ...
%           recalculateIkdeTmpPars( ikdeParams, model, obs_relative_weights )  
%       
% 
%       w_att = ikdeParams.suffSt.w_att ;
%       K_eff_t_1 = ikdeParams.suffSt.K_eff ;
%       C_t_1 = ikdeParams.suffSt.C_t ;
%       D = sum(obs_relative_weights) ;
%       
%       C_t = C_t_1*w_att + D ;
%       K_eff_t = (w_att*C_t_1/C_t)^2 *K_eff_t_1 + C_t^(-2)*sum(obs_relative_weights.^2) ;
%       
%           
%       obs_relative_weights = obs_relative_weights / sum(obs_relative_weights) ;
%                
%       nrml = length(ikdeParams.nonmod_alphas) + length(obs_relative_weights) ;
%       v1 =  length(ikdeParams.nonmod_alphas)/ nrml ;
%       v2 = length(obs_relative_weights)/ nrml ;  
%       
%       tmp_alphas = [ikdeParams.nonmod_alphas*v1, obs_relative_weights*v2] ;
%       if( abs(sum(tmp_alphas) - 1.0000) > 1e-8 )
%           error('Weights should sum to one!!') ;
%       end
%       ikdeParams.nonmod_alphas = tmp_alphas ;
%       
%       ikdeParams.N_eff = 1/sum(ikdeParams.nonmod_alphas.^2) ; 
%     
%       nrml = length(model.w) + length(obs_relative_weights) ;
%       v1 =  length(model.w)/ nrml ;
%       v2 = length(obs_relative_weights)/ nrml ;  
%       ikdeParams.obs_mixing_weights = obs_relative_weights ; 
%       ikdeParams.mix_weights = [v1 v2] ;
%        
% ----------------------------------------------------------------------- %
% 


