%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function pdf_split = splitGaussianInTwoWider( new_mu, new_Cov, new_w, suffStatB, suffStatA, desiredComps ) 
% reduces the component to deinflated and verifies if it is a singleton
% component. If it is a singleton, then the output is just the (inflated)
% original component. If it is not a singleton, then it brakes down
% deinflated component into two parts and reinfaltes them back.

tolSingl = 1e-30 ;
% construct a component and determine if it is a singleton
pdf_split.Mu = new_mu ;
pdf_split.Cov = {new_Cov} ;
pdf_split.w = new_w ;
pdf_split.suffStat.A = suffStatA ;
pdf_split.suffStat.B = suffStatB ;

[ pdf_deinflated, H_opt ]= readjustKernels( pdf_split, new_Cov*0 ) ;
dim = size(new_mu,1) ;
% is the component a singleton?
if (abs(det(pdf_deinflated.Cov{1})))^(1/dim) < tolSingl
    % the component is a singleton and should not be broken any further
    % return the component as it is
    return ;
end

% if the component is not a singleton, generate a two-component equivalent
pdf_split = crumbleComponentNDim( 'dim', dim, 'desiredComps', desiredComps ) ;

% rotate and scale the crumbled prototype according to the reference kernel
    [U,S,V] = svd(pdf_deinflated.Cov{1}) ;
    F_trns = V*sqrt(S) ;
    for i = 1 : length(pdf_split.w)
        pdf_split.Cov{i} =  F_trns*pdf_split.Cov{i}*F_trns'  ;
        pdf_split.Mu(:,i) = F_trns*pdf_split.Mu(:,i) + pdf_deinflated.Mu ;                 
    end 
 pdf_split.w = pdf_split.w * new_w ;
 
 % manage sufficient statistics
 if isempty( suffStatB )
    suffStatB = new_Cov*0 ;
 end
        
 for i = 1 : length(pdf_split.w)
     pdf_split.suffStat.A(i) = {pdf_split.Cov{i} + pdf_split.Mu(:,i)*pdf_split.Mu(:,i)'} ;
     pdf_split.suffStat.B(i) = {suffStatB{1}*0} ;
 end
 
 % inflate the mixture back using H_opt
 pdf_split = readjustKernels( pdf_split, H_opt ) ;
 









% 
% len = size(new_Cov,1) ;
% [U,S,V] = svd(new_Cov) ;
% s = diag(S) ;
% [s , i] = max(s) ;
% 
% m = zeros(len,1) ;
% m(i) = 1 ;
% dmu = sqrt(s)*0.4*V*m ;
%  
% mu_s = [ new_mu + dmu, new_mu - dmu ] ;
% 
% Sc = new_Cov + new_mu*new_mu' - 0.5*(mu_s(:,1)*mu_s(:,1)' + mu_s(:,2)*mu_s(:,2)') ;
% 
% pdf_split.Mu = mu_s ;
% pdf_split.Cov = horzcat({Sc},{Sc}) ;
% pdf_split.w = ones(1,2)*new_w*0.5 ;