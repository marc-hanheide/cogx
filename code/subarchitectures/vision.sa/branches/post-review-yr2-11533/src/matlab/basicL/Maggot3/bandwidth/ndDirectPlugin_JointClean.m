%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2010 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2010
%%
function H = ndDirectPlugin_JointClean( Mu, Cov, w, Cov_smp, N_eff )                           
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
d = size(Mu,1) ;
% [new_mu, Cov_smp, w_out] = momentMatchPdf(pdf.Mu, pdf.ps.Cov, pdf.w) ;

% test
% for i = 1 : length(Cov)
%     Cov{i} = diag(diag(Cov{i})) ;
% end
%end test
 
G = (Cov_smp *(4/((d+2)*N_eff))^(2/(d+4))) ; %*0.8^2 ;
F = Cov_smp/det(Cov_smp)^(1/d) ; % for numerical stability. it could have been: F = Cov_smp ;

applyapproximation = 0 ;

if applyapproximation == 1
    % uses approximation !
    % test
    Ccov = zeros(size(Mu,1),size(Mu,2)) ;
    for i = 1 : length(Cov)
        Ccov(:,i) = diag(Cov{i})' ;
    end
    g = (4/((d+2)*N_eff))^(2/(d+4)) ;
    Rf2 = mex_getIntSquaredHessian(Mu, w, Ccov, g) ;
else
    Rf2 = getIntSquaredHessian( Mu, w, Cov, F, G ) ;
end
% if abs(Rf2-R_x) > 1e-3
%     df = 34 ;
% end
% [Rf2, R_x]
%end test


h_amise = (N_eff^(-1) *det(F)^(-1/2) /( sqrt(4*pi)^d * Rf2 * d ))^(1/(d+4)) ;
H = getBWfromStruct( F, h_amise) ; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 





