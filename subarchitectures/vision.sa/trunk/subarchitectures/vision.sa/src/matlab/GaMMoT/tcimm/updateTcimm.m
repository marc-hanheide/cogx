function pdf = updateTcimm( pdf, x )

oneToOneCoherence = 0 ;
if ( isempty(pdf) )
    pdf = initializePdf( x ) ;
    return ;
end

pdf_c = pdf.pdf_c ; % current mixture
pdf_h = pdf.pdf_h ; % historical mixture

if ( pdf_c.N == 310 )
    df = 56 ;
end

% 1. fixed-complexity update the mixture model
pdf_c_updated = fixedComplexityUpdate( pdf_c, x ) ;

% if ( pdf_n.N == 10 ) 
%     sdf = 56 ;
% end
% 

% 2. compute difference components and mix them with historical mixture
pdf_split = splitCurrentUsingHistoric( pdf_c_updated, pdf_h ) ;

% 3. iteratively combine pairs of components
% if ( pdf_c.N < 120 )
    [pdf_c_new, oneToOneCoherence] = combineComponents( pdf_split ) ;
% else
%     [pdf_c_new, oneToOneCoherence] = combineComponentsFullMDL( pdf_split, pdf_split ) ; % then change first to updated
% end

% 4. if the new model is not coherent with the historic any more, replace
% the historic model.
if ( oneToOneCoherence ~= 1 )
    pdf_h = pdf_c_new ;
end

pdf.pdf_c = pdf_c_new ;
pdf.pdf_h = pdf_h ;


% ----------------------------------------------------- %
function pdf = initializePdf( x )

mu = mean(x) ;
C = cov(x)*cols(x)/( cols(x)- 1 ) ;
w = 1 ;

pdf_c.mu = mu ;
pdf_c.covariances = C ;
pdf_c.weights = 1 ;
pdf_c.N = length(x) ;
pdf_c.E = pdf_c.weights*pdf_c.N ;

pdf_h = pdf_c ;
pdf.pdf_c = pdf_c ;
pdf.pdf_h = pdf_h ;


% ------------------------------------------------------ %

function pdf_n = splitCurrentUsingHistoric( pdf_c, pdf_h ) 
% consecutive splits
 


boundNewSigma =  0.1 ; 0.3; 0.4 ;
boundWeightNewComponent =   0.1 ; 0.3;0.2 ;
 
E_common = [] ;
alpha_common = [] ; 
mu_common = [] ;
C_common = [] ;
tol = 1e-20 ;

len = length(pdf_c.weights) ;
for i = 1 : len 
    N_i = pdf_c.N - pdf_h.N ;
    E_i = pdf_c.E(i) - pdf_h.E(i) ;
    alpha_i = E_i/N_i ;
    
    mu_i_first = pdf_h.mu(i) ;
    E_i_first = pdf_h.E(i) ;
    covar_i_first = pdf_h.covariances(:,:,i) ;
    
    C_c = pdf_c.covariances(:,:,i) ;
    C_h = pdf_h.covariances(:,:,i) ;
      
%    [ (pdf_c.E(i) - pdf_h.E(i))/(pdf_c.E(i) + pdf_h.E(i)), (pdf_c.E(i) - pdf_h.E(i))/(pdf_c.E(i) + pdf_h.E(i)) < boundWeightNewComponent ]
    
    if ( (pdf_c.E(i) - pdf_h.E(i))/(pdf_c.E(i) + pdf_h.E(i)) < boundWeightNewComponent  )  % (alpha_i <= tol  || E_i < 0.2*pdf_h.E(i)) % was E_i < 0.05*pdf_h.E(i)
        alpha_i = 0 ; 
        
        mu_i_first = pdf_c.mu(:,i) ;
        E_i_first =  pdf_c.E(i) ;
        covar_i_first = pdf_c.covariances(:,:,i) ;
 
        mu_i_second = pdf_c.mu(:,i) ;
        E_i_second = alpha_i*N_i ; 
        covar_i_second = pdf_c.covariances(:,:,i) ;
    else  
        mu_i = (pdf_c.mu(:,i)*pdf_c.E(i) - pdf_h.mu(:,i)*pdf_h.E(i))/( pdf_c.E(i) - pdf_h.E(i) ) ;
        
        
%             C_i = ( C_c*pdf_c.E(i) -(C_h+pdf_h.mu(:,i)*pdf_h.mu(:,i)')*pdf_h.E(i) + ...
%                     (pdf_h.mu(:,i)*pdf_c.mu(:,i)' + pdf_c.mu(:,i)*pdf_h.mu(:,i)')*pdf_h.E(i) - pdf_c.mu(:,i)*pdf_c.mu(:,i)'*pdf_c.E(i) )/ ...
%                   (pdf_c.E(i) - pdf_h.E(i)) + ...
%                   mu_i*pdf_c.mu(:,i)' + pdf_c.mu(:,i)*mu_i' - mu_i*mu_i' ;
        
        mu_h = pdf_h.mu(:,i) ; mu_c = pdf_c.mu(:,i) ; mu_d = mu_i ;
        C_i = ((C_c - mu_c*mu_c')*pdf_c.E(i) + pdf_h.E(i)*( -C_h - mu_h*mu_h' + mu_h*mu_c' + mu_c*mu_h' ))/(pdf_c.E(i) - pdf_h.E(i)) -...
            mu_d*mu_d' + mu_d*mu_c' + mu_c*mu_d' ;
%         C_i = abs(C_i) 
           
        mu_i_first = pdf_h.mu(:,i) ;
        E_i_first =  pdf_h.E(i) ;
        covar_i_first = pdf_h.covariances(:,:,i) ;
 
        mu_i_second = mu_d ;
        E_i_second = pdf_c.E(i) - pdf_h.E(i) ; % alpha_i*N_i ; 
        covar_i_second = C_i ;
    end
    
    if sqrt(det(covar_i_second)) <= boundNewSigma*sqrt(det(pdf_c.covariances(:,:,i)))
        alpha_i = 0 ; 
        
        mu_i_first = pdf_c.mu(:,i) ;
        E_i_first =  pdf_c.E(i) ;
        covar_i_first = pdf_c.covariances(:,:,i) ;
 
        mu_i_second = pdf_c.mu(:,i) ;
        E_i_second = alpha_i*N_i ; 
        covar_i_second = pdf_c.covariances(:,:,i) ; 
    end
 
    E_common = [E_common, [E_i_first, E_i_second]] ;
    alpha_common = [alpha_common , [E_i_first/pdf_c.N, E_i_second/pdf_c.N  ]] ;
    mu_common = [mu_common , [mu_i_first, mu_i_second]] ; % pdf_c.mu(:,i) mu_i_first
    C_common = cat(3, C_common, covar_i_first) ;
    C_common = cat(3, C_common, covar_i_second) ;
    
%     if ( alpha_i == 0 )
%         mu_common = [mu_common , [pdf_c.mu(:,i) , mu_i]] ; % pdf_c.mu(:,i) mu_i_first
%         C_common = cat(3, C_common, pdf_c.covariances(:,:,i)) ;
%     end
%     C_common = cat(3, C_common, C_i) ;
end
 
% pdf_n.E = [pdf_h.E, E_new] ;
% pdf_n.mu = [pdf_h.mu, mu_new] ;
% pdf_n.covariances = cat(3,pdf_h.covariances,C_new) ;
% pdf_n.weights = [pdf_h.weights - alpha_new, alpha_new] ;

% regularize alphas
if ( sum(alpha_common) ~= 1 )
    sdfg = 56 ;
end
 
alpha_common = alpha_common / sum(alpha_common) ;
E_common = alpha_common*pdf_c.N ;
% sum(alpha_common)

pdf_n.E = E_common;
pdf_n.mu = mu_common ;
pdf_n.covariances  = C_common ;
pdf_n.weights = alpha_common ;
pdf_n.N = pdf_c.N ;


% ------------------------------------------------------ %
function pdf_n = fixedComplexityUpdate( pdf_c, x ) 

N = pdf_c.N ;
N_new = N + 1 ;
P = getProbCompCondOnData( pdf_c , x ) ;

alpha_new = (pdf_c.E + P)/(N + 1) ;

mu_new = pdf_c.mu*0 ;
C_new = pdf_c.covariances*0 ;

len = length(pdf_c.weights) ;
for i = 1 : len
    mu_new(:,i) = (pdf_c.mu(:,i)*pdf_c.E(i) + x*P(i))/(pdf_c.E(i) + P(i)) ;
    C_c = pdf_c.covariances(:,:,i) ;
    mu_c = pdf_c.mu(:,i) ;
    mu_n = mu_new(:,i) ;
    C_new(:,:,i) = ((C_c + mu_c*mu_c' - mu_c*mu_n' - mu_n*mu_c' + mu_n*mu_n')*pdf_c.E(i) + (x-mu_n)*(x-mu_n)'*P(i))/ ( pdf_c.E(i) + P(i) ) ;
    
%     
%     C_new(:,:,i) = ( (pdf_c.covariances(:,:,i) + (pdf_c.mu(:,i)-mu_new(:,i))*(pdf_c.mu(:,i)-mu_new(:,i))')*pdf_c.E(i) +...
%                     (x - mu_new(:,i))*(x - mu_new(:,i))'*P(i)) / ( pdf_c.E(i) + P(i) );
end

pdf_n.weights = alpha_new ;
pdf_n.covariances = C_new ;
pdf_n.mu = mu_new ;
pdf_n.E = pdf_c.E + P ;
pdf_n.N = N_new ;


% ------------------------------------------------------ %
function P = getProbCompCondOnData( pdf_c , x )

tol = 0 ;
len = length(pdf_c.weights) ;
p_i = zeros(1,len) ;
for i = 1 : len
    p_i(i) = normpdf(x,pdf_c.mu(:,i),[],pdf_c.covariances(:,:,i)) ;
end

if ( sum(p_i) <= tol ) p_i = p_i + 1e-5 ; end

P = p_i.*pdf_c.weights ;
P = P / sum(P) ;


