function alpha = optimizeWeights_MDL( f_ref_data_X, f_ref_data_Y, f_fit, f_ref )

len_fit = length(f_fit.weights) ;
num_data = cols(f_ref_data_X) ;
d = rows(f_fit.mu) ;

f_inflate_data_Y = inflateEstimate(f_ref_data_X, f_ref) ;
s = sqrt(mean((f_inflate_data_Y-f_ref_data_Y).^2)) ;

%t  = 0.001 ; bit = 32 ; theta = 0.1 ; 0.001*1/len_fit ;
% predvsem je obèutljiv na theta
N_b = 1 + d + d*d ; % complexity of basis function: w + mu + cov 
 t = 0.1 ; s = median(f_ref_data_Y)*0.1 ;  bit = 8 ; theta = 0.1*1/len_fit ;
% t  = 0.01 ; s = 0.0000001 ; bit = 32 ; theta = 0.001 ;0.001*1/len_fit ;
K1 = 1 ; K3 = 1 ; K2 = (log2(s^2*2*pi*exp(1)/t^2 )/log2(2))/(2*s^2)/bit ;

% get pointwise approximation and responsibility
[R, Y, norms] = getPartitionedResponsibility( f_fit.mu, f_fit.weights, f_fit.covariances, f_ref_data_X ) ;
for i = 1 : len_fit
    R(i,:) = R(i,:) > theta*norms(i) ;
end

% error matrix
err = (Y-f_ref_data_Y).^2 ;
Ceta = R.*repmat(err,rows(R),1) ;
C_ii = (K1*sum(R,2) - K2*sum(Ceta,2) - K3*N_b) ;

C = zeros(len_fit,len_fit) ;
for i = 1 : len_fit
    R_i = R(i,:) ;
    for j = i+1 : len_fit
        select = R_i.*R(j,:) ;
        C(i,j) = (-K1*sum(select) + K2*sum(err.*select))/2 ;
    end
end
C = (C + transpose(C) + diag(C_ii)) ;

i_max = length(f_fit.weights)*100 ;%length(f_fit.weights)*100 ; 
i_max = min(i_max,1000)  
L = ceil(i_max*0.2) ;
alpha = (f_fit.weights>mean(f_fit.weights)*0.0)  ; 
[optp,alpha] = tabuQO(alpha,C,i_max,L) ;
alpha = (alpha.*f_fit.weights)' ;
alpha = alpha / sum(alpha) ;


function f_inflate_data_Y = inflateEstimate(f_ref_data_X, f_fit)

f_fit.covariances = f_fit.covariances*1.1^2 ;
f_inflate_data_Y = evaluateDistributionAt( f_fit.mu, f_fit.weights, f_fit.covariances, f_ref_data_X ) ;


