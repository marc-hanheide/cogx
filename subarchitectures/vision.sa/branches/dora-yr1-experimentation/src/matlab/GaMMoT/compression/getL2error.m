function er = getL2error( f_ref, f_fit ) 

len_fit = cols(f_fit.mu) ;
d = rows(f_fit.mu) ;
%C = zeros(len_fit, len_fit) ;
C = 0 ;
c1 = 0 ; c0 = 0 ;
for i = 1 : len_fit
    Cov1 = reshape(f_fit.covariances(i, :),d,d) ;
    c1 = c1 + f_fit.weights(i)^2*integOfTwoGaussProd( f_fit.mu(:,i), Cov1, f_fit.mu(:,i), Cov1 ) ;
    for j = i+1 : len_fit
        Cov2 = reshape(f_fit.covariances(j, :),d,d) ;
        c0 = c0 + f_fit.weights(i)*f_fit.weights(j)*...
                    integOfTwoGaussProd( f_fit.mu(:,i), Cov1, f_fit.mu(:,j), Cov2 ) ;
    end
end
C = c1 + 2*c0 ;
 
Pp = 0 ;
len_ref = length(f_ref.weights) ;
for i = 1 : len_fit
    Cov1 = reshape(f_fit.covariances(i, :),d,d) ;
    Mu1  = f_fit.mu(:,i) ;
    p = 0 ;
    for j = 1 : len_ref
        Cov2 = reshape(f_ref.covariances(j, :),d,d) ;
        Mu2  = f_ref.mu(:,j) ; 
        p = p + f_ref.weights(j)*integOfTwoGaussProd( Mu1, Cov1, Mu2, Cov2 ) ;
    end
    Pp = Pp + f_fit.weights(i)*p ;
end

K = 0 ;
k1 = 0 ; k0 = 0 ;
for i = 1 : len_ref
    Cov1 = reshape(f_ref.covariances(i, :),d,d) ;
    k1 = k1 + f_ref.weights(i)^2*integOfTwoGaussProd( f_ref.mu(:,i), Cov1, f_ref.mu(:,i), Cov1 ) ;
    for j = i+1 : len_ref
        Cov2 = reshape(f_ref.covariances(j, :),d,d) ;
        k0 = k0 + f_ref.weights(i)*f_ref.weights(j)*...
                    integOfTwoGaussProd( f_ref.mu(:,i), Cov1, f_ref.mu(:,j), Cov2 ) ;
    end
end
K = k1 + 2*k0 ;


er = C-2*Pp + K ;  
