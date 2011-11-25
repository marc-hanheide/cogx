function f0 = refitGaussianMixtureFunApprox( f0, f_ref )
%
% 1D Gaussian fitting of a single Gaussian in f0 to
% the reference mixture of Gaussians in f_ref. Note that the weights of 
% f_ref need not sum to one; in that case, f0 will have a weight different
% of one.
%
% For theory, see: Zhang and Kwok, "Simplifying mixture models through
% function approximation".
%
%
%

dShow = 0 ;

% for testing
% f0.mu = f_ref.mu(:,1) ;
% f0.weights = f_ref.weights(1) ;
% f0.covariances = f_ref.covariances(1,:) ;
% end for testing

% modify allocation of covariance matrices
d = rows(f_ref.mu) ;
l_refs = cols(f_ref.weights) ;
% f_ref.covariances = reshape(f_ref.covariances',d,d,l_refs) ;

d = rows(f0.mu) ; l_mod = cols(f0.weights) ;
% f0.covariances = reshape(f0.covariances',d,d,l_mod) ;

% select the component to optimize
i = 1 ;
mu_comp = f0.mu ;
C_comp = f0.covariances ;
w_comp = f0.weights ;

% costruct target pdf
f_t = f_ref ;
len_target = cols(f_t.weights) ;

% allocate the covariance matrix structure
C_len = l_refs+l_mod-1 ;
C_inv = zeros(d,d,C_len) ;
C_det = zeros(1,C_len) ;


% initialize mean, covariance and weight of the new component
t1 = mu_comp ;
C1 = C_comp;
w1 = w_comp ;

% initialize precalculated arrays
for i = 1 : C_len
    C_inv(:,:,i) = inv( C1 + f_t.covariances(:,:,i) ) ;
    inv_sqrt_C_det(i) = sqrt(det(C_inv(:,:,i))) ;
end

% debugShow( f_ref, t1, C1, w1 ) ;
 
% optimize the component
t_init = zeros(d,1) ;
C_init = zeros(d,d) ;
optStage = 1 ;

minValueAtZero = 1e-5 ; 
C_atError = 1 ; 

maxIterations =  5 ;

for G_iteration = 1 : maxIterations   
    if ( optStage == -1 ) break ; end
        
    switch optStage
        case 1 % optimizing mean value
            for repeat = 1 : 1
            
            x0 = t_init ;
            B_norm = C_init ;
            for j = 1 : len_target
               C_j_inv = C_inv(:,:,j) ; 
               k_1j = exp(-0.5*sqdist(t1,f_t.mu(:,j),C_j_inv)) ;
               A_1j = f_t.weights(j)*C_j_inv *inv_sqrt_C_det(i) ;
               B_norm = B_norm + A_1j*k_1j ;
               x0 = x0 + A_1j*k_1j* f_t.mu(:,j) ;                
            end
            t1_new = inv(B_norm)*x0 ;
            t1 = t1_new ;
            
            if dShow == 1 
             debugShow( f_ref, t1, C1, w1 ) ; drawnow ;
            end
            end
            % switch to new stage
            optStage = 2 ;
        case 2 % optimizing covariance  
            B_norm = C_init ;
            B_main = C_init ;
            for j = 1 : len_target
%                 C_j_inv0 = inv( C1 + f_t.covariances(:,:,j) ) ;
%                 inv_sqrt_C_det0 = sqrt(det(C_j_inv0)) ; 

                % read stored inverses and determinants                
                C_j_inv0 = C_inv(:,:,j) ;
                inv_sqrt_C_det0 = inv_sqrt_C_det(j) ;
       
                % evaluate other parts
                k_1j = exp(-0.5*sqdist(t1,f_t.mu(:,j),C_j_inv0)) ;
                A_1j = f_t.weights(j)*C_j_inv0 *inv_sqrt_C_det0 ;
                
                B_norm = B_norm + A_1j*k_1j ;
                X = (f_t.mu(:,j) - t1)*transpose(f_t.mu(:,j) - t1) ; 
                B_main = B_main + A_1j*k_1j*( f_t.covariances(:,:,j) + 2*X*C_j_inv0*C1 ) ;
            end
            % avoid singularities at division by zero
            if (abs(det(B_norm)) <= minValueAtZero | abs(det(B_main)) <= minValueAtZero )
               C1_new = C_atError ; 
               C1 = C1_new ;
               w1 = 0 ; break ;
            else
                % calculate new covariance matrix
                C1_new = inv(B_norm)*B_main ;
            end
      
            
%               disp('Matrices'),[B_norm, B_main, C1_new]
%              disp('Determinants'),[det(B_norm), det(B_main)]
          
            % make sure that the matrices are positive definite
            C1_new = chol(C1_new'*C1_new) ;
           
            
            C1 = C1_new ;
            
            if dShow == 1
                debugShow( f_ref, t1, C1, w1 ) ; drawnow ;
            end
            
            % switch to new stage
            optStage = 3 ;
        case 3 % optimizing weight
            w1 = 0 ;
            for j = 1 : len_target
                C_j_inv0 = inv( C1 + f_ref.covariances(:,:,j) ) ;
                inv_sqrt_C_det0 = sqrt(det(C_j_inv0)) ; 
                
                % store partial computations
                C_inv(:,:,j) = C_j_inv0 ;
                inv_sqrt_C_det(j) = inv_sqrt_C_det0 ;
                
                k_1j = exp(-0.5*sqdist(t1,f_t.mu(:,j),C_j_inv0)) ;
                w1 = w1 + f_t.weights(j)*k_1j*inv_sqrt_C_det0 ;                                               
            end
            w1 = abs(w1 * sqrt(det(2*C1))) ;
           
            % perhaps it would be wise to remove the component if it
            % recieves a very small weight -- currently this seems to be
            % the only source of the singularities
            
            if dShow == 1
                debugShow( f_ref, t1, C1, w1 ) ; drawnow ;
            end
           % component should be removed or rejouvinated
           if ( abs(w1) <= minValueAtZero )
               break ;
           end
            % switch to new stage
            optStage = 1 ;           
    end   
end

f0.mu = t1 ;
f0.covariances = C1 ;
f0.weights = w1 ;



% --------------------------------------------------------------------- %
function debugShow( f_ref, t1, C1, w1 )

clf ;
f_ref.covariances = reshape(f_ref.covariances, length(f_ref.weights),1) ;
f_approx1.mu = t1 ;
f_approx1.weights = w1 ;
f_approx1.covariances = reshape(C1,1,1) ;
returnBounds = showDecomposedPdf( f_ref, 'linTypeSum', 'b', 'linTypeSub', 'b--' ) ;
showDecomposedPdf( f_approx1, 'bounds', returnBounds ) ;