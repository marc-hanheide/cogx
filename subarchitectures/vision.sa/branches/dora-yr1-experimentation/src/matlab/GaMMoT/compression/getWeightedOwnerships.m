function [ownerships, norms] = getWeightedOwnerships( f_ref, seeds, f_dest )

if ( ~isempty(seeds) )
    [ownerships, norms] = getOwnershipsDot( f_ref, seeds, f_dest ) ;
else
    [ownerships, norms] = getOwnershipsHell( f_ref, f_dest ) ;
end

% ----------------------------------------------------------------------- %
function [ownerships, norms] = getOwnershipsDot( f_ref, seeds, weights )
% find ownerships of f_ref in terms of seeds

len_refs = length(f_ref.weights) ;
len_dest = length(seeds) ;
d = rows(f_ref.mu) ;
a1 = 2*sqrt(2*pi)^d ; 

f_ref.covariances = reshape(f_ref.covariances',d,d,len_refs) ;
 
ownerships = zeros(len_refs, len_dest) ;
for i_ref = 1 : len_refs
    C_ref = f_ref.covariances(:,:,i_ref) ;
    mu_ref = f_ref.mu(:,i_ref) ;
    w_ref = f_ref.weights(i_ref) ;

    ownerships(i_ref,:) = normpdf(seeds,mu_ref,[],C_ref) ;
end
norms = sum(ownerships,2) ;
ownerships = ownerships./repmat(norms,1,len_dest) ;
% each column is a set of nonnormalized weights to reweight the target
% distribution for the corresponding seed

% --------------------------------------------------------------------- %
function [ownerships, norms] =  getOwnershipsIntegratedLikelihood( f_ref, f_dest )
% given f_dest components, find ownerships for the components in f_ref
% by measuring int(p1*p2)dx .

len_refs = length(f_ref.weights) ;
len_dest = length(f_dest.weights) ;
d = rows(f_dest.mu) ;
a1 = sqrt(2*pi)^d ; 

f_ref.covariances = reshape(f_ref.covariances',d,d,len_refs) ;
f_dest.covariances = reshape(f_dest.covariances',d,d,len_dest) ;

ownerships = zeros(len_refs, len_dest) ;
for i_ref = 1 : len_refs
    C_ref = f_ref.covariances(:,:,i_ref) ;
    mu_ref = f_ref.mu(:,i_ref) ;
    w_ref = f_ref.weights(i_ref) ;
    
    for i_dest = 1 : len_dest
        w_dest = f_dest.weights(i_dest) ;
        mu_dest = f_dest.mu(:,i_dest) ;
        C_dest = f_dest.covariances(:,:,i_dest) ;
        ownerships(i_ref,i_dest) = w_dest*normpdf(mu_ref,mu_dest,[],C_ref+C_dest) ;        
    end
    [i,j]= max(ownerships(i_ref,:)) ;
    ownerships(i_ref,:) = ownerships(i_ref,:)*0 ; 
    ownerships(i_ref,j) = 1 ;    
end
 
norms = sum(ownerships,2) ;
ownerships = ownerships./repmat(norms,1,len_dest) ;
 
% --------------------------------------------------------------------- %
function [ownerships, norms] =  getOwnershipsHell( f_ref, f_dest )
% given f_dest components, find ownerships for the components in f_ref
% by measuring int(p1*p2)dx .

len_refs = length(f_ref.weights) ;
len_dest = length(f_dest.weights) ;
d = rows(f_dest.mu) ;
a1 = sqrt(2*pi)^d ; 

f_ref.covariances = reshape(f_ref.covariances',d,d,len_refs) ;
f_dest.covariances = reshape(f_dest.covariances',d,d,len_dest) ;

ownerships = zeros(len_refs, len_dest) ;
for i_ref = 1 : len_refs
    C_ref = f_ref.covariances(:,:,i_ref) ;
    mu_ref = f_ref.mu(:,i_ref) ;
    w_ref = f_ref.weights(i_ref) ;
    
    for i_dest = 1 : len_dest
        w_dest = f_dest.weights(i_dest) ;
        mu_dest = f_dest.mu(:,i_dest) ;
        C_dest = f_dest.covariances(:,:,i_dest) ;
        ownerships(i_ref,i_dest) = ...
                           a1*(det(C_ref)*det(C_dest))^(1/4) * normpdf( mu_ref, f_dest.mu(:,i_dest),[],2*(C_ref+C_dest)) ;
        
    end
    [i,j]= max(ownerships(i_ref,:)) ;
    ownerships(i_ref,:) = ownerships(i_ref,:)*0 ; 
    ownerships(i_ref,j) = 1 ;    
end
 
norms = sum(ownerships,2) ;
ownerships = ownerships./repmat(norms,1,len_dest) ;

