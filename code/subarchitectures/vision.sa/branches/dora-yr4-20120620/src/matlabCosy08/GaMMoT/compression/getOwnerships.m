function ownerships = getOwnerships( f_ref, f_dest )

ownerships = getOwnershipsHell( f_ref, f_dest ) ;


function ownerships = getOwnershipsDot( f_ref, f_dest )
% given f_dest components, find ownerships for the components in f_ref

len_refs = length(f_ref.weights) ;
len_dest = length(f_dest.weights) ;
d = rows(f_dest.mu) ;
a1 = 2*sqrt(2*pi)^d ; 

f_ref.covariances = reshape(f_ref.covariances',d,d,len_refs) ;
f_dest.covariances = reshape(f_dest.covariances',d,d,len_dest) ;

ownerships = zeros(1,len_refs) ;
dist = zeros(1,len_dest) ;
for i_ref = 1 : len_refs
    C_ref = f_ref.covariances(:,:,i_ref) ;
    mu_ref = f_ref.mu(:,i_ref) ;
    w_ref = f_ref.weights(i_ref) ;
    
    dist = normpdf(f_dest.mu,mu_ref,[],C_ref) ;
    
    [a,id] = max(dist) ;
    ownerships(i_ref) = id ;
end



function ownerships = getOwnershipsHell( f_ref, f_dest )
% given f_dest components, find ownerships for the components in f_ref

len_refs = length(f_ref.weights) ;
len_dest = length(f_dest.weights) ;
d = rows(f_dest.mu) ;
a1 = 2*sqrt(2*pi)^d ; 

f_ref.covariances = reshape(f_ref.covariances',d,d,len_refs) ;
f_dest.covariances = reshape(f_dest.covariances',d,d,len_dest) ;

ownerships = zeros(1,len_refs) ;
dist = zeros(1,len_dest) ;
for i_ref = 1 : len_refs
    C_ref = f_ref.covariances(:,:,i_ref) ;
    mu_ref = f_ref.mu(:,i_ref) ;
    w_ref = f_ref.weights(i_ref) ;
    for i_dest = 1 : len_dest
        w_dest = f_dest.weights(i_dest) ;
        C_dest = f_dest.covariances(:,:,i_dest) ;
        dist(i_dest) = 1 - a1*(det(C_ref)*det(C_dest))^(1/4) * normpdf( mu_ref, f_dest.mu(:,i_dest),[],2*(C_ref+C_dest)) ;
        
%         dist(i_dest) = w_ref + w_dest - 2*sqrt(w_ref*w_dest)*a1*(det(C_ref)*det(C_dest))^(1/4) * normpdf( mu_ref, f_dest.mu(:,i_dest),[],2*(C_ref+C_dest)) ;
    end
    [a,id] = min(dist) ;
    ownerships(i_ref) = id ;
end