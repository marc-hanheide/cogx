function I = debug_getIntSqrdHessian( pdf, varargin )
% Calculates an integral over the squared Hessian of a Gaussian mixture model.
% This version does not use any shortcurts -- it is used for checking validity 
% of the getIntSqrdHessian().
% pdf is composed of :
%     Cov ... covariances [N times, d rows, d columns]
%     Mu ... mean values [d rows, N columns] 
%     w ... component weights [d rows]
%     
% Follows Wand and Jones "Kernel Smoothing", page 101., assuming H=h*F.
% Matej Kristan 2008

I = NaN ;
if ( isempty(pdf.w) )
    return ;
end
% read dimension and number of components
[ d, N ]= size(pdf.Mu) ;
 
F = eye(d) ;
% process arguments
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'F', F = args{i+1} ;
    end
end

% create the lambda matrix and temporary matrices
L = zeros(N,N) ;
A = zeros(d) ; B = A ; C = A ;
 
% generate upper diagonal matrix
for l1 = 1 : N
    S1 = pdf.Cov{l1} ;
    Mu1 = pdf.Mu(:,l1) ;
    for l2 = 1 : N
        S2 = pdf.Cov{l2} ;
        Mu2 = pdf.Mu(:,l2) ;
        A = inv(S1 + S2) ;
        
        B = A*(eye(d) - 2*(Mu1-Mu2)*(Mu1-Mu2)'*A) ;
        C = A*(eye(d) - (Mu1-Mu2)*(Mu1-Mu2)'*A) ;
         
        f_t = normpdf(Mu1,Mu2,'inv',A) ;
                 
        c = 2*trace(F*A*F*B) + trace(F*C)^2 ;
        L(l2,l1) = f_t*c ;        
    end
end
 
% integral is
I = pdf.w*L*pdf.w' ;

