function [pdf, old_mix, ClAic] = greedyEMfit( X, MaxComponents, varargin )

displayFitting = 0 ;

referencePdf = [] ;
args = varargin;
nargs = length(args);
for i=1:2:nargs
    switch args{i}
        case 'displayFitting', displayFitting = args{i+1} ;
        case 'referencePdf', referencePdf = args{i+1} ;
    end
end

[W,M,R,Tlogl] = em(X,[],MaxComponents,[],displayFitting,0,referencePdf) ; 
S = MakeCovMatrix(M,R) ;
ClAic = calculateClAic(X, W, M, S )  ;
weights = W' ;
     mu = M' ;
     Covariances = S ;
pdf.mu = mu ;
pdf.weights = weights ;
pdf.covariances = Covariances ;

old_mix.W = W ;
old_mix.M = M ;
old_mix.R = R ;

return
T = [] ;
Aic = 1e20 ;
max_n = MaxComponents ;
for i = 1 : max_n
 max_k = max_n ;
 ncan = ceil((0.1 * length(X)/max_k)) ; 
 [W,M,R,Tlogl] = em(X,[],i,[],displayFitting,0);

 S = MakeCovMatrix(M,R) ;
 ClAic = calculateClAic(X, W, M, S )  ;
 
 if ( i == 1 || Aic > ClAic )
     Aic = ClAic ;
     weights = W' ;
     mu = M' ;
     Covariances = S ;
 end
end

pdf.mu = mu ;
pdf.weights = weights ;
pdf.covariances = Covariances ;

%[W,M,R,Tlogl] = em(X,T,max_k,ncan,1,0); 

% 
 

%-----------------------------------------------%
function showCurrentResult( pdf, xs, I )

pdf_c = pdf.pdf_c ;
pdf_c.covariances = reshape(pdf_c.covariances,length(pdf_c.weights),1) ;

pdf_h = pdf.pdf_h ;
pdf_h.covariances = reshape(pdf_h.covariances,length(pdf_h.weights),1) ;

figure(1) ; clf ; 
subplot(1,2,1) ;
plot(xs(1:I),xs(1:I)*0,'*b') ; plot(xs(I),0,'*r') ; hold on ;
showDecomposedPdf(pdf_h, 'enumComps' , 1) ; title('Historic') ;

subplot(1,2,2) ;
plot(xs(1:I),xs(1:I)*0,'*b') ; plot(xs(I),0,'*r') ; hold on ;
showDecomposedPdf(pdf_c,'enumComps' , 1) ; title('Current') ;

function installDemoPaths() 
% install Kernel learning
initial_path = pwd ; cd .. ;
cpath = cd ; rmpath(cpath) ; addpath(cpath) ;
cd(initial_path) ;

newPath = 'D:\Work\Matlab\IncrementalKDE\EM_28_03_2003\' ; rmpath(newPath) ; addpath(newPath) ;

newPath = 'D:\Work\Matlab\IncrementalKDE\GaMMoT\aux_tools\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = 'D:\Work\Matlab\IncrementalKDE\GaMMoT\evaluation\'; rmpath(newPath) ; addpath(newPath) ;
% installIncrementalKDEUpdatePaths() ;
 
% cd .. ; cd .. ; 
% installGaMMotPaths() ;
% cd( initial_path ) ;
