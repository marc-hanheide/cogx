function demoTcimm()

installDemoPaths() ;

% uniNorm1000a uniNorm1000b
                                                    
N = 1000 ;                                            % ,'LoadData', 'MaWaX'       
[xs, pdf] = generateValuesPdf( N, 'MaWa', 0, 'MyDi', 1  ,'LoadData', 'MaWaX' ) ; % , , 'LoadData', 'Sampli', 'PermuteData', 1, 'SaveData', 'MaWaX' , 'LoadData', 'MaWaX' ,'LoadData', 'MaWaX'
xs_t = xs ;
% [xs_t, pdf] = generateValuesPdf( N, 'MaWa', 0, 'MyDi', 0    ) ; % , , 'LoadData', 'Sampli', 'PermuteData', 1, 'SaveData', 'MaWaX' , 'LoadData', 'MaWaX' ,'LoadData', 'MaWaX'
 
% [pdf , Aic] = greedyEMTest( xs', xs_t' ) ;

MaxComponents = 10 ;
pdf = greedyEMfit( xs(1:5)', MaxComponents ) ;

clf; showdecomposedpdf( pdf ) ; hold on ; plot(xs, xs*0, 'b*') ;
return 

% xs = [1 2 2.5 2.6 2.7 2.8 2.9 3 3.1 3.2 3 2 2.1]
 
xs = (xs - min(xs)) ;
xs = (xs / (max(xs)-min(xs)))*1.8-0.8 ;
xs = sort(xs) ;
K = 10 ;
pdf = updateTcimm( [], xs(1:K) ) ;

% xs(K+1:length(xs)) = xs(randperm(length(xs)-K)+K) ;

for i = K+1 : length(xs)
    if ( i == 20 )
        sdf = 45 ;
    end
        
   pdf = updateTcimm( pdf, xs(i) ) ; 
   showCurrentResult( pdf, xs, i ) ; 
   title(sprintf('Data number: %d',i))
   drawnow ; pause(0.1) ;
end
 
%-----------------------------------------------%
function [pdf , Aic] = greedyEMTest( X, T, MaxComponents )

Aic = 1e20 ;
max_n = MaxComponents ;
for i = 1 : max_n
 max_k = max_n ;
 ncan = ceil((0.1 * length(X)/max_k)) ; 
 [W,M,R,Tlogl] = em(X,[],i,ncan,1,0);

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

newPath = '..\..\greedyEM\'; rmpath(newPath) ; addpath(newPath) ;
cd(newPath) ; installGEMPaths() ; cd(initial_path) ;

% newPath = 'D:\Work\Matlab\IncrementalKDE\EM_28_03_2003\' ; rmpath(newPath) ; addpath(newPath) ;

newPath = 'D:\Work\Matlab\IncrementalKDE\GaMMoT\aux_tools\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = 'D:\Work\Matlab\IncrementalKDE\GaMMoT\evaluation\'; rmpath(newPath) ; addpath(newPath) ;
% installIncrementalKDEUpdatePaths() ;
 
% cd .. ; cd .. ; 
% installGaMMotPaths() ;
% cd( initial_path ) ;
