function demo_MW_compression() 
installdemo_MW_compressionPaths() ;

% N = 200 ;
% Y = [-5.0:0.01:5.0] ; 
% [actual_density,x_pos] = marron_wand_normal_mixtures(3,Y,N) ;
% figure(1); clf; plot(Y,actual_density) ; hold on ;
% 
% pdf_pos = constructKDEfromData( x_pos, 'compression', 0 ) ;
% h = get1DKernelBWbatch( pdf_pos.mu )  ;
% pdf_pos.covariances = pdf_pos.covariances*0 + h^2 ;
% 
% showPdfAutoBounds( N, pdf_pos.mu, pdf_pos.covariances, pdf_pos.weights, 'b--' );
% 
% return ;
f1_mix = getMixture( 29 ) ;
f2_mix = getMixture( 2 ) ;

f1_mix.mu = f1_mix.means ;
f1_mix.covariances = f1_mix.covariances' ;
f2_mix.mu = f2_mix.means ;
f2_mix.covariances = f2_mix.covariances' ;

pdf_pos = combineDistributions( f1_mix, f2_mix ) ;
showme(pdf_pos, pdf_pos, pdf_pos, 1) ;

f1 = compressDistribution( pdf_pos, 'showIntermediate', 0 ) ;



showme(pdf_pos, f1, f1, 1) ; 

showme(pdf_pos, f1, pdf_pos, 2) ; 

% ----------------------------------------------------------------------- %
function showme(f_ref, f1_mix, f_n, fignum)
figure(fignum); clf; hold on ;

b1 = sqrt(max([f1_mix.covariances;f_ref.covariances])) ;
bmin = min([f1_mix.mu,f_ref.mu]) - b1*5 ;
bmax = max([f1_mix.mu,f_ref.mu]) + b1*5 ;
bounds = [bmin,bmax] ;


for i = 1 : length(f_n.weights)
   showPdf( bounds, 1000, f_n.mu(:,i), f_n.covariances(i,:), f_n.weights(i), 'k',1 ) ; 
end
showPdf( bounds, 1000, f_ref.mu, f_ref.covariances, f_ref.weights, 'r', 2 ) ;
showPdf( bounds, 1000, f1_mix.mu, f1_mix.covariances, f1_mix.weights, 'b--',2 ) ;

msg = sprintf('Reference (red): %d modes, Result (blue): %d modes',...
              length(f_ref.weights), length(f1_mix.weights)) ; title(msg)
drawnow ;

% ----------------------------------------------------------------------- %
function y_evals = showPdf( bounds, N,centers, covariances, weights, color, ln )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
plot ( x_evals, y_evals, color,'LineWidth',ln )

% ------------------------ %
function installdemo_MW_compressionPaths() 
% install Kernel learning
newPath = '..\incrementalPDF\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\aux_tools\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\KDEconstructionTools\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = 'c:\Program Files\MATLAB704\work\IncrementalKDE\optimalIncrementalKDE\' ; 
rmpath(newPath) ; addpath(newPath) ;

% install tools
initial_path = pwd ;
cd .. ; 
installGaMMotPaths() ;
cd( initial_path ) ;