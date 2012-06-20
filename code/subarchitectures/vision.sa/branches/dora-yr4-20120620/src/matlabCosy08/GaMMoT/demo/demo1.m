function demo1()
%close all

global h_plugin ;

export_to_movie = 0 ;
aviobj = initializeMovie( export_to_movie, 'C:\incremental_mix.avi') ;
dat_scale = 1 ;

N=1000;
bounds = [-1 1 -1 1]*dat_scale ;
int = [bounds(1):(bounds(2)-bounds(1))/100:bounds(2)] ;
prior_adapt = 0.5 ;
nMaxComponents = 20 ;
C_sensor = 0.1^2 ;

%install the paths required for demonstration
installDemo1Paths() ;
 %newPath = 'C:\Program Files\MATLAB704\work\IncrementalKDE\optimal_bw_dll\' ; rmpath(newPath) ; addpath(newPath) ;
%data
xs=(rand(1,N)-1)*dat_scale;
 
% x1 = randn(1,floor(N/3))*0.1 - 0.3 ;
% x2 = randn(1,floor(N/3))*0.1 + 0.3 ;
% x3 = randn(1,floor(N/3))*1 ;
% xs = [x1, x2, x3] ;


x1=(rand(1,N*3/10)-1) ;
x2 = randn(1,floor(N*7/10))*0.1 + 0.5 ;
xs = [x1, x2 ] ;

N = length(xs) ;
I = randperm(N) ;
xs = xs(I) ;
 
fname = ['uniGauss','.txt'] ; % gaussMixture uniGauss uniform
%    save(fname,'xs','-ascii') ;
 xs = load(fname)  ;
N = length(xs) ;
%
xs(1) = -0.5 ; xs(2) = +0.5 ;

D = abs(xs(1) - xs(2))
figure(5); m = sprintf('%f',D) ; title(m)
%xs(3) = xs(2) ;
% xs(4) = xs(5) ;


%xs = [-1:1/N:-1/N] ;
ys=(rand(1,N)*2-1)*dat_scale;
X=[xs;ys];


%figure(3); hist(xs,20);

%figure
figure(1);clf

subplot(2,2,1);
title('Input data');
line([0,0],[-1,1]*dat_scale,'color','g');
xlabel('x');ylabel('y');
axis([-1,1,-1,1]*dat_scale);
hold on;

subplot(2,2,2);
title('Single Gaussian');
line([0,0],[-1,1]*dat_scale,'color','g');

subplot(2,2,3);
title('1-D Kernel density - incremental');
line([0,0],[-1,1]*dat_scale,'color','g');


subplot(2,2,4); 
[hst, hst_x] = hist(xs,10) ; hst = hst / sum(hst) ; bar(hst_x,hst) ; axis tight ; ca=axis;
line([0,0],[0,ca(4)],'color','g');
axis([[-1,1]*dat_scale,0,ca(4)]);
title('1-D Kernel density - batch');

%init single Gaussian
mu = X(1,1); sg = [0]; n = 1;
[mu,sg,n] = updateMV(X(1,2),mu,sg,n);
% initialize kernel
C0 = getSilvermanBWfromGaussian( sg, n ) ; % *0.5^2

% example of how to get optimally estimated pdf
pdf = get_1d_OptimalKDE( X(1,1:2) ) ;
% showDecomposedPdf( pdf ) ;
C0 = pdf.covariances(1) ;
C0 = C0 + C_sensor ;

kernel_pdf.mu = X(1,1:2) ;
kernel_pdf.weights = [1 1] ; 
kernel_pdf.weights = kernel_pdf.weights / sum(kernel_pdf.weights) ;
kernel_pdf.covariances = [1; 1]*C0 ;
kernel_pdf.components = 2 ;
kernel_x0 = mu ; % initial point used for initialization if required
kernel_pdf.H0 = -1 ;
kernel_pdf.H0 = C0 ;
 
subplot(2,2,1);
scatter (X(1,1),X(2,1),'b.');

H = [] ;

s1 = 1 ;
%incremental
for i=3:N
   aviobj = recordImage( export_to_movie, aviobj ) ;
 
   x=X(1,i);
   [mu,sg,n] = updateMV(x',mu,sg,n);
 
   
  h_fast = slow_univariate_bandwidth_estimate_STEPI(i,X(1,1:i)) ;
 %  h_plugin = h_fast ; 
   
%    prior_adapt = 0.4*(exp(-i/10))+0.5  
   % update current KDE 
   kernel_pdf = updateKDE( kernel_pdf, kernel_x0, x, sg, n, prior_adapt, 1, C_sensor ) ;
   
  H = [H, sqrt([h_fast^2;h_plugin.^2])] ;
   
   % compress distribution if the number of components exceeds "nMaxComponents"
   if ( length(kernel_pdf.weights) > nMaxComponents )
       H0 = kernel_pdf.H0 ;
       components = kernel_pdf.components ;
       tic
         kernel_pdf = compressDistribution( kernel_pdf, 'showIntermediate', 0) ;
       toc
       kernel_pdf.components = components ;
       kernel_pdf.H0 = H0 ;
   end
 
   figure(1)                                 
   subplot(2,2,1);
   scatter(X(1,i),X(2,i),'b.');

   % draw Gaussian
   subplot(2,2,2);
   y=normpdf(int,mu(1),sqrt(sg(1)));
   plot(int,y);
   ca=axis;
   line([0,0],[0,ca(4)],'color','g');
   axis([[-1,1]*dat_scale,0,ca(4)]);
   title('Single Gaussian');

   % draw Kernel-based estimate
   subplot(2,2,3) ; hold off ; plot(0,0,'.'); 

%   figure(2); clf
   showPdfEstimateResults( [], [], kernel_pdf.mu, kernel_pdf.weights, kernel_pdf.covariances, bounds, [], [] ) ;
   ca=axis;
   line([0,0],[0,ca(4)],'color','g');
   axis([[-1,1]*dat_scale,0,ca(4)]);
   msg = sprintf('1-D incremental KDE,\n N_{components}=%d', length(kernel_pdf.weights)) ; 
   title(msg);

   subplot(2,2,4); 
   [hst, hst_x] = hist(xs(1:i),10) ; hst = hst / sum(hst) ; bar(hst_x,hst) ; axis tight ; ca=axis;
   line([0,0],[0,ca(4)],'color','g');
   axis([[-1,1]*dat_scale,0,ca(4)]);
   msg = sprintf('1-D histogram on %d observed data', i) ; title(msg)
   
   drawnow;
   
   
   figure(3) ; clf ; hold on ;
   plot([1:length(H(1,:))],H(1,:),'r') ;
   plot([1:length(H(2,:))],H(2,:),'g') ;
   if rows(H)> 2 plot([1:length(H(3,:))],H(3,:),'b') ; end
   drawnow ;
    
%    F = [20,120,500] ;
%    if ( sum(i==F)> 0 ) 
%        showCurrentResult(kernel_pdf, xs(1:i), 2);
%        pause ; 
%    end
   
end 
aviobj = closeMovie( aviobj, export_to_movie ) ;
 





% ----------------------------------------------------------------------- %
function showCurrentResult(f_ref, data, fignum)
figure(fignum); clf; 
dat_scale = 1 ;

b1 = sqrt(max([f_ref.covariances])) ;
bmin = min([f_ref.mu]) - b1*5 ;
bmax = max([f_ref.mu]) + b1*5 ;
bounds = [bmin,bmax] ;

subplot(1,3,1) ; hold on ;
[hst, hst_x] = hist(data,[-1:0.1:0]) ; hst = hst / sum(hst) ; bar(hst_x,hst) ; axis tight ; ca=axis;
line([0,0],[0,ca(4)],'color','g');
axis([[-1,1]*dat_scale,0,ca(4)]);

subplot(1,3,2) ; 
pdf_pos = constructKDEfromData( data, 'compression', 0 ) ;
for i = 1 : length(pdf_pos.weights)
   showPdf( bounds, 1000, pdf_pos.mu(:,i), pdf_pos.covariances(i,:), pdf_pos.weights(i), 'k', 1) ; 
end
showPdf( bounds, 1000, pdf_pos.mu, pdf_pos.covariances, pdf_pos.weights, 'r',2  ) ;
ca = axis ;
line([0,0],[0,ca(4)],'color','g');   
axis([[-1,1]*dat_scale,0,ca(4)]);

subplot(1,3,3) ; hold on ;
for i = 1 : length(f_ref.weights)
   showPdf( bounds, 1000, f_ref.mu(:,i), f_ref.covariances(i,:), f_ref.weights(i), 'k',1 ) ; 
end 
showPdf( bounds, 1000, f_ref.mu, f_ref.covariances, f_ref.weights, 'r',2  ) ;
ca = axis ;
line([0,0],[0,ca(4)],'color','g');   
axis([[-1,1]*dat_scale,0,ca(4)]);
 
drawnow ;
% ----------------------------------------------------------------------- %
function y_evals = showPdf( bounds, N,centers, covariances, weights, color, lw )
x_evals = [bounds(1):abs(diff(bounds))/N:bounds(2)] ;
y_evals = evaluateDistributionAt( centers, weights, covariances, x_evals ) ;
plot ( x_evals, y_evals, color, 'LineWidth',lw )



% ---------------------------------------------------------- %
% ---------------------------------------------------------- %
function aviobj = initializeMovie( export_to_movie, aviFileName ) 
aviobj = [] ;
if ( export_to_movie == 1 )
    framesPerSec = 10;
    aviobj = avifile(aviFileName,'fps',framesPerSec');
end
% ------------------------ %
function aviobj = recordImage( export_to_movie, aviobj )
if ( export_to_movie == 1 )
    h = gcf; % get current figure handle
    set(h,'Color',[1 1 1]); % set background color to black
    aviobj = addframe(aviobj,h);
end
% ------------------------ %
function aviobj = closeMovie( aviobj, export_to_movie ) 
if ( export_to_movie == 1 )
     aviobj = close(aviobj) ; 
else
    aviobj = [] ;
end
% ------------------------ %
function installDemo1Paths() 
% install Kernel learning
newPath = '..\incrementalPDF\' ; rmpath(newPath) ; addpath(newPath) ;
newPath = '..\incrementalPDF\' ; rmpath(newPath) ; addpath(newPath) ;
installIncrementalKDEUpdatePaths() ;
