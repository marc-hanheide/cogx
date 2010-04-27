function [ N_comps, S_tp ]= demo2()
%close all
F = [2,7,21,64] ;

export_to_movie = 0 ;
aviobj = initializeMovie( export_to_movie, 'C:\incrementalColors1.avi') ;
dat_scale = 1 ;

%N=500;
[hue_selected, points_xy, ah, raw, mask] = getData( './data/reds', 0 ) ; % apple, tomato, lizard spoon piggy saw object1 apple
X = hue_selected ; 
N = cols(X) ;

bounds = [-1 1 -1 1]*dat_scale ;
int = [bounds(1):(bounds(2)-bounds(1))/100:bounds(2)] ;
prior_adapt = 0.5 ;
nMaxComponents = 20 ;

%install the paths required for demonstration
installDemo2Paths() ;
 
%figure
figure(1);clf

% %init single Gaussian
% mu=X(:,1);
% sg=[0;0];
% n=1;
% 
% kernel_pdf.mu = [] ;
% kernel_pdf.weights = [] ;
% kernel_pdf.covariances = [] ;
% kernel_pdf.components = 1 ;
% kernel_x0 = mu ; % initial point used for initialization if required

C_sensor = 0.01^2 ;

%init single Gaussian
mu = X(1,1); sg = [0]; n = 1;
[mu,sg,n] = updateMV(X(1,2),mu,sg,n);
% initialize kernel
C0 = getSilvermanBWfromGaussian( sg, n )  ;
C0 = C0 + C_sensor ;
kernel_pdf.mu = X(1,1:2) ;
kernel_pdf.weights = [1 1] ; 
kernel_pdf.weights = kernel_pdf.weights / sum(kernel_pdf.weights) ;
kernel_pdf.covariances = [1; 1]*C0 ;
kernel_pdf.components = 2 ;
kernel_x0 = mu ; % initial point used for initialization if required

N_comps = [length(kernel_pdf.weights)] ; 
S_tp = [0 0];
N = 100 ;

s1 = 1 ;
%incremental
for i=3:N
   aviobj = recordImage( export_to_movie, aviobj ) ;
 
   x=X(:,i);
   [mu,sg,n] = updateMV(x',mu,sg,n);

   % update current KDE 
   kernel_pdf = updateKDE( kernel_pdf, kernel_x0, x, sg, n, prior_adapt, 1, C_sensor ) ;
   % compress distribution if the number of components exceeds "nMaxComponents"
   if ( length(kernel_pdf.weights) > nMaxComponents )
       components = kernel_pdf.components ;
       tic
         kernel_pdf = compressDistribution( kernel_pdf, 'showIntermediate', 0) ;
       toc
       kernel_pdf.components = components ;
   end
   kernel_pdf = getMaxOnDistribution( kernel_pdf ) ;

   vals = getNonlinThresholdFun( kernel_pdf, 1000 ) ;
   threshold = getNonLinPdfThreshold( vals, 0.05 ) ;
   kernel_pdf.threshold = threshold ;
   
   [st1,st2] = displayResults( X, points_xy, raw, ah, mask, kernel_pdf, i ) ;
   S_tp = [ S_tp; [st1,st2] ];
   drawnow;
   
   if sum(F==i) > 0
       sw = 0 ;
       if sw == 1
           kernel_pdf = compressDistribution( kernel_pdf, 'showIntermediate', 0) ;
           kernel_pdf = getMaxOnDistribution( kernel_pdf ) ;
           displayResults( X, points_xy, raw, ah, mask, kernel_pdf, i ) ;
       end
       sw = 0 ;
   end
   N_comps = [N_comps, length(kernel_pdf.weights) ] ; 
end 
aviobj = closeMovie( aviobj, export_to_movie ) ;
 
% ------------------------------------------- %
function [tp,fp] = displayResults( X, points_xy, raw, im, mask, kernel_pdf, i )
 
figure(1); clf ;

% plot selected points
subplot(2,3,1);
imagesc(raw) ; hold on ;
scatter(points_xy(1:i,1), points_xy(1:i,2),'b.') ;
scatter(points_xy(i,1), points_xy(i,2),'r.') ;
axis equal ; axis tight ; title(int2str(i))

% plot belief image
subplot(2,3,2); 
% kernel_pdf.max.val = 1 ; ;
%bel = getBeliefImage( im, kernel_pdf ) ;
bel = getBeliefImageVar( im, kernel_pdf ) ;

imagesc(bel) ; colormap gray ; axis equal ; axis tight ;

% plot selection image
subplot(2,3,3);
threshold = kernel_pdf.threshold ;0.5 ;
mm = bel > threshold; 
C = double(raw) ; C(:,:,3) = C(:,:,3).*mm + (1-mm)*255 ;  
C(:,:,2) = C(:,:,2).*mm + (1-mm)*255 ;C(:,:,1) = C(:,:,1).*mm + (1-mm)*255;
% C = hsv2rgb(C) ; 
% C = mm*255 ;
imagesc(uint8(C)) ; colormap gray ; axis equal ; axis tight ;

tp = sum(sum(mm.*mask))/sum(sum(mask)) ;
fp = sum(sum(mm.*(1-mask)))/sum(sum(1-mask)) ;

% plot hue values
subplot(2,3,4);
% pdf_pos = constructKDEfromData( X(1:i), 'compression', 0 ) ;
% pdf_pos = getMaxOnDistribution( pdf_pos ) ;
% bel = getBeliefImage( im, pdf_pos ) ;
% imagesc(bel) ; colormap gray ; axis equal ; axis tight ;

scatter(X(1:i),0*X(1:i),'b*') ; hold on ;
scatter(X(i),0*X(i),'r*') ;
axis([-0.01,1.01,-0.1,0.1]) ;

% draw Kernel-based estimate
subplot(2,3,5);
showDecomposedPdf( kernel_pdf ) ;
msg = sprintf('1-D incremental KDE,\n N_{components}=%d', length(kernel_pdf.weights)) ; 
title(msg);

% ------------------------------------------- %
function bel = getBeliefImage( im, pdf )

w = cols(im) ; h = rows(im) ;
a_r = reshape(im,1,w*h) ;

bel = getBeliefOfdata( pdf, a_r ) ;
bel = reshape( bel, h, w ) ;

% ------------------------------------------- %
function bel = getBeliefImageVar( im, pdf )

w = cols(im) ; h = rows(im) ;
a_r = reshape(im,1,w*h) ;

bel = evaluateDistributionAt( pdf.mu, pdf.weights, pdf.covariances, a_r ) ;
bel = reshape( bel, h, w ) ;



% ------------------------------------------- %
function showDecomposedPdf( pdf )

b1 = sqrt(max([pdf.covariances])) ;
bmin = min([pdf.mu]) - b1*5 ;
bmax = max([pdf.mu]) + b1*5 ;
bounds = [bmin,bmax] ;

bounds = [-0.03,0.13]

hold on ;
for i = 1 : length(pdf.weights)
   showPdf( bounds, 1000, pdf.mu(:,i), pdf.covariances(i,:), pdf.weights(i), 'k', 1) ; 
end
showPdf( bounds, 1000, pdf.mu, pdf.covariances, pdf.weights, 'r',2  ) ;
ca = axis ;
axis([bounds,0,ca(4)]);

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
function installDemo2Paths() 
% install Kernel learning
initial_path = pwd ;
cd ..\.. ; 
installGaMMotPaths() ;
cd( initial_path ) ;


% newPath = '..\..\incrementalPDF\' ; rmpath(newPath) ; addpath(newPath) ;
% newPath = '..\..\incrementalPDF\' ; rmpath(newPath) ; addpath(newPath) ;
% newPath = '..\..\vbwms\' ; rmpath(newPath) ; addpath(newPath) ;
% installIncrementalKDEUpdatePaths() ;
