function demo_GaMMoT()

% install paths of toolbox
warning off
install_GaMMoT_DemoPaths() ;
warning on

showIntroduction() ;

% ---------- generate test data ------------ %
% generate data points for "positive distribution"
p = generateGaussianMixture( 4, 1 ) ;
x_pos = sampleMixtureOfGaussians( p.mu, p.weights, p.covariances, 50 ) ;
% generate data points for "nwgative distribution"
p = generateGaussianMixture( 4, 2 ) ;
x_neg = sampleMixtureOfGaussians( p.mu, p.weights, p.covariances, 50 ) ;

% construct array of pdfs for testing
pdf_array = getArrayOfpdfs() ; displayArrayOfpdfs(pdf_array, 2) ;
% ---------- test data generated ------------ %

disp('Data generated. Press a key to start demonstration...'); pause() ;
% calculate KDE of positive data points
pdf_pos = constructKDEfromData( x_pos, 'compression', 3 ) ;
% calculate KDE of negative data points
pdf_neg = constructKDEfromData( x_neg, 'compression', 3 ) ;

disp('Positive and negative distributions estimated.');
disp('Press a key to start unlearning...'); pause() ;
tic
% remove negative KDE from positiove
pdf_unl = unlearnAndCompress( pdf_pos, pdf_neg ) ;
toc

% % % % % % display distributions % % % % % % 
displayUnlearnedDistributions(pdf_pos, pdf_unl, pdf_neg , 1) ; 

disp('Unlearning and compression finished. Press a key to continue...') ; pause() ;
% evaluate Likelihood ratio of a point on the unlearned KDE
dat = x_pos(:,1) ;
bel = getBeliefOfdata( pdf_unl, dat ) ;

% % % % % % display beliefs % % % % % % 
printBelValue(dat, bel) ;

% compare two KDEs
dist_u_p = suHellinger( pdf_unl, pdf_pos ) ;
dist_u_n = suHellinger( pdf_unl, pdf_neg ) ;

% % % % % % display distances % % % % % % 
printDistances(dist_u_p, dist_u_n) ;
 
% get the "spread" function between KDEs
[mu_spread, med_spread] = getSpreadOfpdfs( pdf_array ) ;

% % % % % % display spread % % % % % % 
printSpreads(mu_spread, med_spread) ;

% ----------------------------------------------------------------------- %
function printSpreads(mu_spread, med_spread)
disp('-------- Spread calculation ----------') ;
msg = sprintf('Mean spread of array pdfs: %1.3g ', mu_spread) ; disp(msg) ;
msg = sprintf('Median spread of array pdfs: %1.3g ', med_spread) ; disp(msg) ;
disp(' ') ;

% ----------------------------------------------------------------------- %
function printBelValue(dat, bel)
% plot point and display value
hold on ; plot(dat,0,'*b') ; text(dat,0,num2str(bel),'BackgroundColor',[1 .9 1]) ;
ax = axis ; plot([dat,dat],[0,ax(4)],'b') ;
disp('-------- Belief calculation ----------') ;
msg = sprintf('Belief of point at %1.3g is %1.3g.', dat, bel) ; disp(msg) ;
disp(' ') ;

% ----------------------------------------------------------------------- %
function printDistances(dist_u_p, dist_u_n)
disp('-------- Distance calculation ----------') ;
msg = sprintf('Distance from unlearned pdf to positive pdf: %1.3g', dist_u_p) ;
disp(msg) ;
msg = sprintf('Distance from unlearned pdf to negative pdf: %1.3g', dist_u_n) ;
disp(msg) ;
disp(' ') ;

% ----------------------------------------------------------------------- %
function install_GaMMoT_DemoPaths() 
% install tools
initial_path = pwd ;
cd .. ; 
installGaMMotPaths() ;
cd( initial_path ) ;


% ----------------------------------------------------------------------- %
function pdf_array = getArrayOfpdfs() 

pdf_array = {} ;
num_pdf = 3 ; modes = 4 ;
for i = 1 : num_pdf
    p0 = generateGaussianMixture( modes, 1 ) ;
    pdf_array = horzcat(pdf_array,{p0}) ;
end

num_pdf = 3 ; modes = 5 ;
for i = 1 : num_pdf
    p0 = generateGaussianMixture( modes, 1 ) ;
    pdf_array = horzcat(pdf_array,{p0}) ;
end

% ----------------------------------------------------------------------- %
function showIntroduction()
disp(' ')
disp('--------------------------------------------------------')
disp(' * A demo for GaMMoT: Gaussian Mixture Models Toolbox *')
disp('--------------------------------------------------------')
disp(' ')

