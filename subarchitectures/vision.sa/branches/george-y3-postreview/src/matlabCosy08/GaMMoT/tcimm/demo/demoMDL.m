function demoMDL()

installDemoPaths() ;

pdf.mu = [1 3 5  7 ] ;
pdf.covariances = ones(1,1,4).^2 ;

pdf.covariances = ones(4,1).^2 ;

weights = [5 0.1 1   6 ] ;ones(1,4) ;
weights = weights/sum(weights) ;
pdf.weights = weights ;
pdf.N = 1000 ;
pdf.E = pdf.weights*pdf.N ;
 
disp(' ') ;
K = 1 ;
for i = 1 : K
     disp( sprintf('---- Iteration no.%d', i)) ;
%     [ pdf_to_reduce, oneToOneCoherence ] = combineComponents( pdf )  ;  
    addMDL = 0 ;
%     [pdf_to_reduce, oneToOneCoherence, delta_mdl] = combineComponentsFullMDL( pdf, pdf, addMDL );   
    [pdf_to_reduce ] = getMDLoptimalDistribution( pdf, pdf ) ;

%     addMDL = delta_mdl ;
%     
    showCurrentResult( pdf, pdf_to_reduce ) ; 
    pdf = pdf_to_reduce ;

    
    
    if K ~= 1
%         pause(1) ;
    end
end
% [pdf_to_reduce, oneToOneCoherence] = combineComponentsFullMDL( pdf_to_reduce, pdf_to_reduce ) ;
% showCurrentResult( pdf, pdf_to_reduce ) ; pause()
% [pdf_to_reduce, oneToOneCoherence] = combineComponentsFullMDL( pdf_to_reduce, pdf_to_reduce ) ;

%-----------------------------------------------%
function showCurrentResult( pdf_c, pdf_h )
 
pdf_c.covariances = reshape(pdf_c.covariances,length(pdf_c.weights),1) ;
 
pdf_h.covariances = reshape(pdf_h.covariances,length(pdf_h.weights),1) ;

figure(1) ; clf ; 
subplot(1,2,1) ; hold on ;
showDecomposedPdf(pdf_h, 'enumComps' , 1) ; title('compressed') ;

subplot(1,2,2) ; hold on ;
showDecomposedPdf(pdf_c,'enumComps' , 1) ; title('Original') ;

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
