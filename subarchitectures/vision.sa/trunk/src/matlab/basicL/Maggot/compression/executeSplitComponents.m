%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function pdf_out = executeSplitComponents( pdf, inPars, otherClasses )
% determines singletons and which components should undergo a split


if nargin < 3
    otherClasses = [] ;
end


TolSing = 1e-20 ;
[singletons , not_singletons ] = findSingletonsBySublayer( pdf.suffStat.subLayer ) ;

% exit if pdf contains only singletons 
if isempty(not_singletons)
    pdf_out = pdf ;
    return ;
end

pdf_out.Mu = [] ;
pdf_out.Cov = {} ;
pdf_out.w = [] ;
pdf_out.suffStat.A = {} ;
pdf_out.suffStat.B = {} ;
pdf_out.suffStat.subLayer = [] ;

for i = 1 : length(pdf.w)
    if any( i==not_singletons ) 
        % evaluate if a split is required         
        eq = testSplitAction( pdf, i, inPars, otherClasses ) ; 
    else
        % by default, do not split
        eq = 0 ;
    end
    
     
    if eq == 1               
        % generate two subcomponents from the subLayer
        pdfX.w = pdf.suffStat.subLayer(i).w*pdf.w(i) ;
        pdfX.Mu = pdf.suffStat.subLayer(i).Mu ;
        pdfX.Cov = pdf.suffStat.subLayer(i).Cov ;
        pdfX.suffStat.A = pdf.suffStat.subLayer(i).A ;
        pdfX.suffStat.B = pdf.suffStat.subLayer(i).B ;
        pdfX.suffStat.subLayer = [] ;
        
        pdf_tmpx = pdfX ;
        [pdf_tmpx, H_prev] = readjustKernels( pdf_tmpx, pdfX.Cov{1}*0 ) ;
        % now generate for each new subcomponent its own sublayer
        for j = 1 : length(pdfX.w) 
           
%             % if resulting points are singletons
%             if ( det(pdfX.Cov{1}) < TolSing )            
%                 pdf_sub.Mu = pdfX.Mu(:,j) ;
%                 pdf_sub.Cov = {pdfX.Cov{1}*0} ;
%                 pdf_sub.w = 1 ;
%                 pdf_sub.A = {pdf_sub.Mu*pdf_sub.Mu'} ;
%                 pdf_sub.B = {pdfX.Cov{1}*0} ;
%             else
%                 pdf_sub_tmp = splitGaussianInTwoWider( pdfX.Mu(:,j), pdfX.Cov{j},...
%                     1, pdfX.suffStat.B(j), pdfX.suffStat.A(j), 2 ) ;
%                 pdf_sub.Mu = pdf_sub_tmp.Mu ;
%                 pdf_sub.Cov = pdf_sub_tmp.Cov ;
%                 pdf_sub.w = pdf_sub_tmp.w ;
%                 pdf_sub.A = pdf_sub_tmp.suffStat.A ;
%                 pdf_sub.B = pdf_sub_tmp.suffStat.B ;                
%             end
 % if resulting points are singletons
            if ( abs(det(pdfX.Cov{1})) < TolSing )            
                pdf_sub1.Mu = pdf_tmpx.Mu(:,j) ;
                pdf_sub1.Cov = {H_prev} ;
                pdf_sub1.w = 1 ;
                pdf_sub1.suffStat.A = {pdf_sub1.Mu*pdf_sub1.Mu'} ;
                pdf_sub1.suffStat.B = {H_prev} ;
            else
                pdf_sub_tmp = splitGaussianInTwoWider( pdf_tmpx.Mu(:,j), pdf_tmpx.Cov{j},...
                    1, pdf_tmpx.suffStat.B(j), pdf_tmpx.suffStat.A(j), 2 ) ;
                pdf_sub1.Mu = pdf_sub_tmp.Mu ;
                pdf_sub1.Cov = pdf_sub_tmp.Cov ;
                pdf_sub1.w = pdf_sub_tmp.w ;
                pdf_sub1.suffStat.A = pdf_sub_tmp.suffStat.A ;
                pdf_sub1.suffStat.B = pdf_sub_tmp.suffStat.B ; 
                pdf_sub1 = readjustKernels( pdf_sub1, H_prev) ;
            end            
            pdf_sub.Mu = pdf_sub1.Mu  ;
                pdf_sub.Cov = pdf_sub1.Cov ;
                pdf_sub.w = pdf_sub1.w ;
                pdf_sub.A = pdf_sub1.suffStat.A ;
                pdf_sub.B = pdf_sub1.suffStat.B ;
            
             pdfX.suffStat.subLayer = horzcat(pdfX.suffStat.subLayer, pdf_sub ) ;            
        end       
    else
        pdfX.w = pdf.w(i) ;
        pdfX.Cov = pdf.Cov(i) ;
        pdfX.Mu = pdf.Mu(:,i) ;
        pdfX.suffStat.A = pdf.suffStat.A(i) ;
        pdfX.suffStat.B = pdf.suffStat.B(i) ;
        pdfX.suffStat.subLayer = pdf.suffStat.subLayer(i) ;
    end
    
    % augment the output kde mixture model
    pdf_out.Mu = [pdf_out.Mu, pdfX.Mu] ;
    pdf_out.Cov = horzcat( pdf_out.Cov, pdfX.Cov ) ;
    pdf_out.w = [pdf_out.w, pdfX.w] ;
    pdf_out.suffStat.A = horzcat(pdf_out.suffStat.A, pdfX.suffStat.A) ;
    pdf_out.suffStat.B = horzcat(pdf_out.suffStat.B, pdfX.suffStat.B) ;
    pdf_out.suffStat.subLayer = horzcat( pdf_out.suffStat.subLayer, pdfX.suffStat.subLayer ) ;
end
if abs(sum(pdf_out.w)-1) > (1e-4)/length(pdf_out.w) 
    error('Weights should sum to one!!') ;
%     pdf_out.w = pdf_out.w / sum(pdf_out.w) ; % just to be safe
end
pdf_out.w = pdf_out.w / sum(pdf_out.w) ;




% --------------------------------------------------------------------- %
function eq = evaluateCovSimsBhatt( C0, C1, alpha )

% BC = exp(-0.5*log( det((C0+C1)/2) / sqrt(det(C0)*det(C1))))  ;
BC = (det((C0+C1)/2) / sqrt(det(C0)*det(C1)) )^(-0.5) ;
H = sqrt( (2-2*BC)/2 ) ;
if H < alpha
    eq = 1 ;
else
    eq = 0 ;
end

% ---------------------------------------------------------------------- %
function [singletons , not_singletons ] = findSingletonsBySublayer( subLayer )

 
singletons = [] ;
not_singletons = [] ;
for i = 1 : length(subLayer)
    if length(subLayer(i).w) == 1
        singletons = [singletons, i] ;
    else
        not_singletons = [ not_singletons, i ] ;
    end    
end
% 
% % ---------------------------------------------------------------------- %
% function [singletons , not_singletons ] = determineSingletonsHere( pdf )
% 
% detTol = 1e-30 ;
% singletons = [] ;
% not_singletons = [] ;
% for i = 1 : length(pdf.w)
%    if det(pdf.Cov{i}) < detTol 
%        singletons = [singletons, i] ;
%    else
%        not_singletons = [ not_singletons, i ] ;
%    end
% end

