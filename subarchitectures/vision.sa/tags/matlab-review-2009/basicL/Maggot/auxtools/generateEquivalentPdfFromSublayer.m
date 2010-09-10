%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function pdf_out = generateEquivalentPdfFromSublayer( pdf )
% generates an equivalent pdf from the pdf's sublayer

pdf_out.Mu = [] ;
pdf_out.Cov = {} ;
pdf_out.w = [] ;
pdf_out.suffStat.A = {} ;
pdf_out.suffStat.B = {} ;
pdf_out.suffStat.subLayer = [] ;
for i = 1 : length(pdf.w)              
        % generate two subcomponents from the subLayer
        pdfX.w = pdf.suffStat.subLayer(i).w*pdf.w(i) ;
        pdfX.Mu = pdf.suffStat.subLayer(i).Mu ;
        pdfX.Cov = pdf.suffStat.subLayer(i).Cov ;
        pdfX.suffStat.A = pdf.suffStat.subLayer(i).A ;
        pdfX.suffStat.B = pdf.suffStat.subLayer(i).B ;
        pdfX.suffStat.subLayer = [] ;
        % now generate for each new subcomponent its own sublayer
        for j = 1 : length(pdfX.w)
            pdf_sub_tmp = splitGaussianInTwoWider( pdfX.Mu(:,j), pdfX.Cov{j},...
                            1, pdfX.suffStat.B(j), pdfX.suffStat.A(j), 2 ) ;
            pdf_sub.Mu = pdf_sub_tmp.Mu ;
            pdf_sub.Cov = pdf_sub_tmp.Cov ;
            pdf_sub.w = pdf_sub_tmp.w ;
            pdf_sub.A = pdf_sub_tmp.suffStat.A ;
            pdf_sub.B = pdf_sub_tmp.suffStat.B ;
            pdfX.suffStat.subLayer = horzcat(pdfX.suffStat.subLayer, pdf_sub ) ;
        end       
   
    
    % augment the output kde mixture model
    pdf_out.Mu = [pdf_out.Mu, pdfX.Mu] ;
    pdf_out.Cov = horzcat( pdf_out.Cov, pdfX.Cov ) ;
    pdf_out.w = [pdf_out.w, pdfX.w] ;
    pdf_out.suffStat.A = horzcat(pdf_out.suffStat.A, pdfX.suffStat.A) ;
    pdf_out.suffStat.B = horzcat(pdf_out.suffStat.B, pdfX.suffStat.B) ;
    pdf_out.suffStat.subLayer = horzcat( pdf_out.suffStat.subLayer, pdfX.suffStat.subLayer ) ;
end

% 
% pdf0.Mu = [] ;
% pdf0.Cov = {} ;
% pdf0.w = [] ;
% pdf0.suffStat.A = [] ;
% pdf0.suffStat.B = [] ;
% for i = 1 : length(pdf.w)
%     sublay = pdf.suffStat.subLayer(i) ;
%     pdf0.Mu = horzcat(pdf0.Mu, sublay.Mu) ;
%     pdf0.Cov = horzcat(pdf0.Cov, sublay.Cov) ;
%     pdf0.w = horzcat(pdf0.w, sublay.w*pdf.w(i)) ;
%     pdf0.suffStat.A = horzcat(pdf0.suffStat.A,pdf.suffStat.A(i)) ;
%     pdf0.suffStat.B = horzcat(pdf0.suffStat.B,pdf.suffStat.B(i)) ;
% end