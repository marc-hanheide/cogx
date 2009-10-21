%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function eq = testSplitAction( pdf, idx_sel, inPars, otherClasses )
% determines whether component in pdf with index idx_sel should be split or not
 
% generate a sub_pdf from a sublayer 
pdfX.w = pdf.suffStat.subLayer(idx_sel).w*pdf.w(idx_sel) ;
pdfX.Mu = pdf.suffStat.subLayer(idx_sel).Mu ;
pdfX.Cov = pdf.suffStat.subLayer(idx_sel).Cov ;
pdfX.suffStat.A = pdf.suffStat.subLayer(idx_sel).A ;
pdfX.suffStat.B = pdf.suffStat.subLayer(idx_sel).B ;
        
if ~isempty(otherClasses)
     % make unsplit pdf
     selection = 1:length(pdf.w) ;
     selection = find(selection ~= idx_sel) ;
     pdf_sel = extractSubPdf( pdf , selection ) ;  
     % recombine components into augmented pdf
     pdf_augmented = mergeDistributions( pdf_sel, pdfX, [1 1] ) ;
    
     % set priors for negaative to positive examples
%      modelPriors.pPos = 0.5 ;
%      modelPriors.pNeg = 0.5 ;
%      modelPriors.pNeg = otherClasses.priors ;
%      modelPriors.pPos = 1 - modelPriors.pNeg ;
 
     pdfX.w = pdfX.w / sum(pdfX.w) ;
     
     % calculate distance
     d = uCostModel( otherClasses.pdf, pdf, pdf_augmented, otherClasses.priors, inPars.approximateCost ) ;
%      d = 0
     costThreshold = inPars.costThreshold ;
else
    if inPars.useLocalDistanceEvaluation == 1
        pdf_glob.Mu = pdf.Mu(:,idx_sel) ;
        pdf_glob.Cov = pdf.Cov(idx_sel) ;
        pdf_glob.w = pdf.w(idx_sel) ;
        %     pdf_glob.w = pdf_glob.w / sum(pdf_glob.w) ;
        %     pdfX.w = pdfX.w / sum(pdfX.w) ;
        d = uHellingerJointSupport2_ND( pdf_glob, pdfX,...
            'useMarginals', inPars.useMargHellingerCompression,...
            'useWeightedHellinger', inPars.useWeightedHellinger) ;
        N_in = inPars.numberOfSamples *  pdf.w(idx_sel) ;
        %     d = evalMDLDistanceBetweenPdfs( pdf_glob, pdfX , N_in) ;
        costThreshold = inPars.costThreshold ;
    else
        % extract components from pdf without idx_sel component
        selection = 1:length(pdf.w) ;
        selection = find(selection ~= idx_sel) ;
        pdf_sel = extractSubPdf( pdf , selection ) ;
        
        % recombine components into augmented pdf
        pdf_aug = mergeDistributions( pdf_sel, pdfX, [1 1] ) ; % [1 1] since they are already weighted
        
        [d, costThreshold] = calculateDistance( pdf_aug, inPars.costFunction, ...
            inPars.costThreshold, inPars.numberOfSamples,...
            inPars.MDL_params, inPars.useMargHellingerCompression ) ;
    end
end
% if distance "d" is larger than a prespecified threshold, 
% then a split is required.
eq = d > costThreshold ;


