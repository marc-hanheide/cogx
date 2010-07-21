%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2010
%%
function [ pdf2, idxToref_out, augmented_pdf ] = compressPdf( pdf, varargin )
 
idxToref_out = [] ;
turn_off_splitting = 0 ;
minEigenEnergy = 1e-5 ; 
svdRes = [] ;
maxDistSelect = 2 ;
selectionSeeds = [] ;
use_revitalization = 1 ;
minNumberOfComponentsThreshold = 0 ;
debugForceCompress = [] ;
approximateCost = 0 ;
otherClasses = [] ;
memorylimitUseNumComps = 0 ;
useWeightedHellinger = 1 ;
memorylimit = [] ;
compressionDirection = 'topDown' ; %'bottomUp' ;
applyProjectionToSubspace = 0 ;
useLocalDistanceEvaluation = 0 ; 
useMargHellingerCompression = 1 ;
useSMOprunning = 0 ;
threshOnSplitMethods = inf ;       
granularity_cell_num = 50 ; 
typeNoiseDetermination = 'inflation' ; % 'granularity'      
typeCompression = 'hierarchical' ; % 'affinity, hierarchical'
costFunction = 'hellinger' ;
costThreshold = 0.01 ;
numberOfSamples = [] ;

% process arguments
args = varargin;
nargs = length(args);
for i = 1:2:nargs
    switch args{i}        
        case 'typeCompression', typeCompression = args{i+1} ; 
        case 'costFunction', costFunction = args{i+1} ;
        case 'costThreshold', costThreshold = args{i+1} ;   
        case 'numberOfSamples', numberOfSamples = args{i+1} ;  
        case 'granularity_cell_num', granularity_cell_num = args{i+1} ;  
        case 'typeNoiseDetermination', typeNoiseDetermination = args{i+1} ; 
        case 'threshOnSplitMethods', threshOnSplitMethods = args{i+1} ;
        case 'useSMOprunning', useSMOprunning = args{i+1} ;
        case 'useMargHellingerCompression', useMargHellingerCompression = args{i+1} ;
        case 'useLocalDistanceEvaluation', useLocalDistanceEvaluation = args{i+1} ; 
        case 'applyProjectionToSubspace', applyProjectionToSubspace = args{i+1} ; 
        case 'compressionDirection', compressionDirection = args{i+1} ; 
        case 'memorylimit', memorylimit = args{i+1} ; 
        case 'useWeightedHellinger', useWeightedHellinger = args{i+1} ; 
        case 'memorylimitUseNumComps', memorylimitUseNumComps = args{i+1} ; 
        case 'otherClasses', otherClasses = args{i+1} ;
        case 'approximateCost', approximateCost = args{i+1} ;
        case 'debugForceCompress', debugForceCompress = args{i+1} ;    
        case 'minNumberOfComponentsThreshold', minNumberOfComponentsThreshold = args{i+1} ;    
        case 'use_revitalization', use_revitalization = args{i+1} ; 
        case 'selectionSeeds', selectionSeeds = args{i+1} ; 
        case 'maxDistSelect', maxDistSelect = args{i+1} ; 
        case 'svdRes', svdRes = args{i+1} ; 
        case 'turn_off_splitting', turn_off_splitting = args{i+1} ; 
    end
end
 
% otherClasses = [] 

if ~isempty(otherClasses)
    costFunction = 'hellinger' ;
    compressionDirection = 'bottomUp' ;
end
 
if applyProjectionToSubspace == 1  
   output = subspacePrewhitenTransform( 'pdf', pdf, 'minEigenEnergy',minEigenEnergy, ...
                                        'transDirection', 'forward',...
                                        'allLayers', 1 ) ;                                
   pdf = output.pdf ;
   svdRes = output.svdRes ;
end

 
% allways project all other classes into the subspace of the target
% distribution
if ~isempty(otherClasses)
    minNumberOfComponentsThreshold = 1 ;
    if ~isempty(svdRes)         
        % regularize input pdf within the selected subspace
        for i_other = 1 : length(otherClasses.pdfs)
            t_ret = subspacePrewhitenTransform( 'pdf', otherClasses.pdfs{i_other}, 'minEigenEnergy', minEigenEnergy, ...
                'transDirection', 'forward', ...
                'allLayers', 0,...
                'regularize', 'subRegularize',...
                'svdRes', svdRes ) ;
            otherClasses.pdfs{i_other} = t_ret.pdf ;            
        end
%         minNumberOfComponentsThreshold = 1 ;
%     else
%         turn_off_splitting = 1 ;
    end
    % merge other classes if accessible
%     otherClass_pdf.pdf = mergeOtherClasses( otherClasses ) ;
    otherClass_pdf.pdf.pdfs = otherClasses.pdfs ; % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11
    otherClass_pdf.pdf.inner_priors = otherClasses.inner_priors ;
    otherClass_pdf.pdf.priors = otherClasses.priors ;
%     t_ret = subspacePrewhitenTransform( 'pdf', otherClass_pdf.pdf, 'minEigenEnergy', minEigenEnergy, ...
%             'transDirection', 'forward', ...
%             'allLayers', 0,...
%             'regularize', 'subRegularize',...
%             'svdRes', svdRes ) ;
%     otherClass_pdf.pdf = t_ret.pdf ;
    otherClass_pdf.priors = otherClasses.priors ;
    clear otherClasses ;
    %      MaxV = 3 ;
    %      otherClass_pdf.pdf.sigmaPoints = precalculateSigmaPoints(
    %      otherClass_pdf.pdf, MaxV ) ;
else
    otherClass_pdf = [] ;
end

 
if ~isempty(selectionSeeds) 
   % determine and extract the subkde -> pdf, pdf_oth   
   [pdf, pdf1] = extractRelevantSubkde( pdf, selectionSeeds, maxDistSelect ) ;  
   
   if ~isempty(otherClass_pdf)
       ignoreSublayer = 1 ;
       pdf_tmp = extractRelevantSubkde( otherClass_pdf.pdf, selectionSeeds, maxDistSelect*2, [], ignoreSublayer ) ;
       if isempty(pdf_tmp)
           otherClass_pdf.pdf = [] ; 
       else
           otherClass_pdf.pdf = pdf_tmp ;
       end
   end
   
end

pdf2 = [] ;
if ~isempty(pdf)
    switch(typeCompression)
        case 'meanshift'
            [ pdf2 ] = meanshiftCompression( pdf, 'costFunction', costFunction,...
                'costThreshold', costThreshold,...
                'numberOfSamples', numberOfSamples,...
                'threshOnSplitMethods', threshOnSplitMethods,...
                'useSMOprunning', useSMOprunning,...
                'useMargHellingerCompression', useMargHellingerCompression,...
                'useLocalDistanceEvaluation', useLocalDistanceEvaluation,...
                'memoryLimit',  memorylimit,...
                'useWeightedHellinger', useWeightedHellinger, ...
                'use_revitalization', use_revitalization) ;
            stateComponents0 = [] ;
        case 'hierarchical'
            if isequal(costFunction,'MDL')
                
                stateComponents0 = [] ;
                [ pdf2, idxToref_out, augmented_pdf ] = hierarchicalCompression_BestFirst( pdf, ...
                    'costFunction', costFunction,...
                    'costThreshold', costThreshold,...
                    'numberOfSamples', numberOfSamples,...
                    'granularity_cell_num', granularity_cell_num,...
                    'typeNoiseDetermination', typeNoiseDetermination,...
                    'threshOnSplitMethods', threshOnSplitMethods,...
                    'useSMOprunning', useSMOprunning,...
                    'useMargHellingerCompression', useMargHellingerCompression,...
                    'useLocalDistanceEvaluation', useLocalDistanceEvaluation,...
                    'memoryLimit',  memorylimit,...
                    'useWeightedHellinger', useWeightedHellinger,...
                    'memorylimitUseNumComps', memorylimitUseNumComps,...
                    'debugForceCompress', debugForceCompress, ...
                    'minNumberOfComponentsThreshold', minNumberOfComponentsThreshold, ...
                    'use_revitalization', use_revitalization) ;
                
% % % % % % % % % % % % % % % % % % % %                 %             [ pdf2, stateComponents0 ] = hierarchicalCompression_MDL( pdf, ...
% % % % % % % % % % % % % % % % % % % %                 %                                         'costFunction', costFunction,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'costThreshold', costThreshold,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'numberOfSamples', numberOfSamples,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'granularity_cell_num', granularity_cell_num,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'typeNoiseDetermination', typeNoiseDetermination,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'threshOnSplitMethods', threshOnSplitMethods,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'useSMOprunning', useSMOprunning,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'useMargHellingerCompression', useMargHellingerCompression,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'useLocalDistanceEvaluation', useLocalDistanceEvaluation,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'memoryLimit',  memorylimit,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'useWeightedHellinger', useWeightedHellinger,...
% % % % % % % % % % % % % % % % % % % %                 %                                         'memorylimitUseNumComps', memorylimitUseNumComps ) ;
                
            elseif isequal(costFunction,'hellinger')
                 
                switch compressionDirection
                    case 'bottomUp'
%                         error('Did not check this!') ;
                                           [ pdf2, idxToref_out, augmented_pdf ] = hierarchicalCompression_BestFirstCombined( pdf, ...
                                                                'costFunction', costFunction,...
                                                                'costThreshold', costThreshold,...
                                                                'numberOfSamples', numberOfSamples,...
                                                                'granularity_cell_num', granularity_cell_num,...
                                                                'typeNoiseDetermination', typeNoiseDetermination,...
                                                                'threshOnSplitMethods', threshOnSplitMethods,...
                                                                'useSMOprunning', useSMOprunning,...
                                                                'useMargHellingerCompression', useMargHellingerCompression,...
                                                                'useLocalDistanceEvaluation', useLocalDistanceEvaluation,...
                                                                'memoryLimit',  memorylimit,...
                                                                'useWeightedHellinger', useWeightedHellinger,...
                                                                'memorylimitUseNumComps', memorylimitUseNumComps,...
                                                                'debugForceCompress', debugForceCompress, ...
                                                                'minNumberOfComponentsThreshold', minNumberOfComponentsThreshold,...
                                                                 'otherClasses', otherClass_pdf,...
                                                                 'approximateCost', approximateCost,...
                                                                 'turn_off_splitting', turn_off_splitting ) ;
                        %                   This is originally used in ICPR and ERK papers
%                         [ pdf2, stateComponents0 ] = hierarchicalCompression( pdf, ...
%                             'costFunction', costFunction,...
%                             'costThreshold', costThreshold,...
%                             'numberOfSamples', numberOfSamples,...
%                             'granularity_cell_num', granularity_cell_num,...
%                             'typeNoiseDetermination', typeNoiseDetermination,...
%                             'threshOnSplitMethods', threshOnSplitMethods,...
%                             'useSMOprunning', useSMOprunning,...
%                             'useMargHellingerCompression', useMargHellingerCompression,...
%                             'useLocalDistanceEvaluation', useLocalDistanceEvaluation,...
%                             'otherClasses', otherClass_pdf, ...
%                             'approximateCost', approximateCost) ;
                        
                        % % %                      [ pdf2, stateComponents0 ] = hierarchicalCompression_BestFirstDiscriminative( pdf, ...
                        % % %                                         'costFunction', costFunction,...
                        % % %                                         'costThreshold', costThreshold,...
                        % % %                                         'numberOfSamples', numberOfSamples,...
                        % % %                                         'granularity_cell_num', granularity_cell_num,...
                        % % %                                         'typeNoiseDetermination', typeNoiseDetermination,...
                        % % %                                         'threshOnSplitMethods', threshOnSplitMethods,...
                        % % %                                         'useSMOprunning', useSMOprunning,...
                        % % %                                         'useMargHellingerCompression', useMargHellingerCompression,...
                        % % %                                         'useLocalDistanceEvaluation', useLocalDistanceEvaluation,...
                        % % %                                         'otherClasses', otherClass_pdf, ...
                        % % %                                         'approximateCost', approximateCost) ;
                        
                        
                    case 'topDown'
                        error('Did not check this!') ;
                        [ pdf2, stateComponents0 ] = hierarchicalCompression_New( pdf, ...
                            'costFunction', costFunction,...
                            'costThreshold', costThreshold,...
                            'numberOfSamples', numberOfSamples,...
                            'granularity_cell_num', granularity_cell_num,...
                            'typeNoiseDetermination', typeNoiseDetermination,...
                            'threshOnSplitMethods', threshOnSplitMethods,...
                            'useSMOprunning', useSMOprunning,...
                            'useMargHellingerCompression', useMargHellingerCompression,...
                            'useLocalDistanceEvaluation', useLocalDistanceEvaluation,...
                            'useWeightedHellinger', useWeightedHellinger) ;
                    otherwise
                        msg = sprintf('Unknown compression direction: %s', compressionDirection) ;
                        error(msg) ;
                end
            end
        otherwise
            error('Affinity not implemented here!') ;
    end
end
 
if ~isempty(selectionSeeds)
   % merge pdf2 and pdf_oth into pdf2  
   if isempty(pdf1)
        % all components have been modified, i.e., pdf2 = pdf2 ;
   elseif isempty(pdf2)
       pdf2 = pdf1 ;
   else
        pdf2 = mergeKDEs( pdf1 , pdf2,1 ) ;
   end
end


if applyProjectionToSubspace == 1    
   output = subspacePrewhitenTransform( 'pdf', pdf2, 'minEigenEnergy', minEigenEnergy, ...
                                        'transDirection', 'backward',...
                                        'allLayers', output.allLayers,...
                                        'svdRes', output.svdRes ) ;    
    pdf2 = output.pdf ;     
end
 
pdf2.w = pdf2.w / sum(pdf2.w) ;
 

% if nargout == 2
%     stateComponents = stateComponents0 ;
% else
%     stateComponents = [] ;
% end

% ---------------------------------------------------------------------- %
function otherClass_pdf = mergeOtherClasses( otherClasses )

if isempty(otherClasses)
    otherClass_pdf = [] ;
    return ;
end
otherClass_pdf = otherClasses.pdfs{1} ;  

for i = 2 : length(otherClasses.pdfs)
    prrs = [sum(otherClasses.inner_priors(1:i-1)) , otherClasses.inner_priors(i) ] ;
    prrs = prrs / sum(prrs) ;
    otherClass_pdf =  mergeDistributions( otherClass_pdf, otherClasses.pdfs{i}, prrs ) ;
end

% ---------------------------------------------------------------------- %
function sigmaPoints = precalculateSigmaPoints( pdf, MaxV ) 

    % calculate sigma points for posModel only
    [X, sigPointsPerComponent, w, k ] = getAllSigmaPointsOnMixture( pdf, MaxV ) ;
    W = repmat(pdf.w,sigPointsPerComponent,1) ;
    W = reshape(W,1,length(pdf.w)*sigPointsPerComponent) ;
    w2 = repmat(w,1,length(pdf.w)) ;
    W = W.*w2 ;
    sigmaPoints.W_final = W ;
    sigmaPoints.X_final = X ;
 
