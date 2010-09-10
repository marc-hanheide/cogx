%%
% Originally a part of: Maggot (developed within EU project CogX)
% Author: Matej Kristan, 2009 (matej.kristan@fri.uni-lj.si; http://vicos.fri.uni-lj.si/matejk/)
% Last revised: 2009
%%
function out_kde = executeOperatorIKDE( input_kde, varargin )
% input_data ... may be input points or even a pdf for unlearning
% input_kde  ... kernel density estimate
% Structure:
%   input_kde.ikdeParams
%   input_kde.otherParams
%   input_kde.pdf
%           Structure of the pdf :
%                          pdf.Mu(:,N)
%                          pdf.Cov{1:N} ; {1} = [dxd]
%                          pdf.w = [1:N] ;
%                          pdf.suffStat[1].B{1:N}; {1} = [dxd]
%                          pdf.suffStat[1].A{1:N}; {1} = [dxd]   
%                          pdf.sublayer[1:N].Mu(:,1 or 2)
%                          pdf.sublayer[1:N].Cov{1 or 2}; {1} = [dxd] ;
%                          pdf.sublayer[1:N].w[1 or 2];  
%                          pdf.sublayer[1:N].A{1:N}; {1} = [dxd] ;
%                          pdf.sublayer[1:N].B{1:N}; {1} = [dxd] ;
% -------------- Potrebno bo verjetno še narediti:
% evalUnderKde(kde, x)
% da preveri, èe potrebuje subspace, nato pa toèko(e) projecira
% v subspace in tam izraèuna verjetnosti.
% -------------- ZAdnje spremen+mbe:
% spremenil sem updatanje uteži
%
%
% Operators: 'evalPdfOnData', 'add_input', 'unlearn_with_input',
% 'compress_pdf', 'evalHellingerBetween', 'getSubDimKDE', 'evalTypOnData'

showTabulated = 0 ;
showkdecolor = 'k' ;
additional_kde = [] ; % for calculating distances between KDEs
selectSubDimensions = [] ; % if we would like to perform an operation in subdimensional space
minNumberOfComponentsThreshold = [] ;
useSomeOtherKindOfEstimator = [] ; % other types of estimators: 'figJainEM', 'adaptiveMixtures'
debugForceCompress = [] ; % for debugging to force compression of only certain components
typeCompression = [] ; % used to identify compression algorithm
approximateCost = [] ; % used for classifier optimization
otherClasses = [] ; % used for classifier training
usehalfHellingerInCompression = [] ; % 0 , 1
forceBandwidth = [] ; % to pass in a forced bandwidth
unlearnPointsType = [] ; % 'jointBandwidth' 'separateBandwidth' ;
MDL_memorylimitUseComps = [] ;
typeOfKDEInit = [] ; % 'directPlugin' ; 'stePlugin' ; 'lscv' ; 'bivDiffusion'
typePluginCalculation = [] ; % 'one_Stage' ; %'n_Stage'
priorBandwidth = [] ;
MDL_memorylimit = -1 ;
compressionDirection = [] ;%'topDown' ; 'bottomUp'
applyAPPreprocessingStep = [] ;
useLocalDistanceEvaluation = [] ;
useSublayerForDerivativeModel = [] ;
useMargHellingerCompression = [] ;
useSMOprunning = [] ;
threshOnSplitMethods = [] ;
% MDL_guides_typeNoiseDetermination = [] ;
% MDL_guides_granularity_cell_num = [] ;
maxNumCompsBeforeCompression = [] ;
MDL_guided_BW_Cprss = [] ;
granularity_cell_num = [] ; 
typeNoiseDetermination = [] ; %'inflation' ; % 'granularity' 
costFunctionCompression = [] ; %'hellinger, numberOfComponents, alpha_MDL'
max_ibw_iterations = [] ;
kde_w_attenuation = [] ;
precompressUnlearning = 0 ;
unlearning_MakeProperPdf = 1 ;
compressionClusterThresh = [] ;
% unlearning_AllowCompleteUnlearning = [] ;
compressionMaxComponents = [] ;
out_kde = [] ;
input_data = [] ;
operator_data = '' ; % unlearn_with_input, add_input 
obs_relative_weights = [] ;

args = varargin;
nargs = length(args);
i = 1 ;
while i <= nargs   
    switch args{i}        
        case 'input_data', input_data = args{i+1} ; i = i + 2 ; 
        case 'add_input', operator_data = args{i} ; i = i + 1 ; 
        case 'evalPdfOnData', operator_data = args{i} ; i = i + 1 ; 
        case 'unlearn_with_input', operator_data = args{i} ; i = i + 1 ;             
        case 'compress_pdf', operator_data = args{i} ; i = i + 1 ; 
        case 'evalHellingerBetween', operator_data = args{i} ; i = i + 1 ;  
        case 'getSubDimKDE', operator_data = args{i} ; i = i + 1 ;
        case 'evalTypOnData', operator_data = args{i} ; i = i + 1 ;   
        case 'showKDE', operator_data = args{i} ; i = i + 1 ;      
        case 'tabulated', showTabulated = args{i+1} ; i = i + 2 ;
        case 'showkdecolor', showkdecolor = args{i+1} ; i = i + 2 ;        
        case 'initialize', operator_data = args{i} ; input_kde = [] ; i = i + 1 ;             
        case 'obs_relative_weights', obs_relative_weights = args{i+1} ; i = i + 2 ;
        case 'compressionClusterThresh', compressionClusterThresh = args{i+1} ; i = i + 2 ;
        case 'compressionMaxComponents', compressionMaxComponents = args{i+1} ; i = i + 2 ;    
        case 'unlearning_MakeProperPdf', unlearning_MakeProperPdf = args{i+1} ; i = i + 2 ;
        case 'usehalfHellingerInCompression', usehalfHellingerInCompression = args{i+1} ; i = i + 2 ;
        case 'precompressUnlearning', precompressUnlearning = 1 ;  i = i + 1 ;   
        case 'kde_w_attenuation', kde_w_attenuation = args{i+1} ; i = i + 2 ;
        case 'max_ibw_iterations', max_ibw_iterations = args{i+1} ; i = i + 2 ;
        case 'costFunctionCompression', costFunctionCompression = args{i+1} ; i = i + 2 ;
        case 'granularity_cell_num', granularity_cell_num = args{i+1} ; i = i + 2 ;
        case 'typeNoiseDetermination', typeNoiseDetermination = args{i+1} ; i = i + 2 ; 
        case 'MDL_guided_BW_Cprss', MDL_guided_BW_Cprss = args{i+1} ; i = i + 2 ; 
        case 'maxNumCompsBeforeCompression', maxNumCompsBeforeCompression = args{i+1} ; i = i + 2 ; 
%         case 'MDL_guides_typeNoiseDetermination', MDL_guides_typeNoiseDetermination = args{i+1} ; i = i + 2 ; 
%         case 'MDL_guides_granularity_cell_num', MDL_guides_granularity_cell_num = args{i+1} ; i = i + 2 ;     
        case 'threshOnSplitMethods', threshOnSplitMethods = args{i+1} ; i = i + 2 ; 
        case 'useSMOprunning', useSMOprunning = args{i+1} ; i = i + 2 ; 
        case 'useMargHellingerCompression', useMargHellingerCompression = args{i+1} ; i = i + 2 ;
        case 'useSublayerForDerivativeModel', useSublayerForDerivativeModel = args{i+1} ; i = i + 2 ;
        case 'useLocalDistanceEvaluation', useLocalDistanceEvaluation = args{i+1} ; i = i + 2 ; 
        case 'applyAPPreprocessingStep', applyAPPreprocessingStep = args{i+1} ; i = i + 2 ;
        case 'compressionDirection', compressionDirection = args{i+1} ; i = i + 2 ;
        case 'MDL_memorylimit', MDL_memorylimit = args{i+1} ; i = i + 2 ;
        case 'priorBandwidth', priorBandwidth = args{i+1} ; i = i + 2 ;
        case 'typePluginCalculation', typePluginCalculation = args{i+1} ; i = i + 2 ;
        case 'typeOfKDEInit', typeOfKDEInit = args{i+1} ; i = i + 2 ;
        case 'MDL_memorylimitUseComps', MDL_memorylimitUseComps = args{i+1} ; i = i + 2 ;
        case 'unlearnPointsType', unlearnPointsType = args{i+1} ; i = i + 2 ;
        case 'forceBandwidth', forceBandwidth = args{i+1} ; i = i + 2 ;
        case 'otherClasses', otherClasses = args{i+1} ; i = i + 2 ;
        case 'approximateCost', approximateCost = args{i+1} ; i = i + 2 ;
        case 'typeCompression', typeCompression = args{i+1} ; i = i + 2 ;
        case 'debugForceCompress', debugForceCompress = args{i+1} ; i = i + 2 ;
        case 'useSomeOtherKindOfEstimator', useSomeOtherKindOfEstimator = args{i+1} ; i = i + 2 ;
        case 'minNumberOfComponentsThreshold', minNumberOfComponentsThreshold = args{i+1} ; i = i + 2 ;
        case 'additional_kde', additional_kde = args{i+1} ; i = i + 2 ;
        case 'selectSubDimensions', selectSubDimensions = args{i+1} ; i = i + 2 ;
        otherwise
            msg = sprintf('Unknown switch "%s"!',args{i});
            error(msg) ;
    end
end

% preprocessing switches
% determine type of input data
if isempty(input_data)
    type_init = 'null' ;
elseif isfield(input_data,'w')
    type_init = 'pdf' ;
elseif isfield(input_data,'pdf')
    type_init = 'kde' ;    
else
    type_init = 'points' ;
    if isempty(obs_relative_weights)
        obs_relative_weights = ones(1,size(input_data,2)) ;
%         obs_relative_weights = obs_relative_weights / sum(obs_relative_weights) ;
    end        
end

% if initialization reqested
if isequal(operator_data,'initialize')
    input_kde = [] ;
    operator_data = 'add_input' ;
end
% end of preprocessing switches

% generate prototype input_kde if neccesary
if ~isfield(input_kde,'pdf')
    input_kde.pdf.Mu = [] ;
    input_kde.pdf.Cov = {} ;
    input_kde.pdf.w = [] ;
    input_kde.pdf.suffStat.B = {} ;
    input_kde.pdf.suffStat.A = {} ;
    input_kde.pdf.suffStat.subLayer = [] ;
end
if ~isfield(input_kde,'ikdeParams')
    [input_kde.ikdeParams, input_kde.otherParams]= initializeParamsKDE( [] ) ;    
end

% modify parameters if required
if ~isempty(useSomeOtherKindOfEstimator)
    input_kde.otherParams.useSomeOtherKindOfEstimator = useSomeOtherKindOfEstimator ; 
    out_kde = input_kde ;    
end
 

if ~isempty(minNumberOfComponentsThreshold)
   input_kde.otherParams.minNumberOfComponentsThreshold = minNumberOfComponentsThreshold ; 
    out_kde = input_kde ;     
end

if ~isempty(typeCompression)
    input_kde.otherParams.typeCompression = typeCompression ; 
    out_kde = input_kde ;    
end

if ~isempty(approximateCost)
    input_kde.otherParams.approximateCost = approximateCost ; 
    out_kde = input_kde ;    
end

if ~isempty(unlearnPointsType)
    input_kde.otherParams.unlearnPointsType = unlearnPointsType ; 
    out_kde = input_kde ;    
end

if ~isempty(MDL_memorylimitUseComps)
    input_kde.otherParams.MDL_memorylimitUseComps = MDL_memorylimitUseComps ; 
    out_kde = input_kde ;    
end


if ~isempty(typeOfKDEInit)
    input_kde.otherParams.typeOfKDEInit = typeOfKDEInit ; 
    out_kde = input_kde ;    
end

if ~isempty(typePluginCalculation)
    input_kde.otherParams.typePluginCalculation = typePluginCalculation ; 
    out_kde = input_kde ;    
end

if ~isempty(kde_w_attenuation)
    input_kde.ikdeParams.suffSt.w_att = kde_w_attenuation ;
    out_kde = input_kde ;    
end
 
if ~isempty(MDL_guided_BW_Cprss)
    input_kde.otherParams.MDL_guides.MDL_guided_BW_Cprss = MDL_guided_BW_Cprss ; 
    out_kde = input_kde ;    
end
 
% if ~isempty(MDL_guides_typeNoiseDetermination)
%     input_kde.otherParams.MDL_guides.typeNoiseDetermination = MDL_guides_typeNoiseDetermination ; 
%     out_kde = input_kde ;    
% end

% if ~isempty(MDL_guides_granularity_cell_num)
%     input_kde.otherParams.MDL_guides.granularity_cell_num = MDL_guides_granularity_cell_num ; 
%     out_kde = input_kde ;    
% end

if ~isempty(maxNumCompsBeforeCompression)
    input_kde.ikdeParams.maxNumCompsBeforeCompression = maxNumCompsBeforeCompression ;
    out_kde = input_kde ;    
end
 
if ~isempty(costFunctionCompression)
    input_kde.otherParams.costFunctionCompression = costFunctionCompression ;
    out_kde = input_kde ;    
end

if ~isempty(max_ibw_iterations)
    input_kde.otherParams.max_ibw_iterations = max_ibw_iterations ;
    out_kde = input_kde ;    
end
 
if ~isempty(compressionClusterThresh)
    input_kde.otherParams.compressionClusterThresh = compressionClusterThresh ;
    out_kde = input_kde ;    
end

if ~isempty(usehalfHellingerInCompression)
    input_kde.otherParams.usehalfHellingerInCompression = usehalfHellingerInCompression ;
    out_kde = input_kde ;    
end 

if ~isempty(compressionMaxComponents)
    input_kde.otherParams.compressionMaxComponents = compressionMaxComponents ;
    out_kde = input_kde ;  
end

if ~isempty(granularity_cell_num)
    input_kde.otherParams.granularity_cell_num = granularity_cell_num ;
    out_kde = input_kde ;  
end

if ~isempty(typeNoiseDetermination)
    input_kde.otherParams.typeNoiseDetermination = typeNoiseDetermination ;
    out_kde = input_kde ;  
end

if ~isempty(threshOnSplitMethods)
    input_kde.otherParams.threshOnSplitMethods = threshOnSplitMethods ;
    out_kde = input_kde ;  
end

if ~isempty(useSMOprunning)
    input_kde.otherParams.useSMOprunning = useSMOprunning ;
    out_kde = input_kde ;  
end

if ~isempty(useMargHellingerCompression)
    input_kde.otherParams.useMargHellingerCompression = useMargHellingerCompression ;
    out_kde = input_kde ;  
end

if ~isempty(useSublayerForDerivativeModel)
    input_kde.otherParams.useSublayerForDerivativeModel = useSublayerForDerivativeModel ;
    out_kde = input_kde ;  
end 

if ~isempty(useLocalDistanceEvaluation)
    input_kde.otherParams.useLocalDistanceEvaluation = useLocalDistanceEvaluation ;
    out_kde = input_kde ;  
end 

if ~isempty(applyAPPreprocessingStep)
    input_kde.otherParams.applyAPPreprocessingStep = applyAPPreprocessingStep ;
    out_kde = input_kde ;  
    warning('Affinity Propagation preprocessing step not implemented yet!') ;
end 
 
if ~isempty(compressionDirection)
    input_kde.otherParams.compressionDirection = compressionDirection ;
    out_kde = input_kde ;  
end 

if ~isequal(MDL_memorylimit,-1)
    input_kde.otherParams.MDL_memorylimit = MDL_memorylimit ;
    out_kde = input_kde ;  
end 

if ~isempty(priorBandwidth)
    input_kde.otherParams.priorBandwidth = priorBandwidth ;
    out_kde = input_kde ;  
end 
 
% ------ process KDE
switch operator_data
    case 'add_input'
        if isequal(type_init,'null') 
            return ;
        end
        input_kde.otherParams.maximumsOnPdf = [] ;
        % use other method?
        if isequal(input_kde.otherParams.useSomeOtherKindOfEstimator, 'adaptiveMixtures')  
            [ pdf_out, ikdeParams ] = updateAdaptiveMixtures( input_kde.pdf, input_data,... 
                                                              input_kde.ikdeParams,...
                                                              input_kde.otherParams ) ;
            out_kde = input_kde ;
            out_kde.pdf= pdf_out ; 
            out_kde.ikdeParams = ikdeParams ; 
            return ;
        elseif isequal(input_kde.otherParams.useSomeOtherKindOfEstimator, 'figJainEM')   
            figJainEM = input_kde.otherParams.figJainEM ;
            pdf_out = figjainEM( input_data , figJainEM.mink , figJainEM.maxk, figJainEM.regularize, figJainEM.minth, figJainEM.convoption ) ;
            out_kde = input_kde ;
            out_kde.pdf= pdf_out ;  
            return ;
        end
        
        
        obs = [] ;
        if isequal(type_init,'points') % if adding points
            obs = input_data ;
        elseif isequal(type_init,'pdf') % if adding pdf (initializing)
           [ ikdeParams, otherParams ]= initializeParamsKDE( input_data ) ;
            pdf = input_data ;
            out_kde.ikdeParams = ikdeParams ;
            out_kde.otherParams = otherParams ;
            out_kde.pdf = pdf ;
            B = zeros(size(pdf.Mu,1)) ;
            out_kde.pdf.suffStat.B = {} ;
            out_kde.pdf.suffStat.A = {} ;
            out_kde.pdf.subLayer = [] ;
            
            for j = 1 : length(pdf.w)
                out_kde.pdf.suffStat.B = horzcat(out_kde.pdf.suffStat.B, {B}) ; 
                out_kde.pdf.suffStat.A = horzcat(out_kde.pdf.suffStat.A, {pdf.Mu(:,j)*pdf.Mu(:,j)'}) ;  
                
                sub_pdf.Mu = pdf.Mu(:,j) ;
                sub_pdf.Cov = pdf.Cov{j} ;
                sub_pdf.w = 1 ;
                sub_pdf.A = {pdf.Mu(:,j)*pdf.Mu(:,j)'} ;
                sub_pdf.B = {B} ;
                out_kde.pdf.suffStat.subLayer = horzcat( out_kde.pdf.subLayer, sub_pdf ) ;
            end                     
            return ;
        end                
                   
        % should MDL completely guide bw_selection and component threshold?
        if input_kde.otherParams.MDL_guides.MDL_guided_BW_Cprss == 1 ;
            MDL_guides = input_kde.otherParams.MDL_guides ;
        else
            MDL_guides = [] ;
        end

        [model_new, ikdeParams] = estimateAmiseOptimalKernel( 'model', input_kde.pdf, 'obs', obs,...
                                'obs_relative_weights', obs_relative_weights,...
                                'ikdeParams', input_kde.ikdeParams,...
                                'useMarginalBasedBWs',0,...
                                'accountForVirginComponents',1,...
                                'max_ibw_iterations', input_kde.otherParams.max_ibw_iterations,...
                                'MDL_guides', MDL_guides,...
                                'useSublayerForDerivativeModel', input_kde.otherParams.useSublayerForDerivativeModel,...
                                'priorBandwidth', input_kde.otherParams.priorBandwidth,...
                                'typePluginCalculation', input_kde.otherParams.typePluginCalculation,...
                                'typeOfKDEInit', input_kde.otherParams.typeOfKDEInit,...
                                'forceBandwidth', forceBandwidth ) ; 
          
        % for debugging, track the last estimated amise
%         input_kde.otherParams.debug.lastCov = model_new.Cov( length(input_kde.pdf.w) + 1: length(model_new.w) ) ;
        
        
        out_kde.pdf = model_new ;
        out_kde.ikdeParams = ikdeParams ; 
        out_kde.otherParams = input_kde.otherParams ;
%         out_kde.ikdeParams.virginComps = out_kde.ikdeParams.virginComps*0 
        
        if ikdeParams.maxNumCompsBeforeCompression == -1
            ikdeParams.maxNumCompsBeforeCompression = ikdeParams.dim_subspace*5 ;
        end

        % engage automatic compression   
        if ~isempty( input_kde.pdf.w )            
            [model_new, ikdeParams] = manageAutomaticCompression( out_kde, otherClasses ) ;
            out_kde.pdf = model_new ;
            out_kde.ikdeParams = ikdeParams ; 
        else
            out_kde.ikdeParams.maxNumCompsBeforeCompression = length(out_kde.pdf.w) ;
        end
        
        [new_mu, new_Cov, w_out] = momentMatchPdf(out_kde.pdf.Mu, out_kde.pdf.Cov, out_kde.pdf.w) ; 
        input_kde.otherParams.singleGaussApp.Mu = new_mu ;
        input_kde.otherParams.singleGaussApp.Cov = new_Cov ;
    case 'unlearn_with_input'
         if isempty(input_kde)
             out_kde = input_kde ;
             return ;
         end
        
         input_kde.otherParams.maximumsOnPdf = [] ;
         % get unlearning pdf if required
         if isequal(type_init,'points') 
%              t_ikdeParams = initializeParamsKDE( [] ) ;
%              kde_tmp = executeOperatorIKDE( [], 'input_data', input_data,'add_input' ) ;
%              pdf_neg = kde_tmp.pdf ;
%              t_ikdeParams = kde.ikdeParams ;
%              [pdf_neg, t_ikdeParams] = estimateAmiseOptimalKernel( 'model', [], 'obs', input_data,...
%                                 'obs_relative_weights', obs_relative_weights,...
%                                 'ikdeParams', t_ikdeParams,...
%                                 'useMarginalBasedBWs',0,...
%                                 'accountForVirginComponents',1,...
%                                 'max_ibw_iterations', input_kde.otherParams.max_ibw_iterations) ;
%              kde_neg = pdf_neg ;

 

             if isequal(input_kde.otherParams.unlearnPointsType,'separateBandwidth')   
                    kde_neg = executeOperatorIKDE( [], 'input_data', input_data,'add_input' ) ;
             elseif isequal(input_kde.otherParams.unlearnPointsType,'jointBandwidth') 
                 % should MDL completely guide bw_selection and component threshold?
                 if input_kde.otherParams.MDL_guides.MDL_guided_BW_Cprss == 1 ;
                     MDL_guides = input_kde.otherParams.MDL_guides ;
                 else
                     MDL_guides = [] ;
                 end
                 [model_new, ikdeParams, H_opt] = estimateAmiseOptimalKernel( 'model', input_kde.pdf, 'obs', input_data,...
                                'obs_relative_weights', obs_relative_weights,...
                                'ikdeParams', input_kde.ikdeParams,...
                                'useMarginalBasedBWs',0,...
                                'accountForVirginComponents',1,...
                                'max_ibw_iterations', input_kde.otherParams.max_ibw_iterations,...
                                'MDL_guides', MDL_guides,...
                                'useSublayerForDerivativeModel', input_kde.otherParams.useSublayerForDerivativeModel,...
                                'priorBandwidth', input_kde.otherParams.priorBandwidth,...
                                'typePluginCalculation', input_kde.otherParams.typePluginCalculation,...
                                'typeOfKDEInit', input_kde.otherParams.typeOfKDEInit) ;     
%                             input_kde.otherParams.scaleKernelNegative = 2.5^2
                    H_opt = H_opt*input_kde.otherParams.scaleKernelNegative ; 
                    kde_neg = executeOperatorIKDE( [], 'input_data', input_data,'add_input', 'forceBandwidth', H_opt ) ;
             end
                    
         elseif isequal(type_init,'pdf') 
             pdf_neg = input_data ;
             t_ikdeParams = initializeParamsKDE( [] ) ;
             kde_neg.pdf = pdf_neg ;
             kde_neg.ikdeParams = t_ikdeParams ;
         elseif isequal(type_init,'kde') 
%              pdf_neg = input_data.pdf ;
%              t_ikdeParams = input_data.ikdeParams ;
               kde_neg = input_data ;
         end
         
         % if precompression required
         if precompressUnlearning == 1 && ~isequal(type_init,'pdf') 
%              d = size(input_kde.pdf.Mu,1) ;
%              thresh = input_kde.otherParams.compressionClusterThresh^(1/d) ; 
%              typeCompression = 'hierarchical' ;
%              costFunction = 'hellinger';
%              costThreshold = thresh ; 
% 
%              [pdf_neg, t_stateComponents ]= compressPdf( pdf_neg, 'typeCompression', typeCompression,...
%                  'costThreshold', costThreshold,...
%                  'costFunction', costFunction,...
%                  'numberOfSamples', 0 ) ;
             kde_neg.otherParams = input_kde.otherParams ;
             kde_neg = executeOperatorIKDE( kde_neg, 'compress_pdf') ;               
         end
         
%          % construct inputs
%          input_pos = input_kde.pdf ;
%          input_pos.ikdeParams = input_kde.ikdeParams ;         
%          input_neg = pdf_neg;
%          input_neg.ikdeParams = t_ikdeParams ;        
%          
%          d = size(input_kde.pdf.Mu,1) ;                  
%          thresh = input_kde.otherParams.compressionClusterThresh^(1/d) ;
%  
         
         out_kde = kdeUnlearnKde( input_kde, kde_neg, ...
                                  'unlearning_MakeProperPdf', unlearning_MakeProperPdf,...
                                  'usehalfHellinger', input_kde.otherParams.usehalfHellingerInCompression,...
                                  'selectSubDimensions', selectSubDimensions) ;
         
%          % unlearn
%          pdf_res = gaussUnlearn( input_pos, input_neg, ...
%                                  'makeProperPdf', unlearning_MakeProperPdf, ...
%                                  'costThreshold', thresh,...
%                                  'maxComponents', input_kde.otherParams.compressionMaxComponents,...
%                                  'allowCompleteUnlearning', input_kde.otherParams.unlearning_AllowCompleteUnlearning ) ;
%          % turn off virgin components     
% %          input_kde.ikdeParams.virginComps = zeros(1, length(pdf_res.w)) ;
%          % translate
%          out_kde.otherParams = input_kde.otherParams ;
%          out_kde.ikdeParams = pdf_res.ikdeParams ;
%          out_kde.pdf.Mu = pdf_res.Mu ;
%          out_kde.pdf.Cov = pdf_res.Cov ;
%          out_kde.pdf.w = pdf_res.w ;
         [new_mu, new_Cov, w_out] = momentMatchPdf(out_kde.pdf.Mu, out_kde.pdf.Cov, out_kde.pdf.w) ; 
         input_kde.otherParams.singleGaussApp.Mu = new_mu ;
         input_kde.otherParams.singleGaussApp.Cov = new_Cov ;
    case 'compress_pdf'    
         d = size(input_kde.pdf.Mu,1) ;                  
         thresh = input_kde.otherParams.compressionClusterThresh  ;     
         input_kde.otherParams.maximumsOnPdf = [] ;
%          typeCompression = 'hierarchical' , 'meanshift';
    
         costFunction = input_kde.otherParams.costFunctionCompression ; %'numberOfComponents'; ;'alpha_MDL'; %'hellinger' ;
         costThreshold = thresh ; 

         out_kde.otherParams = input_kde.otherParams ;
         out_kde.ikdeParams = input_kde.ikdeParams ;
         [out_kde.pdf] = compressPdf( input_kde.pdf, 'typeCompression', input_kde.otherParams.typeCompression,...
                    'costThreshold', costThreshold,...
                    'costFunction', costFunction,...
                    'numberOfSamples', input_kde.ikdeParams.N_eff,...
                    'granularity_cell_num', input_kde.otherParams.granularity_cell_num,...
                    'typeNoiseDetermination', input_kde.otherParams.typeNoiseDetermination,...
                    'threshOnSplitMethods', input_kde.otherParams.threshOnSplitMethods,...
                    'useSMOprunning', input_kde.otherParams.useSMOprunning,...
                    'useMargHellingerCompression', input_kde.otherParams.useMargHellingerCompression, ...
                    'useLocalDistanceEvaluation', input_kde.otherParams.useLocalDistanceEvaluation,...
                    'applyProjectionToSubspace', input_kde.otherParams.applyProjectionToSubspace,...
                    'compressionDirection', input_kde.otherParams.compressionDirection,...
                    'memorylimit', input_kde.otherParams.MDL_memorylimit,...
                    'memorylimitUseNumComps', input_kde.otherParams.MDL_memorylimitUseComps,...
                    'otherClasses', otherClasses, ...
                    'approximateCost', input_kde.otherParams.approximateCost,...
                    'debugForceCompress', debugForceCompress, ...
                    'minNumberOfComponentsThreshold', input_kde.otherParams.minNumberOfComponentsThreshold) ;   
        [new_mu, new_Cov, w_out] = momentMatchPdf(out_kde.pdf.Mu, out_kde.pdf.Cov, out_kde.pdf.w) ; 
        input_kde.otherParams.singleGaussApp.Mu = new_mu ;
        input_kde.otherParams.singleGaussApp.Cov = new_Cov ;
%          out_kde.ikdeParams.virginComps = stateComponents ;     
    case 'evalPdfOnData' 
        % required : input_kde, 'input_data', 
        % also available: 'selectSubDimensions',
        
        kde1 = input_kde ;
        % verify if the user has chosen a subspace -- then marginalize
        % mixtures as well as data
        if ~isempty(selectSubDimensions)
            kde1.pdf = marginalizeMixture( input_kde.pdf, selectSubDimensions ) ;
            input_data = input_data( selectSubDimensions, : ) ;            
        end
        
        % extract and analyze the current bandwidth -- regularize the
        % null space                        
        [kde1_r, subindicator] = regularizeKDEInBandwidth( kde1, 'practicallyZero', 1e-5 ) ;

        % evaluate pdf
        pdf_data = evaluatePointsUnderPdf( kde1_r.pdf, input_data ) ;
        out_kde = [] ;
        out_kde.subRegularized = subindicator ;
        out_kde.evalpdf = pdf_data ;
     case 'evalTypOnData' 
        % required : input_kde, 'input_data', 
        % also available: 'selectSubDimensions',
        
        [kde, x_max, max_val, subindicator, typ_data] = getTypEvalInDimSpace( input_kde , selectSubDimensions, input_data ) ;
        
        out_kde = [] ;
        out_kde.kde = kde ;
        out_kde.subRegularized = subindicator ;
        out_kde.evaltyp = typ_data ;    
    case 'getSubDimKDE'
        % required : input_kde, 
        % also available: 'selectSubDimensions'
        % extract dimensions and return the kde
        out_kde = marginalizeKDE( input_kde, selectSubDimensions ) ;
    case 'evalHellingerBetween'
         % required : input_kde, 'additional_kde',  
         % also available: 'selectSubDimensions'
         kde1 = input_kde ;
         kde2 = additional_kde ;

         % verify if the user has chosen a subspace -- then marginalize
         % mixtures as well as data
         if ~isempty(selectSubDimensions)
            kde1.pdf = marginalizeMixture( kde1.pdf, selectSubDimensions ) ;
            kde2.pdf = marginalizeMixture( kde2.pdf, selectSubDimensions ) ;            
         end
         
         % extract and analyze the current bandwidth -- regularize the
         % null space                        
         [kde1_r, subindicator1] = regularizeKDEInBandwidth( kde1, 'practicallyZero', 1e-5 ) ;
        
         % extract and analyze the current bandwidth -- regularize the
         % null space                        
         [kde2_r, subindicator2] = regularizeKDEInBandwidth( kde2, 'practicallyZero', 1e-5 ) ;
         
         subindicator = max([ subindicator1, subindicator2 ]) ;
         distance_hell = uHellingerJointSupport2_ND( kde2_r.pdf, kde1_r.pdf  ) ;
         out_kde = [] ;
         out_kde.subRegularized = subindicator ;
         out_kde.distance_hell = distance_hell ;
    case 'showKDE'
         if ~isempty(selectSubDimensions)
            input_kde.pdf = marginalizeMixture( input_kde.pdf, selectSubDimensions, 0 ) ;            
         end
         % extract and analyze the current bandwidth -- regularize the
         % null space                        
         [input_kde, subindicator] = regularizeKDEInBandwidth( input_kde, 'practicallyZero', 1e-5 ) ;
 
         if size(input_kde.pdf.Mu,1) == 1
             showkdecolor = 'r' ;
         end
         visualizeKDE('kde', input_kde, 'input_data', input_data, 'tabulated',...
             showTabulated, 'showkdecolor', showkdecolor) ;
         out_kde = [] ;
end
 
% --------------------------------------------------------------------- %
function [ikdeParams, otherParams ]= initializeParamsKDE( inputPdf, showMessages ) 
if nargin < 2
    showMessages = 0 ;
end

priontoutInitMessage( 'By initialization:', showMessages ) ;
if ~isempty(inputPdf)
    % initialize sufficient stats
    ikdeParams.suffSt.C_t = sum(inputPdf.w) ;
    ikdeParams.suffSt.w_att = 1.0 ;  
    ikdeParams.suffSt.K_eff = sum(inputPdf.w.^2) ;
%     ikdeParams.virginComps = ones(1,length(inputPdf.w))*0 ;
    
    % estimate scale of distribution
    [new_mu, C] = momentMatchPdf( inputPdf.Mu, inputPdf.Cov, inputPdf.w ) ;
    ikdeParams.scale.Cov = C ;     
    ikdeParams.N_eff = length(inputPdf.w) ;
else
    % initialize sufficient stats
    ikdeParams.suffSt.C_t = 0 ;
    ikdeParams.suffSt.w_att = 1.0 ;
    ikdeParams.suffSt.K_eff = 0 ;
%     ikdeParams.virginComps = [] ;     
    ikdeParams.scale.Cov = [] ; 
    ikdeParams.N_eff = 0 ;
end
ikdeParams.maxNumCompsBeforeCompression = -1 ;
otherParams.compressionClusterThresh = 0.02  ;
otherParams.compressionMaxComponents = Inf ;
otherParams.usehalfHellingerInCompression = 0 ;
otherParams.max_ibw_iterations = 10 ;
otherParams.costFunctionCompression = 'MDL';  % 'MDL' , 'hellinger' ; Note that MDL is also hellinger but different search!!
otherParams.granularity_cell_num = 50 ; 
otherParams.typeNoiseDetermination = 'granularity' ; % 'granularity'  'inflation'

% whether MDL should trigger compression and bandwidth selection
% it will likely take more time
otherParams.MDL_guides.MDL_guided_BW_Cprss = 0 ;
otherParams.MDL_guides.typeNoiseDetermination = 'inflation' ; % 'granularity' 'inflation'
otherParams.MDL_guides.granularity_cell_num = 50 ;

% threshold on component splitting
otherParams.threshOnSplitMethods = inf ;
 
% prune before compression
otherParams.useSMOprunning = 0 ;
otherParams.debug.lastCov = {} ;
 
% by default use marginal Hellinger evaluation for compression
otherParams.useMargHellingerCompression = 0 ;

% to use the components in the sublayer for derivative estimation in the BW selection
otherParams.useSublayerForDerivativeModel = 0 ;

% to use local distance evaluation in compression algorithm
otherParams.useLocalDistanceEvaluation = 1 ;

% use subspace projection in compression of a pdf
otherParams.applyProjectionToSubspace = 1 ;
 
% use Affinity propagation for preprocessing when number of samples is large
% (appropriate for batch KDE with compression)
otherParams.applyAPPreprocessingStep = 0 ;

otherParams.compressionDirection = 'topDown' ;

otherParams.MDL_memorylimit = inf ; % [] ;

otherParams.priorBandwidth = [] ;

otherParams.typePluginCalculation = 'one_Stage' ; %'n_Stage'

otherParams.typeOfKDEInit = 'directPlugin' ;

otherParams.MDL_memorylimitUseComps = 0 ;

otherParams.unlearnPointsType = 'jointBandwidth' ; % 'jointBandwidth' 'separateBandwidth' ;

otherParams.approximateCost = 0 ; % 0 , 1

otherParams.typeCompression = 'hierarchical' ; % 'meanshift'

otherParams.useSomeOtherKindOfEstimator = [] ;

otherParams.minNumberOfComponentsThreshold = 2 ; % lower bound on the number of components in the mixture
otherParams.scaleKernelNegative  = 2.5^2 ;

otherParams.figJainEM.mink = 1 ;                         
otherParams.figJainEM.maxk = 50 ;
otherParams.figJainEM.regularize = [] ; 
otherParams.figJainEM.minth = [] ; 
otherParams.figJainEM.convoption = [] ; 

otherParams.maximumsOnPdf = [] ;
otherParams.singleGaussApp = [] ;

msg = sprintf('Min number of components: %d', otherParams.minNumberOfComponentsThreshold ) ;
priontoutInitMessage( msg, showMessages ) ;
priontoutInitMessage( '------- End by initialization -------', showMessages ) ;

% ----------------------------------------------------------------------- %
function priontoutInitMessage( msg, showthis )

if showthis == 1
    disp(msg) ;
end




