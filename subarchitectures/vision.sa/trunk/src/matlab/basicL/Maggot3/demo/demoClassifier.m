function demoClassifier()

nfake_dims = 0 ;
dim = nfake_dims + 2 ;

% - parameters of the online classifier
switchSelectionSeeds = 0 ;                      % (0) turn off approximative compression
classifyWithPosterior = 1 ;                     % (1) use posterior info for classification
typeRecDescr = 'dKDE' ;                         % 'dKDE', 'oKDE', switchess between discriminative and standard oKDE
minNumDataPointsToFormKDE = (dim^2-dim)/2+dim+dim ; % (int) minimum number of points before forming a KDE
react_compression_to_feature_selection = 1 ;    % (1) apply compression in subdimension selected by feature selection
min_samps_per_model_feat_sel = (dim^2-dim)/2+dim+dim ; % (int) minimum number of samples per model observed before applying feature selection
min_th_feat_sel = 0.1 ;                           % (double) threshold on importance below which a feature is removed
costThreshold.thReconstructive = 0.05 ;         % thresholds on reconstructive and discriminative compression
costThreshold.thDiscriminative = 0.05 ;
autoUpdateThres_upper = 0.1 ;                   % in self verified mode, this is the threshold on entropy for asking
autoUpdateThres_lower = 1e-2 ;                  % in self verified mode, this is the threshold on entropy for auto update
random_fselect_threshold = 0*0.05 ;               % probability of feature selection occuring when called
pair_dist_struct_use_approx = 1 ;               % switch for compression: test which classes shuld be taken into account during compression of i-th class
turn_off_splitting = 1 ;
% - end of parameters

% generate test data
 [xc1, xc2, p_ref] = generateDots( 1000, 0, nfake_dims ) ;
% data generated
 
% create a classifier object with selected parameters
kde_cl = executeOperatorIKDEClsfr( [], 'init', 'compressionClusterThresh', costThreshold,...
                    'typeRecDescr', typeRecDescr, ...
                    'switchSelectionSeeds', switchSelectionSeeds,...
                    'minNumDataPointsToFormKDE', minNumDataPointsToFormKDE, ...
                    'react_compression_to_feature_selection', react_compression_to_feature_selection,...
                    'min_samps_per_model_feat_sel', min_samps_per_model_feat_sel,... 
                    'min_th_feat_sel', min_th_feat_sel,...
                    'autoUpdateThres_upper', autoUpdateThres_upper,...
                    'autoUpdateThres_lower', autoUpdateThres_lower,...
                    'random_fselect_threshold', random_fselect_threshold,...
                    'pair_dist_struct_use_approx', pair_dist_struct_use_approx,...
                    'turn_off_splitting', turn_off_splitting) ;
N_init = 50 ;
% construct data only for class 1                
input_data = {} ; indat = [] ;
indat.data = xc1(:,1:N_init) ; 
% indat.class =1 ;
indat.class_name = 'red' ;  
input_data = horzcat(input_data, indat) ;              
           
% initialize the class 1  
kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input' ) ;

% construct data only for class 2                
input_data = {} ; indat = [] ;
indat.data = xc2(:,1:N_init) ; 
% indat.class = 2 ;
indat.class_name = 'blue' ;  
input_data = horzcat(input_data, indat) ;              
           
% initialize the class 2  
kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input' ) ;


% continue to add new data
for i = N_init+1 : 200
    %- create input data
    input_data = {} ; 
    indat = [] ;
    indat.data = xc1(:,i) ; indat.class = 1 ;
    input_data = horzcat(input_data, indat) ;
    indat = [] ;
    indat.data = xc2(:,i) ; indat.class = 2 ;
    input_data = horzcat(input_data, indat) ;
    
    % add input to kde
    tic
    kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input' ) ;
    toc
    
    % perform a feature selection 
    kde_cl = executeOperatorIKDEClsfr( kde_cl, 'make_simple_feature_selection') ;
    if isfield(kde_cl, 'debug')
        for_plot = kde_cl.debug ;
    else
        for_plot = [] ;
    end
    
    % ------- some testing -------- %
    % 1. try to classify some input
    rslt = executeOperatorIKDEClsfr( kde_cl, 'input_data', xc1(:,500), 'classifyData',...
                                     'use_unknown_model',1, 'extensive_answer', 1  ) ;
    % display results %                             
    msg = sprintf('Posterior over classes (last one is unknown: %f, %f , %f )', rslt.P(1), rslt.P(2), rslt.P(3)) ;
    disp(msg) ;
     
   
    cls_name = executeOperatorIKDEClsfr( kde_cl, 'get_name_at_index', rslt.C  ) ;  
    msg = sprintf('Classified as: %s, Entropy: %f, R1: %f, R2: %f', cls_name, rslt.H, rslt.R1, rslt.R2 ) ; disp(msg) ;
    if rslt.H >  0.3
        disp('Uncertain classification.') ;
    else
        disp('Certain Classification.') ;
    end
    % results displayed %
    
    % draw distributions
    figure(1) ; clf ;
    subplot(1,4,1) ;
    plot(xc1(1,1:i), xc1(2,1:i), 'r.') ; hold on ; plot(xc2(1,1:i), xc2(2,1:i), 'b.') ; 
    msg = sprintf('N_{samps}=%d', i); title(msg) ;
    subplot(1,4,2) ;
    if ~isempty(for_plot)
        plot(1:length(kde_cl.debug.Cost(1,:)), kde_cl.debug.Cost(4,:),'g') ;  hold on ; %/kde_cl.debug.Cost(4,size(kde_cl.debug.Cost,2))
        plot(1:length(kde_cl.debug.Cost(1,:)), [1:length(kde_cl.debug.Cost(1,:))]*0 + kde_cl.min_th_feat_sel,'k') ;
%          plot(1:length(kde_cl.debug.Cost(1,:)), kde_cl.debug.Cost(1,:)./kde_cl.debug.Cost(2,:),'g') ;
        
        a = axis ; a(3) = 0 ; axis(a) ;
        set(gca,'XTick',[1:length(kde_cl.debug.Cost(1,:))]) ;
        set(gca,'XTickLabel', {kde_cl.debug.Cost(3,:)'})
        title('f_{scores}'); %axis tight
%         figure(3) ; bar(kde_cl.debug.Cost(5,:)) ; figure(1) ;
    end
    subplot(1,4,3) ; 
    executeOperatorIKDEClsfr( kde_cl, 'showKDE_of_class_index', 1, 'showkdecolor', 'r'  ) ;
    cls_name1 = executeOperatorIKDEClsfr( kde_cl, 'get_name_at_index', 1  ) ;
    title(cls_name) ;
    
    hold on ;
    executeOperatorIKDEClsfr( kde_cl, 'showKDE_of_class_index', 2, 'showkdecolor', 'b'  ) ;
    cls_name2 = executeOperatorIKDEClsfr( kde_cl, 'get_name_at_index', 2  ) ;
    title([cls_name1, ', ', cls_name2]) ;   
        
    subplot(1,4,4) ;
    bar(kde_cl.cummulative_feat_costs) ;
 
end

% calculate gains of learning some data-point
input_data = {} ;
indat = [] ;
indat.data = xc1(:,500) ; 
input_data = horzcat(input_data, indat) ;
rslt = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data{1}.data, 'calculate_gains'  )  ; 
disp('Gains for learning some datapoint:')
rslt{1}

% introspect models % 
rslt = executeOperatorIKDEClsfr( kde_cl, 'introspect' ) ;
disp('Introspection confusion matrix:')
rslt.Con_matrix
% interpret:
msg = sprintf('Portion of the first model that is explained by the second model: %f', rslt.Con_matrix(1,2) ) ; disp(msg) ;
msg = sprintf('Portion of the second model that is explained by the first model: %f', rslt.Con_matrix(2,1) ) ; disp(msg) ;

% list the number of classes learned so far
index_list = executeOperatorIKDEClsfr( kde_cl, 'get_class_indexes' ) 
name_list = executeOperatorIKDEClsfr( kde_cl, 'get_class_names' )

% change the name of class 2
kde_cl = executeOperatorIKDEClsfr( kde_cl, 'set_name_at_index', 2, 'cyan' ) ;

% unlearn class 2 %
indat = [] ;
indat.data = xc1(:,i) ; indat.class = 2 ;
kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', {indat}, 'unlearn_with_input'  ) ;

% ----------------------------------------------------------------------- %
function [xc1, xc2, p_ref] = generateDots( N_points, deltaC2, nfake_dims )
 
pdf.Mu = [ [0;1] , [-1;0], [0;-1]  ] ;
C = eye(2)*0.01 ;
pdf.Cov = { C, C , C  } ;
pdf.w = ones(1,size(pdf.Mu,2)) ; pdf.w = pdf.w / sum(pdf.w) ;

pdf0.Mu = [ [0+deltaC2;0]  ] ;
C = [0.1, 0 ; 0 , 0.1] ;
pdf0.Cov = { C } ;
pdf0.w = [1] ;

xc1 = sampleGaussianMixture( pdf, N_points ) ; 
xc2 = sampleGaussianMixture( pdf0, N_points ) ; 

xc1 = [xc1; rand(nfake_dims,size(xc1,2))] ; 
xc2 = [xc2; rand(nfake_dims,size(xc2,2))] ; 

p_ref = [] ;
if nargout == 3
    p_ref = {pdf, pdf0} ;
end
 
 