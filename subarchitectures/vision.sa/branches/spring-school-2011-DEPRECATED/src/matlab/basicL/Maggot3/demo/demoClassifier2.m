function demoClassifier2()

plotting_activated = 1 ;
num_add_classes = 0 ;
nfake_dims = 0 ;
dim = nfake_dims + 2 ;

lbl={'r', 'g', 'b', 'c', 'm', 'k', 'y', 'k'} ;

% - parameters of the online classifier
switchSelectionSeeds = 0 ;                      % (0) turn off approximative compression
classifyWithPosterior = 1 ;                     % (1) use posterior info for classification
typeRecDescr = 'dKDE' ;                         % 'dKDE', 'oKDE', switchess between discriminative and standard oKDE
% minNumDataPointsToFormKDE = dim + 1 ;         % (int) minimum number of points before forming a KDE
minNumDataPointsToFormKDE = 2 ; %(dim^2-dim)/2+dim+dim ; % (int) minimum number of points before forming a KDE
react_compression_to_feature_selection = 0 ;    % (1) apply compression in subdimension selected by feature selection
min_samps_per_model_feat_sel = (dim^2-dim)/2+dim+dim ; % (int) minimum number of samples per model observed before applying feature selection
min_th_feat_sel = 0.1 ;                         % (double) threshold on importance below which a feature is removed
costThreshold.thReconstructive = 0.01 ;         % thresholds on reconstructive and discriminative compression
costThreshold.thDiscriminative = 0.02  ; %0.02 ;
autoUpdateThres_upper = 0.07 ;                   % in self verified mode, this is the threshold on entropy for asking
autoUpdateThres_lower = 1e-2 ;                  % in self verified mode, this is the threshold on entropy for auto update
random_fselect_threshold = 1 ; 0.95 ;               % probability of feature selection occuring when called
pair_dist_struct_use_approx = 1 ;               % switch for compression: test which classes shuld be taken into account during compression of i-th class
force_value_init_of_maxNumCompsBeforeCompression = 2 ; % determine initial num of components before compression
turn_off_splitting = 1 ;
% - end of parameters
N_init = 1 ;  

% generate test data
[xc, p_ref ]= generateDots( 1000, 0, nfake_dims, num_add_classes ) ;

% save('testing.mat') ;
% load('testing.mat') ;
opt_score = testClassifications( p_ref, [], 1, xc ) ;
% data generated
 

maxNumCompsBeforeCompression = 5 ;

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
                    'pair_dist_struct_use_approx', pair_dist_struct_use_approx, ...
                    'force_value_init_of_maxNumCompsBeforeCompression', force_value_init_of_maxNumCompsBeforeCompression,...
                    'turn_off_splitting', turn_off_splitting) ;
 
                
warning off ;                

Num_questions = N_init ;
input_data = {} ;
for i_c = 1 : length(xc)
    indat = [] ;
    indat.data = xc{i_c}(:,1:N_init) ; 
%     indat.class_name = num2str(i_c) ;
    indat.class = i_c ;
    input_data = horzcat(input_data, indat) ;
end
kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input', 'turn_off_splitting', turn_off_splitting ) ;
  
c_score = [] ;
% continue to add new data
for i = N_init+1 : 200
    %- create input data
    input_data = {} ; 
    for i_c = 1 : length(xc)
        indat = [] ;
        indat.data = xc{i_c}(:,i) ; indat.class_name = num2str(i_c) ;
        input_data = horzcat(input_data, indat) ;       
    end
    
%     disp('Update...')
    tic
    kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input', ...
                                        'autonomous_update', 'pure_oracle',...
                                        'autoUpdateThres_upper', 0.15,...
                                        'turn_off_splitting', turn_off_splitting ) ; % 'self_verified', 'oracle_verified', 'pure_oracle'
    toc
    Num_questions = Num_questions + sum(kde_cl.answers) ;
  
    % perform a feature selection 
%     disp('Select features')
    tic
    kde_cl = executeOperatorIKDEClsfr( kde_cl, 'make_simple_feature_selection') ;
    toc
    if isfield(kde_cl, 'debug')
        for_plot = kde_cl.debug ;
    else
        for_plot = [] ;
    end

    % draw distributions
    if plotting_activated == 1
        figure(1) ; clf ;
        subplot(1,4,1) ; hold on ;
        for i_c = 1 : length(xc)
            plot(xc{i_c}(1,1:i), xc{i_c}(2,1:i), [lbl{i_c},'.']) ; hold on ;
        end 
        axis equal ;
         
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
        subplot(1,4,3) ; axis equal ;
        
        cls_name = [] ;
        for i_c = 1 : length(xc)
             executeOperatorIKDEClsfr( kde_cl, 'showKDE_of_class_index', i_c, 'showkdecolor', lbl{i_c} ) ;
             cls_name1 = executeOperatorIKDEClsfr( kde_cl, 'get_name_at_index', i_c  ) ;
             cls_name = [cls_name, ' , ', cls_name1 ] ; hold on ;
        end
        title( cls_name ) ; axis equal ;
  
        subplot(1,4,4) ;
        bar(kde_cl.cummulative_feat_costs) ;
        
        figure(2) ; clf ;
        score = testClassifications( p_ref, kde_cl, 0 ) ;
        
        c_score = [c_score, score] ;
        plot(1:length(c_score), c_score) ;
        title(['score_{opt}= ' , num2str(opt_score), ' , score_{curr}= ' , num2str(score)]) ;
        axis([0,i,0,1]) ; 
        
        text(i/3,0.5, ['Sel feats: ' , num2str(kde_cl.sub_selected_features)])  ;
        text(1/3,0.3, ['N_{q} per class: ' , num2str(Num_questions/2)])  ;
        
    end
end


% save('score.txt','c_score', '-mat') ;

% calculate gains of learning some data-point
input_data = {} ;
indat = [] ;
indat.data = xc{1}(:,500) ; 
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
% indat = [] ;
% indat.data = xc1(:,i) ; indat.class = 2 ;
% kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', {indat}, 'unlearn_with_input'  ) ;

% ----------------------------------------------------------------------- %
function score = testClassifications( p_ref, kde_cl, refrnc, xctest )
N_points = 1000 ;

len = 0 ;
c_c = 0 ;
for i_c = 1 : length(p_ref)
%     if refrnc == 0 
        xc = sampleGaussianMixture( p_ref{i_c}, N_points ) ; 
%     else
%        xc = xctest{i_c}  ;
%     end
    
    if refrnc == 0         
        rslt = executeOperatorIKDEClsfr( kde_cl, 'input_data', xc, ...
                                     'classifyData', 'use_unknown_model',0   ) ;         
                                 len = len + size(xc,2) ;
    else
      P = [] ;
       for j_c = 1 : length(p_ref) 
           p = evaluatePointsUnderPdf( p_ref{j_c}, xc) ;
           P = [P; p(1:999)] ;
       end
       [tt,C] = max(P) ;
       rslt.C = C ; 
       len = len + size(xc,2) ;
    end
    c_c = c_c + sum(rslt.C == i_c) ;
    
end
score = c_c / (len ) ;


% ----------------------------------------------------------------------- %
function [xc pdfs] = generateDots( N_points, deltaC2, nfake_dims, num_add_classes )
 

% pdf.Mu = [ [0;1] , [-1;0], [0;-1]  ] ;
% C = eye(2)*0.01 ;
% pdf.Cov = { C, C , C  } ;
% pdf.w = ones(1,size(pdf.Mu,2)) ; pdf.w = pdf.w / sum(pdf.w) ;
% 
% pdf0.Mu = [ [0+deltaC2;0]  ] ;
% C = [0.1, 0 ; 0 , 0.1] ;
% pdf0.Cov = { C } ;
% pdf0.w = [1] ;
% 
%  
% xc1 = sampleGaussianMixture( pdf, N_points ) ; 
% xc2 = sampleGaussianMixture( pdf0, N_points ) ; 
% p_ref = [] ;
% if nargout == 3
%     p_ref = {pdf, pdf0} ;
% end
% xc = horzcat({xc1},{xc2}) ;

M_x = [2 -3 -5 4 3 5 5 6 7 ; 3 3 4 5 2 3 1 3 1; rand(nfake_dims, 9)*0] ;
M = [ [ [0;1] , [-1;0], [0;-1]  ] ; rand(nfake_dims, 3)*0 ] ;
M0  = [ [0+deltaC2;0] ; rand(nfake_dims, 1)*0 ]  ;
 
pdf.Mu = M ;
C = ones(1,2+nfake_dims)*0.01 ;
C(3:2+nfake_dims) = 4 ;
C = diag(C) ;
pdf.Cov = { C, C , C  } ;
pdf.w = ones(1,size(pdf.Mu,2)) ; pdf.w = pdf.w / sum(pdf.w) ;

pdf0.Mu = M0 ;
C = ones(1,2+nfake_dims)*0.1 ;
C(3:2+nfake_dims) = 4 ;
C = diag(C) ;
pdf0.Cov = { C } ;
pdf0.w = [1] ;

pdfs = {pdf, pdf0} ;
for i = 1 : num_add_classes
    pdfx.Mu = M_x(:,i) ;
    C = [0.1, 0.1] ; C(3:2+nfake_dims) = 4 ;
    C = diag(C) ;
    pdfx.Cov = { C } ;
    pdfx.w = [1] ;  
    pdfs = horzcat(pdfs, pdfx) ;
end

xc = {} ;
for i = 1 : num_add_classes+2   
    xc_t = sampleGaussianMixture( pdfs{i}, N_points ) ; 
    xc = horzcat(xc, xc_t) ;
end
 




 
 