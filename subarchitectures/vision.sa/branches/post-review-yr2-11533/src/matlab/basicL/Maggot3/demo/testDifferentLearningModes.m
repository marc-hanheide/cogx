function testDifferentLearningModes() 

% folder that contains your dataset
rootdirmy = 'D:\Work\Matlab\IncrementalKDE\Maggot3Data\' ;

datname = 'objectdataset_colors' ;
dbname = [rootdirmy, datname,'\objectdataset_colors.mat'] ;
% datname = 'colorDataset' ;
% dbname = [rootdirmy, datname,'\database_color_five_fold.mat'] ;
database = load(dbname) ;
database = database.database ;
 
just_show_results = 0  ;
sel_mode_of_operations = 3 ;% : 3 ;

for sel_mode_now = 1 : length(sel_mode_of_operations )
    
    sel_mode_of_operation = sel_mode_of_operations(sel_mode_now) ;
    modes_of_operation = {'TD', 'NSTA', 'STA'} ;
    perform_feature_selection = 0 ;
       
    switch(modes_of_operation{sel_mode_of_operation})
        case 'TD'
            proactive_mode = 0 ;
            type = 'pure_oracle' ;
        case 'NSTA'
            proactive_mode = 1 ;
            type = 'pure_oracle'; %'situated_verified'; % 'pure_oracle' ; % 'self_verified'
        case 'STA'
            proactive_mode = 0 ;
            type = 'situated_verified' ;
    end
        
    H = 0.005 ;  0.03 ;
    H_low = 0*0.0001; %H ; %H ;0.11 ;  
    if isequal(type,'pure_oracle')
        H = 0 ;
    end
    
    out_name = [rootdirmy, datname,'\random_fsel', num2str(perform_feature_selection),'_',type,'H', num2str(H),'_mode',modes_of_operation{sel_mode_of_operation},'.mat'] ;
    
    if just_show_results == 0
        kde_models = {} ;
        R_resul = {} ;
        for i_repeats = 1 :  1
            for i = 1 : length(database.folds)
                [R_res, kde_cl,C_result,C_reference] =...
                    classifier_test( database.folds{i}, type, ...
                    perform_feature_selection, proactive_mode, H, H_low,...
                    modes_of_operation{sel_mode_of_operation} ) ;
                
                R_resul = horzcat(R_resul, {R_res}) ;
                kde_models = horzcat(kde_models, kde_cl) ;
            end
            
        end
        R_results.results = R_resul ;
        R_results.database = dbname ;
        R_results.C_result = C_result ; % rezultat klasifikacije
        R_results.C_reference = C_reference ; % ground truth klasi
        R_results.kde_models = kde_models ;
        save(out_name,'-mat') ;
    end
    
    r = load(out_name) ;
    % clf ;
    analyzePerformance( r.R_results, modes_of_operation{sel_mode_of_operation} )
end

% ---------------------------------------------------------------------- %
function analyzePerformance( R_results, mode_of_operation )

switch(mode_of_operation) 
    case 'TD'
         c = 'r' ;
         labl = 'tutor driven' ;
    case 'NSTA'
         c = 'g' ;
         labl = 'nonsituated tutor asisted' ;
    case 'STA'
        c = 'c:' ;
         labl = 'situated tutor asisted' ;
end
delta_cost = 0.25 ;
max_cost = 0 ;
max_len = 0 ;
for i = 1 : length(R_results.results)
     cs_cost = cumsum(R_results.results{i}(5,:)) ;
     max_cost = max([max_cost, max(cs_cost)]) ;
     max_len = max([max_len, size(R_results.results{i},2)]) ;
end
    
cost_axis_x = [ 0 : delta_cost : max_cost ] ;
n_entries_cost_axis_y = zeros(1, length(cost_axis_x)) ;
cost_axis_y = zeros(1, length(cost_axis_x)) ;
recgn = zeros(1, max_len) ;
n_entries_recgn = zeros(1, max_len) ;
cumcost = zeros(1,max_len) ;
n_entries_cs_cost = zeros(1,max_len) ;
for i = 1 : length(R_results.results)    
    cs_cost = cumsum(R_results.results{i}(5,:)) ;  
    cumcost = cumcost + cs_cost ;
    n_entries_cs_cost(1:length(cs_cost)) = n_entries_cs_cost(1:length(cs_cost)) + 1 ;
    
    for j = 1 : length(cost_axis_x)
       idx = find(cs_cost <= cost_axis_x(j)) ;
       if isempty(idx)
        cost_axis_y(j) = 0 ; 
       else
        cost_axis_y(j) = cost_axis_y(j) + R_results.results{i}(6,idx(length(idx))) ;   
        n_entries_cost_axis_y(j) = n_entries_cost_axis_y(j) + 1 ;
       end       
    end   
    
    recgn = recgn + R_results.results{i}(6,:) ;
    n_entries_recgn(1:length(R_results.results{i}(6,:) )) = ...
        n_entries_recgn(1:length(R_results.results{i}(6,:) )) + 1 ;
end
cost_axis_y = cost_axis_y ./ (n_entries_cost_axis_y+1e-50) ;
recgn = recgn ./ (n_entries_recgn + 1e-50) ;
cumcost = cumcost ./ (n_entries_cs_cost + 1e-50) ;
 
subplot(1,3,1) ; hold on ;
plot(cost_axis_x, cost_axis_y, c, 'LineWidth', 2) ; 
xlabel('Cummulative effort') ; ylabel('Score') ;
a = axis ; a(4) = 1; a(3) = 0 ; axis(a) ;
subplot(1,3,2) ; hold on ;
plot(1:length(recgn), recgn, c, 'LineWidth', 2) ; 
xlabel('N_{observations}') ; ylabel('Score') ;
a = axis ; a(4) = 1; a(3) = 0 ; axis(a) ;
subplot(1,3,3) ; hold on ;
plot(1:length(cumcost), cumcost, c, 'LineWidth', 2) ; 
xlabel('N_{observations}') ; ylabel('Cummulative effort') ;

 
% ---------------------------------------------------------------------- %
function [R_results, kde_cl, C_result, C_reference] = classifier_test( database, type_update, perform_feature_selection, proactive_mode, autoUpdateThres_upper, autoUpdateThres_lower, mode_of_operation )

Cshowselected = 1 ;
Cdescribe = 1 ;
Clisten = 0.25 ;
C_showrand = 0.25 ;
% 'self_verified', 'oracle_verified', 'pure_oracle' type_update
% perform_feature_selection = fsel ;
plotting_activated = 0 ;
% num_add_classes = 2 ;
% nfake_dims = 5 ;
% dim = nfake_dims + 2 ;

dim = size(database.learn_base,1) ;

lbl={'r', 'g', 'b', 'c', 'm', 'k', 'y', 'k'} ;

% - parameters of the online classifier
switchSelectionSeeds = 0 ;                      % (0) turn off approximative compression
classifyWithPosterior = 1 ;                     % (1) use posterior info for classification
typeRecDescr = 'dKDE' ;                         % 'dKDE', 'oKDE', switchess between discriminative and standard oKDE
% minNumDataPointsToFormKDE = dim + 1 ;           % (int) minimum number of points before forming a KDE
minNumDataPointsToFormKDE = (dim^2-dim)/2+dim+dim ; % (int) minimum number of points before forming a KDE
react_compression_to_feature_selection = 0 ;    % (1) apply compression in subdimension selected by feature selection
min_samps_per_model_feat_sel = (dim^2-dim)/2+dim+dim ; % (int) minimum number of samples per model observed before applying feature selection
min_th_feat_sel = 0.1 ;                       % (double) threshold on importance below which a feature is removed
costThreshold.thReconstructive = 0.1 ;         % thresholds on reconstructive and discriminative compression
costThreshold.thDiscriminative = 0.1;
probability_of_feature_selection = 0.1 ;
% - end of parameters
N_init = 20 ;  

% generate test datatestClassifier_dataset
% [xc, p_ref ]= generateDots( 1000, 0, nfake_dims, num_add_classes ) ;
% opt_score = testClassifications( p_ref, [], 1, xc ) ;
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
                    'turn_off_splitting', 1,...
                    'random_fselect_threshold', probability_of_feature_selection) ;

Num_questions = N_init ;
input_data = {} ;
for i_c = 1 : N_init
    cls_pref = [] ;
%     if proactive_mode == 1
%         cls_pref = i_c ;
%         if cls_pref > length(database.index_classes)
%            cls_pref = [] ; 
%         end
%     end
    
    if i_c < length(database.current_pos_classes)
        cls_pref = i_c ;
    end
    
    [ret, database] = get_new_datapoint_from_database( database , cls_pref ) ;
    indat.data = ret.data ; 
    indat.class_name = num2str(ret.class) ;
    input_data = horzcat(input_data, indat) ;
end
 
kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input' ) ;
 

last_class = input_data{length(input_data)}.class_name ;
no_more_classes = [] ;
curr_id = N_init ;
data_in_database = 1 ; 
R_results = [] ;
% continue to add new data
while data_in_database == 1
    %- create input data
    input_data = {} ; 
  
    Cost_comunication = 0 ;
    % determine class
    cls_pref = [] ;
    if proactive_mode == 1
        n_classes = {} ;
        for i = 1 : length(no_more_classes)
            n_classes = horzcat(n_classes, num2str(no_more_classes(i))) ;
        end
        
        cls_pref_t = executeOperatorIKDEClsfr( kde_cl, 'auto_choose_model', 'exclude_model', n_classes ) ; 
   
        cls_pref = str2double(cls_pref_t.class_name) ;
        if sum(no_more_classes == cls_pref) > 0
            cls_pref = [] ;
        end
%         Cost_comunication = Clisten + Cshowselected ;
    else
%         modes_of_operation = {'TD', 'NSTA', 'STA'} ;
%         Cost_comunication = C_showrand ;
    end
    
    switch(mode_of_operation)
        case 'TD'
            Cost_comunication = Cost_comunication + C_showrand + Cdescribe ;
        case 'NSTA'
            Cost_comunication = Cost_comunication + Clisten + Cshowselected ;
        case 'STA'  
            Cost_comunication = Cost_comunication + C_showrand   ;
    end
    
    try
        [ret, database] = get_new_datapoint_from_database( database , cls_pref ) ;
    catch
        error('It is possible, that classifier was not initialized from all classes!') ;
    end
    
    if isempty(ret)
       no_more_classes = [no_more_classes, cls_pref] ;
       continue ; 
    end
    
    if isequal(ret, -1)
        break ;
    end
    
    indat.data = ret.data ; 
    indat.class_name = num2str(ret.class) ;
    input_data = horzcat(input_data, indat) ;

    % classify to determine which class it is
    rslt = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data{1}.data, ...
                                     'classifyData', 'use_unknown_model', 1, 'extensive_answer', 1  ) ;
    C = executeOperatorIKDEClsfr( kde_cl, 'get_name_at_index', rslt.C ) ;
    rsl1 = [str2num(indat.class_name); str2num(C); rslt.H; curr_id] ; 
    % --------
 
    
    
     kde_clx = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input', ...
                                        'autonomous_update', type_update,...
                                        'autoUpdateThres_upper', autoUpdateThres_upper  ) ;
    
    
    tic
    kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input', ...
                                        'autonomous_update', type_update,...
                                        'autoUpdateThres_upper', autoUpdateThres_upper  ) ; % 'self_verified', 'oracle_verified', 'pure_oracle' type_update
    toc
    if proactive_mode == 1 || ~isequal(type_update,'pure_oracle')
        q_here = Cost_comunication*length(kde_cl.answers) + sum(kde_cl.answers) ; 
    else
        q_here = Cost_comunication*length(kde_cl.answers) + sum(kde_cl.answers) ;
    end
  
    % perform a feature selection 
    if perform_feature_selection == 1
        kde_cl = executeOperatorIKDEClsfr( kde_cl, 'make_simple_feature_selection') ;
    end 
    
    [ score, C_result ]= evaluateModel( kde_cl, database.test_base ) ;
    
    C_reference = database.test_base( size(database.test_base, 1) , : ) ;
    full_diag = [rsl1;q_here; score] ;
    R_results = [ R_results, full_diag ] ;
    curr_id = curr_id + 1 ;
end

% ---------------------------------------------------------------------- %
function [score, C_result] = evaluateModel( kde_cl, test_base ) 

data = test_base(1:size(test_base,1)-1,:) ;
rslt = executeOperatorIKDEClsfr( kde_cl, 'input_data', data, ...
                                 'classifyData', 'use_unknown_model',...
                                 0, 'extensive_answer', 0  ) ;
trans_table = [] ;
for i = 1 : length(kde_cl.kde_cl)
   id = str2num(kde_cl.class_labels_names{i}) ;
   trans_table = [trans_table, [i; id]] ;
end

for i = 1 : length(rslt.C)
    rslt.C(i) = trans_table(2,rslt.C(i)) ;
end

C_result = rslt.C ;
D = rslt.C - test_base( size(test_base, 1) , : ) ;
score = sum(D == 0)/length(D) ;


 
% ---------------------------------------------------------------------- %
function R_results = demoClassifier()

test_for_ROC = 1 ;

perform_feature_selection = 0 ;
plotting_activated = 0 ;
num_add_classes = 2 ;
nfake_dims = 1 ;
dim = nfake_dims + 2 ;

lbl={'r', 'g', 'b', 'c', 'm', 'k', 'y', 'k'} ;

% - parameters of the online classifier
switchSelectionSeeds = 0 ;                      % (0) turn off approximative compression
classifyWithPosterior = 1 ;                     % (1) use posterior info for classification
typeRecDescr = 'dKDE' ;                         % 'dKDE', 'oKDE', switchess between discriminative and standard oKDE
% minNumDataPointsToFormKDE = dim + 1 ;           % (int) minimum number of points before forming a KDE
minNumDataPointsToFormKDE = (dim^2-dim)/2+dim+dim ; % (int) minimum number of points before forming a KDE
react_compression_to_feature_selection = 1 ;    % (1) apply compression in subdimension selected by feature selection
min_samps_per_model_feat_sel = (dim^2-dim)/2+dim+dim ; % (int) minimum number of samples per model observed before applying feature selection
min_th_feat_sel = 0.1 ;                       % (double) threshold on importance below which a feature is removed
costThreshold.thReconstructive = 0.05 ;         % thresholds on reconstructive and discriminative compression
costThreshold.thDiscriminative = 0.05 ;
% - end of parameters
N_init = 5 ;  

% generate test datatestClassifier_dataset
[xc, p_ref ]= generateDots( 1000, 0, nfake_dims, num_add_classes ) ;


opt_score = testClassifications( p_ref, [], 1, xc ) ;
% data generated
 
% create a classifier object with selected parameters
kde_cl = executeOperatorIKDEClsfr( [], 'init', 'compressionClusterThresh', costThreshold,...
                    'typeRecDescr', typeRecDescr, ...
                    'switchSelectionSeeds', switchSelectionSeeds,...
                    'minNumDataPointsToFormKDE', minNumDataPointsToFormKDE, ...
                    'react_compression_to_feature_selection', react_compression_to_feature_selection,...
                    'min_samps_per_model_feat_sel', min_samps_per_model_feat_sel,... 
                    'min_th_feat_sel', min_th_feat_sel ) ;
 

Num_questions = N_init ;
input_data = {} ;
for i_c = 1 : length(xc)
    indat = [] ;
    indat.data = xc{i_c}(:,1:N_init) ; 
%     indat.class_name = num2str(i_c) ;
    indat.class = i_c ;
    input_data = horzcat(input_data, indat) ;
end
kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input' ) ;
  
R_false = {} ; R_true = {} ;
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
 
    
    if test_for_ROC == 1
        for i_c = 1 : length(xc)
            rslt = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data{i_c}.data, ...
                                        'classifyData', 'use_unknown_model',1, 'extensive_answer', 1  ) ;
            if rslt.C ~= i_c && rslt.C ~= 0
                R_true = horzcat(R_true, rslt) ;
            elseif rslt.C ~= i_c && rslt.C == 0
                
            elseif rslt.C == i_c
                R_false = horzcat(R_false, rslt) ;
            end
        end
    end
 
    
    tic
    kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input', ...
                                        'autonomous_update', 'pure_oracle',...
                                        'autoUpdateThres_upper', 0.1  ) ; % 'self_verified', 'oracle_verified', 'pure_oracle'
    toc
    Num_questions = Num_questions + sum(kde_cl.answers) ;
  
    % perform a feature selection 
    if perform_feature_selection == 1
    kde_cl = executeOperatorIKDEClsfr( kde_cl, 'make_simple_feature_selection') ;
    end 
    if isfield(kde_cl, 'debug')
        for_plot = kde_cl.debug ;
    else
        for_plot = [] ;
    end

    % draw distributions
    if plotting_activated == 1
        figure(4) ; clf ;
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

R_results.R_false = R_false ;
R_results.R_true = R_true ;

% save('score.txt','c_score', '-mat') ;

% calculate gains of learning some data-point
% input_data = {} ;
% indat = [] ;
% indat.data = xc{1}(:,500) ; 
% input_data = horzcat(input_data, indat) ;
% rslt = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data{1}.data, 'calculate_gains'  )  ; 
% disp('Gains for learning some datapoint:')
% rslt{1}
% 
% % introspect models % 
% rslt = executeOperatorIKDEClsfr( kde_cl, 'introspect' ) ;
% disp('Introspection confusion matrix:')
% rslt.Con_matrix
% % interpret:
% msg = sprintf('Portion of the first model that is explained by the second model: %f', rslt.Con_matrix(1,2) ) ; disp(msg) ;
% msg = sprintf('Portion of the second model that is explained by the first model: %f', rslt.Con_matrix(2,1) ) ; disp(msg) ;
% 
% % list the number of classes learned so far
% index_list = executeOperatorIKDEClsfr( kde_cl, 'get_class_indexes' ) 
% name_list = executeOperatorIKDEClsfr( kde_cl, 'get_class_names' )
% 
% % change the name of class 2
% kde_cl = executeOperatorIKDEClsfr( kde_cl, 'set_name_at_index', 2, 'cyan' ) ;
% 
% % unlearn class 2 %
% indat = [] ;
% indat.data = xc1(:,i) ; indat.class = 2 ;
% kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', {indat}, 'unlearn_with_input'  ) ;


% ---------------------------------------------------------------------- %
function [ret, database] = get_new_datapoint_from_database( database , pref_class )

% check if all is empty
good_list = [] ;
is_empty = 1 ;
for i = 1 : length(database.index_classes)
   if database.current_pos_classes(i) < length(database.index_classes{i})
       is_empty = 0 ; 
       good_list = [good_list, i] ;
   end       
end

if is_empty == 1
   ret = -1 ; 
   return ;
end
 
if isempty(pref_class)
    id = randperm(length(good_list)) ;
    pref_class = good_list(id(1)) ;    
end
   
if database.current_pos_classes(pref_class) >= length(database.index_classes{pref_class})
    ret = [] ;
    return ;    
end 
database.current_pos_classes(pref_class) = database.current_pos_classes(pref_class) + 1 ;
idx = database.index_classes{pref_class}( database.current_pos_classes(pref_class) ) ;
data = database.learn_base(:, idx) ;
 
ret.data = data(1:size(data,1)-1,:) ;
ret.class = data(size(data,1),:) ;
  
% ----------------------------------------------------------------------- %
function database = make_five_fold_experiment( filname )

F = load('colorDataset\colors.txt') ;

% permute
idx = randperm(size(F,2)) ;  
F = F(:,idx) ;

% get classes
clss = F(size(F,1),:) ;
cls_ids = unique(clss) ;

% cut to folds
delt = round(size(F,2)/5) ;

database.folds = {} ;
for j = 0 : 4
    id = ones(1, size(F,2)) ;
    j_end = min([j*delt+delt,size(F,2)]) ;
    j_str = j*delt+1 ;
    
%     [j_str,j_end]
    id(j_str:j_end) = 0 ;
    
    test_base = F(:,id==0) ;
    learn_base = F(:,id==1) ;
 
    index_classes = {} ;
    for i = 1 : length(cls_ids)
        id_c = find(learn_base(size(learn_base,1),:) == cls_ids(i)) ; 
        index_classes = horzcat(index_classes, {id_c} ) ;  
    end
    current_pos_classes = ones(1, length(cls_ids)) ;
    fold.learn_base = learn_base ;
    
    fold.test_base = test_base ;
    fold.index_classes = index_classes ;
    fold.current_pos_classes = current_pos_classes ;
    database.folds = horzcat(database.folds, fold) ;
end


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
 




 
 