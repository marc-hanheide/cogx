function testClassifier_dataset()
% database = make_five_fold_experiment( [] ) ;

% load database
dbname = 'colorDataset\database_color_five_fold.mat' ;
database = load(dbname) ;
database = database.database ;


% % ROC curves
% R_results.R_false = [] ;
% R_results.R_true = [] ;
% for i = 1 : 10 
%     R_res = demoClassifier() ;
%     R_results.R_false = [R_results.R_false, R_res.R_false] ;
%     R_results.R_true = [R_results.R_true, R_res.R_true] ;
% end
% save('colorDataset\ROC_random_nofsel_newCond2.mat','R_results','-mat') ;
% r = load('colorDataset\ROC_random_nofsel_newCond2.mat') ;
% analyzeUncertainty(r.R_results.R_false, r.R_results.R_true,'H', 1, 'b') ;
 
H = 0.1 ;
H_low = 1e-2 ;
fsel = 0 ;
type = 'self_verified' ; % 'self_verified', 'oracle_verified', 'pure_oracle' 
proactive_mode = 1 ;
 % pure_oracle ne pro, pure_oracle pro, self_verified nopro, 
if isequal(type,'pure_oracle') 
    H = 0 ;
end

out_name = ['colorDataset\random_fsel', num2str(fsel),'_',type,'H', num2str(H),'_proact', num2str(proactive_mode),'.mat'] ;

R_resul = {} ;
for i_repeats = 1 : 10

for i = 1 : length(database.folds) 
   
    R_res = classifier_test( database.folds{i}, type, fsel, proactive_mode, H, H_low ) ;
    
    R_resul = horzcat(R_resul, {R_res}) ;
end

end
R_results.results = R_resul ;
R_results.database = dbname ;
save(out_name,'-mat') ; 

r = load(out_name) ;
% clf ; 
analyzePerformance( r.R_results )

% ---------------------------------------------------------------------- %
function analyzePerformance( R_results )

% average performance
c1 = 'c' ; c2 = 'c--' ; c3 = 'c' ;
% c1 = 'g' ; c2 = 'g' ; c3 = 'g' ;
%  c1 = 'c' ; c2 = 'c' ; c3 = 'c' ;
%  c1 = 'k' ; c2 = 'k' ; c3 = 'k' ;
%  c1 = 'y' ; c2 = c1 ; c3 = c1 ;
% c1 = 'r' ; c2 = 'r' ; c3 = 'r' ;

last_score = {} ;
last_pos_q = {} ;

qqestions = {} ;

sc_w = [] ;
qq_s = [] ;
questions = zeros(2,1000) ;
rcc = [] ;
askng = [] ;
recgn = [] ;
for i = 1 : length(R_results.results)
    score = R_results.results{i}(size(R_results.results{i},1),:) ;        
    ask = R_results.results{i}( 5,:) ;    
    ask(1) = ask(1) + R_results.results{i}( 4, 1 ) ; 
    csa = cumsum(ask) ;
    
    if isempty(recgn)
        recgn = score ;
        askng = csa ;
    else
        recgn = recgn + score ;
        askng = askng + csa ;
    end    
    
    rcc = [ rcc ; score ] ;
    
    id_ask = find(ask ~= 0) ; 
    id_askx = find(ask == 0) ; 
    id_nask = find(ask == 1 ) ; 
    if length(id_ask) + length(id_askx) < length(ask) - 1
        error('Only one question at a time is allowed!') ;
    end

    sc_q = score(id_ask) ;
    as_q = csa(id_ask) ;
    if sum(diff(as_q)==1) - length(as_q)+1 ~= 0
       error('Possibly multiple answers!') ; 
    end
    
    
    
    
    questions(1,as_q) = questions(1,as_q) + sc_q ;
    reg = as_q(length(as_q))+1 : length(questions) ;
    questions(1,reg) = questions(1,reg) + sc_q(length(sc_q)) ;
    questions(2,as_q) = questions(2,as_q) + 1 ;        
    questions(2,reg) = questions(2,reg) + 1 ;
    
%     sc_w=[sc_w,sc_q(length(sc_q))] 
    qq_s = [qq_s, sum(ask)] ;
    
    id = find(ask) ; id = id(length(id)) ;
    last_score = horzcat(last_score, {[score(length(score)), score(id)]}) ;
    last_pos_q = horzcat(last_pos_q, {id}) ;
    
end
askng = askng / length(R_results.results) ;
recgn = recgn / length(R_results.results) ;
N_obs = R_results.results{1}( size(R_results.results{1},1)-2,:) ;
rcg_std = sqrt(var(rcc)) ;

% id_last = find(questions(2,:)>0) ; 
id_last = max(qq_s) ;%id_last(length(id_last)) ;
questions(2,questions(2,:)==0) = 1e-5 ;
questions = questions(1,:)./questions(2,:) ;
questions = questions(1,1:id_last) ;

subplot(1,4,1) ; hold on ;
% plot(N_obs, recgn-rcg_std, c1) ; 
% plot(N_obs, recgn+rcg_std, c1) ;
plot(N_obs, recgn, c2, 'LineWidth', 2) ; 
axis([0,N_obs(length(N_obs)), 0, 1.1]) ; 
xlabel('N_{obs}') ; ylabel('Recognition') ;
box on ;
subplot(1,4,2) ; 
plot(N_obs, askng, c2, 'LineWidth', 2) ;% axis equal ; 
hold on ; 
axis tight ; axis([0, N_obs(length(N_obs)),0,N_obs(length(N_obs))])
xlabel('N_{obs}') ; ylabel('N_{questions}') ; box on ;
subplot(1,4,3) ; 
% plot(1:length(questions), questions, c2, 'LineWidth', 2) ; hold on ;
plot(1:85, questions(1:85), c2, 'LineWidth', 2) ; hold on ;
xlabel('N_{questions}') ; ylabel('Recognition') ;
box on ;
subplot(1,4,4) ; 
plot(N_obs, R_results.results{1}(1,:),[c3,'.'], 'LineWidth', 2) ;
hold on ;
box on ;
% ---------------------------------------------------------------------- %
function R_results = classifier_test( database, type_update, fsel, proactive_mode, autoUpdateThres_upper, autoUpdateThres_lower )


% 'self_verified', 'oracle_verified', 'pure_oracle' type_update
perform_feature_selection = fsel ;
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
react_compression_to_feature_selection = 1 ;    % (1) apply compression in subdimension selected by feature selection
min_samps_per_model_feat_sel = (dim^2-dim)/2+dim+dim ; % (int) minimum number of samples per model observed before applying feature selection
min_th_feat_sel = 0.1 ;                       % (double) threshold on importance below which a feature is removed
costThreshold.thReconstructive = 0.05 ;         % thresholds on reconstructive and discriminative compression
costThreshold.thDiscriminative = 0.05 ;
% - end of parameters
N_init = 5 ;  

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
                    'autoUpdateThres_lower', autoUpdateThres_lower) ;
 

Num_questions = N_init ;
input_data = {} ;
for i_c = 1 : N_init
    cls_pref = [] ;
    if proactive_mode == 1
        cls_pref = i_c ;
        if cls_pref > length(database.index_classes)
           cls_pref = [] ; 
        end
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
 
    tic
    kde_cl = executeOperatorIKDEClsfr( kde_cl, 'input_data', input_data, 'add_input', ...
                                        'autonomous_update', type_update,...
                                        'autoUpdateThres_upper', autoUpdateThres_upper  ) ; % 'self_verified', 'oracle_verified', 'pure_oracle' type_update
    toc
    q_here = sum(kde_cl.answers) ; 
  
    % perform a feature selection 
    if perform_feature_selection == 1
        kde_cl = executeOperatorIKDEClsfr( kde_cl, 'make_simple_feature_selection') ;
    end 
    
    score = evaluateModel( kde_cl, database.test_base ) ;
    
    full_diag = [rsl1;q_here; score] ;
    R_results = [ R_results, full_diag ] ;
    curr_id = curr_id + 1 ;
end


% ---------------------------------------------------------------------- %
function score = evaluateModel( kde_cl, test_base ) 

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
 




 
 