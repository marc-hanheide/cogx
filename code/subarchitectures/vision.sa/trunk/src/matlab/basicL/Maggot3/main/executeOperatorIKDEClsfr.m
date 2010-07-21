function hyper_output_kde_cl = executeOperatorIKDEClsfr( hyper_input_kde_cl, varargin )
% If inputs are data-points
% input_data{i}.class ... with data we need to specify also the class label
% input_data{i}.data  ... actual input data-points

% If inputs are kdes
% input_data{i}.class ... class label
% input_data{i}.kde   ... kde

% by default the classifier construction is discriminative rather than reconstructive


%  hyper_input_kde_cl.kde_cl = classes_kde_cl ;
%  hyper_input_kde_cl.typeRecDescr = 'discriminative' ; % {discriminative, reconstructive}
%  hyper_input_kde_cl.class_labels = class_labels ;
%


type_update = 'partial' ; % 'partial', 'joint'
% useSomeOtherKindOfEstimator = [] ; % empty means default, 'adaptiveMixtures' uses adaptive mixtures
use_equalimportance =  0 ;
classifyWithPosterior = 1 ;
extensive_answer = 0 ;
minNumDataPointsToFormKDE = [] ; 
 
% not operational % minimal_required_examps_mode = [] ; % if 1, then a vector is added only if it's wrongly classified
min_samps_per_model_feat_sel = [] ;
react_compression_to_feature_selection = [] ;
min_th_feat_sel = [] ;
clear_selected_features = 0 ;
sub_selected_features = [] ;
val_set = [] ;
val_get = [] ;
use_unknown_model = [] ;
unknown_model_value = [] ;
create_clsfr_from_this = [] ;
ignoreClasses = [] ;
typeRecDescr = [] ; % type of model: {discriminative, reconstructive}
operator_data = [] ;
input_data = [] ;
vforwvargin = {} ;
args = varargin;
nargs = length(args);
i = 1 ;
while i <= nargs   
    switch args{i}                
        case 'input_data', input_data = args{i+1} ; i = i + 2 ; 
        case 'make_simple_feature_selection', operator_data = args{i} ; i = i + 1 ;   
        case 'init', operator_data = args{i} ; i = i + 1 ;
        case 'get_class_names', operator_data = args{i} ; i = i + 1 ;
        case 'get_class_indexes', operator_data = args{i} ; i = i + 1 ;  
        case 'get_name_at_index', operator_data = args{i} ; val_get = args{i+1} ; i = i + 2 ;            
        case 'get_index_at_name', operator_data = args{i} ; val_get = args{i+1} ; i = i + 2 ; 
        case 'set_name_at_index', operator_data = args{i} ; val_get = args{i+1} ; val_set = args{i+2} ; i = i + 3 ; 
        case 'add_input', operator_data = args{i} ; i = i + 1 ; 
        case 'introspect', operator_data = args{i} ; i = i + 1 ;     
        case 'kdes_to_classifier', operator_data = args{i} ; i = i + 1 ;    
        case 'compress_pdf', operator_data = args{i} ; i = i + 1 ;
        case 'classifyData', operator_data = args{i} ; i = i + 1 ; 
        case 'predict_missing', operator_data = args{i} ; i = i + 1 ; 
        case 'calculate_gains', operator_data = args{i} ; i = i + 1 ;
        case 'unlearn_with_input', operator_data = args{i} ; i = i + 1 ;
        case 'showKDE_of_class_index', operator_data = 'showKDE_of_class_index' ; val_get = args{i+1} ; i = i + 2 ;
        case 'min_samps_per_model_feat_sel', min_samps_per_model_feat_sel = args{i+1} ; i = i + 2 ;
        case 'sub_selected_features', sub_selected_features = args{i+1} ; i = i + 2 ;
        case 'clear_selected_features', clear_selected_features = 1 ; i = i + 1 ;
        case 'min_th_feat_sel', min_th_feat_sel = args{i+1} ; i = i + 2 ;
        case 'react_compression_to_feature_selection', react_compression_to_feature_selection = args{i+1} ; i = i + 2 ;
        case 'typeRecDescr', typeRecDescr = args{i+1} ; i = i + 2 ;   
        case 'ignoreClasses', ignoreClasses = args{i+1} ; i = i + 2 ; 
        case 'classifyWithPosterior', classifyWithPosterior = args{i+1} ; i = i + 2 ; 
        case 'use_equalimportance', use_equalimportance = args{i+1} ; i = i + 2 ; 
        case 'type_update', type_update = args{i+1} ; i = i + 2 ;     
        case 'unknown_model_value', unknown_model_value = args{i+1} ; i = i + 2 ; 
        case 'use_unknown_model', use_unknown_model = args{i+1} ; i = i + 2 ; 
        case 'minNumDataPointsToFormKDE', minNumDataPointsToFormKDE = args{i+1} ; i = i + 2 ; 
%         case 'minimal_required_examps_mode', minimal_required_examps_mode
%         = args{i+1} ; i = i + 2 ;  % not operational
        case 'extensive_answer', extensive_answer = 1 ; i = i + 1 ;         
        case 'useSomeOtherKindOfEstimator', 
            if ~isequal(args{i+1},0)
                vforwvargin = horzcat(vforwvargin, args{i} ) ;
                vforwvargin = horzcat(vforwvargin, args{i+1} ) ;                 
            end
            i = i + 2 ; 
        case 'compressionClusterThresh', 
            vforwvargin = horzcat(vforwvargin, args{i} ) ;
            vforwvargin = horzcat(vforwvargin, args{i+1} ) ;
            i = i + 2 ; 
        otherwise
             vforwvargin = horzcat(vforwvargin, args{i} ) ;
             i = i + 1 ; 
    end
end

% initialize parameters if required
if isempty(hyper_input_kde_cl)
    hyper_input_kde_cl.kde_cl = {} ;
    hyper_input_kde_cl.typeRecDescr = 'discriminative' ; % {discriminative, reconstructive}
    hyper_input_kde_cl.class_labels = [] ;
    hyper_input_kde_cl.class_labels_names = {} ;
    hyper_input_kde_cl.unknown_model_value = 0 ;
    hyper_input_kde_cl.use_unknown_model = 0 ;
    hyper_input_kde_cl.Params.minNumDataPointsToFormKDE = 2 ;
    hyper_input_kde_cl.sub_selected_features = [] ;
    hyper_input_kde_cl.min_th_feat_sel = 0.1 ;
    hyper_input_kde_cl.min_samps_per_model_feat_sel = 5 ;
    hyper_input_kde_cl.react_compression_to_feature_selection = 0 ;
    hyper_input_kde_cl.sub_feature_sel_forgetting = 0 ;
    hyper_input_kde_cl.cummulative_feat_costs = [] ;
%     hyper_input_kde_cl.minimal_required_examps_mode = 0 ;
end

% if ~isempty(minimal_required_examps_mode)
%     hyper_input_kde_cl.minimal_required_examps_mode  = minimal_required_examps_mode ;
% end
 
if ~isempty(min_samps_per_model_feat_sel)
    hyper_input_kde_cl.min_samps_per_model_feat_sel  = min_samps_per_model_feat_sel ;
end

if ~isempty(react_compression_to_feature_selection)
    hyper_input_kde_cl.react_compression_to_feature_selection  = react_compression_to_feature_selection ;
end

if ~isempty(min_th_feat_sel)
    hyper_input_kde_cl.min_th_feat_sel  = min_th_feat_sel ;
end
 
if clear_selected_features ~= 0
    hyper_input_kde_cl.sub_selected_features = [] ;
end

if ~isempty(sub_selected_features)
   hyper_input_kde_cl.sub_selected_features = sub_selected_features ; 
end

if ~isempty(minNumDataPointsToFormKDE)
    hyper_input_kde_cl.Params.minNumDataPointsToFormKDE = minNumDataPointsToFormKDE ;
end

if ~isempty(typeRecDescr)
    hyper_input_kde_cl.typeRecDescr = typeRecDescr ;
end

if ~isempty(unknown_model_value)
    hyper_input_kde_cl.unknown_model_value = unknown_model_value ;
end

if ~isempty(use_unknown_model)
   hyper_input_kde_cl.use_unknown_model = use_unknown_model  ;
end

switch operator_data    
    case 'init'
        hyper_output_kde_cl = hyper_input_kde_cl ;
    case 'get_name_at_index'
        hyper_output_kde_cl = [] ;
        if isempty(val_get)
            return ;
        end
        
        if val_get == -1 
            hyper_output_kde_cl = 'unknown' ;
            return ;
        end
        
        if val_get <= length(hyper_input_kde_cl.class_labels)
            hyper_output_kde_cl = hyper_input_kde_cl.class_labels_names{val_get} ;
        else
            return ;
        end
    case 'get_index_at_name' 
        hyper_output_kde_cl = [] ;
        if isempty(val_get)
            return ;
        end
        
        for i = 1 : length(hyper_input_kde_cl.class_labels_names)
           if isequal(val_get, hyper_input_kde_cl.class_labels_names{i}) 
               hyper_output_kde_cl = i ;
               return ;
           end
        end
        return ;
    case 'set_name_at_index'
        hyper_output_kde_cl = hyper_input_kde_cl ;
        if isempty(val_set) || val_get > length(hyper_output_kde_cl.class_labels)
            return ;
        end
        hyper_output_kde_cl.class_labels_names{val_get} = val_set;
        return ;        
    case 'get_class_indexes'
        hyper_output_kde_cl = hyper_input_kde_cl.class_labels ;
        return ;
    case 'get_class_names'
        hyper_output_kde_cl = hyper_input_kde_cl.class_labels_names ;
        return ;    
    case 'kdes_to_classifier'   
        class_labels = [] ;
        classes_kde_cl = {} ;
        class_labels_names = {} ;
        for i = 1 : length(input_data)
            
            % initialize cumulative feature selector
            if isempty(hyper_input_kde_cl.cummulative_feat_costs)
               hyper_input_kde_cl.cummulative_feat_costs = zeros(1,size(input_data{i}.kde.pdf.Mu,1)) ;  
            end
            
            classes_kde_cl = horzcat(classes_kde_cl, input_data{i}.kde) ;
            class_labels = horzcat(class_labels, input_data{i}.class) ;
            class_labels_names = horzcat(class_labels_names, num2str(input_data{i}.class)) ;
        end        
        hyper_input_kde_cl.kde_cl = classes_kde_cl ;
        hyper_input_kde_cl.typeRecDescr = 'discriminative' ; % {discriminative, reconstructive}
        hyper_input_kde_cl.class_labels = class_labels ;
        
        % recalculate the unknown model value!!
        hyper_input_kde_cl.unknown_model_value = prob_of_unknown( hyper_input_kde_cl.kde_cl ) ;
        hyper_output_kde_cl = hyper_input_kde_cl ;
    case 'make_simple_feature_selection'
        % check if there is enough samples per model
        for i = 1 : length(hyper_input_kde_cl.kde_cl)
           if  hyper_input_kde_cl.kde_cl{i}.ikdeParams.N_eff < hyper_input_kde_cl.min_samps_per_model_feat_sel
               hyper_output_kde_cl = hyper_input_kde_cl ;
               return ;
           end
        end
 
        d_before = hyper_input_kde_cl.sub_selected_features ;
        fast_search = 1 ;
        % prepare distributions  
        pdf_classes.N_data = 0 ;
        pdf_classes.F_giv_cj = {} ;
        pdf_classes.cj = [] ;
        for i = 1 : length(hyper_input_kde_cl.kde_cl)
%             [new_mu, new_Cov, w_out] = momentMatchPdf(hyper_input_kde_cl.kde_cl{i}.pdf.Mu, hyper_input_kde_cl.kde_cl{i}.pdf.Cov, hyper_input_kde_cl.kde_cl{i}.pdf.w) ;
%             p.Mu = new_mu ;
%             p.Cov = {new_Cov} ;
%             p.w = 1 ;
           % get kde from sublayer
            pdf_tmp = extractMixtureFromSublayers( hyper_input_kde_cl.kde_cl{i}.pdf ) ;

            p.Mu = pdf_tmp.Mu ; % hyper_input_kde_cl.kde_cl{i}.pdf.Mu ;
            p.Cov = pdf_tmp.Cov ; % hyper_input_kde_cl.kde_cl{i}.pdf.Cov ;
            for i_cv = 1 : length(p.Cov)
               p.Cov{i_cv} = regularizeCovariance( p.Cov{i_cv} )  ;
            end
            p.w = pdf_tmp.w ; %hyper_input_kde_cl.kde_cl{i}.pdf.w ;
            pdf_classes.F_giv_cj = horzcat(pdf_classes.F_giv_cj, p) ;
            pdf_classes.N_data = pdf_classes.N_data + hyper_input_kde_cl.kde_cl{i}.ikdeParams.N_eff ;
            pdf_classes.cj = [pdf_classes.cj, hyper_input_kde_cl.kde_cl{i}.ikdeParams.N_eff] ;
        end
        pdf_classes.cj = pdf_classes.cj / sum(pdf_classes.cj) ;
  
        % select features                
        [ Cost, f_sel ] = mixtureFeatureSelection( pdf_classes, fast_search ) ;        
        rslt = Cost(3:size(Cost,1),:) ;
        indic = rslt(2,:) > hyper_input_kde_cl.min_th_feat_sel ;
        frst = 1 ; 
        for i = length(indic) :-1: 1
           if indic(i) == 0 
               frst = i + 1;
               break ;
           end
        end
        indic = indic*0 ;
        indic(frst:length(indic)) = 1 ;
        sel_feats = sort(rslt(1, find(indic))) ;
        hyper_input_kde_cl.sub_selected_features = sel_feats ;      
        hyper_output_kde_cl = hyper_input_kde_cl ;
        
        hyper_output_kde_cl.debug.Cost = Cost ;
        
        [ix, xsrt] = sort(rslt(1,:)) ;
        cumad = rslt(2,:).*indic ;
        hyper_output_kde_cl.cummulative_feat_costs = hyper_output_kde_cl.cummulative_feat_costs + cumad(xsrt) ; 
        
        d_b = sort(d_before)  ;
        d_n = sort(sel_feats)  ;
        if isequal(d_b, d_n) || isempty(d_before)
            
        else            
            if ~isequal(d_b, d_n) %length(d_b) < length(d_n)
                hyper_output_kde_cl.sub_feature_sel_forgetting = 20*length(hyper_input_kde_cl.kde_cl) ;
            else
                hyper_output_kde_cl.sub_feature_sel_forgetting = 0 ;
            end
        end
         
    case 'add_input'
        % determine if the class already exists, and initialize it if it
        % does not exist

        for i = 1 : length(input_data)            
            % check if the label already exists and check for errors
            [data, class, class_name, class_exists] = parseClassData(hyper_input_kde_cl, input_data{i} ) ;
            
            % initialize cumulative feature selector
            if isempty(hyper_input_kde_cl.cummulative_feat_costs)
               hyper_input_kde_cl.cummulative_feat_costs = zeros(1,size(data,1)) ;  
            end
            
            % if attenuation should be triggered
            if hyper_input_kde_cl.sub_feature_sel_forgetting > 0
                hyper_input_kde_cl.sub_feature_sel_forgetting = hyper_input_kde_cl.sub_feature_sel_forgetting - 1 ;
                kde_w_attenuation = 1 - 1/10  ;
            else
                kde_w_attenuation = 1  ;
            end
            kde_w_attenuation = 1 ;
 
            if class_exists == 0                                        
                % initialize new class                 
                kde = executeOperatorIKDE( [], 'input_data', input_data{i}.data, 'add_input', vforwvargin{:} ) ; 
 
                hyper_input_kde_cl.kde_cl = horzcat(hyper_input_kde_cl.kde_cl, kde) ;                
                hyper_input_kde_cl.class_labels = horzcat(hyper_input_kde_cl.class_labels, class) ;
                hyper_input_kde_cl.class_labels_names = horzcat(hyper_input_kde_cl.class_labels_names, class_name) ;
                hyper_input_kde_cl.unknown_model_value = prob_of_unknown( hyper_input_kde_cl.kde_cl ) ;
            else
                % update existing class
            
% %                 % if minimal_required_examps_mode is on, then first check
% %                 % for classification
% %                 if hyper_input_kde_cl.minimal_required_examps_mode == 1
% %                     reslt = executeOperatorIKDEClsfr( hyper_input_kde_cl, 'input_data', data, 'classifyData', 'use_unknown_model', 1 ) ;                     
% %                     if reslt.C == class
% %                        % correct classification, so continue  
% %                        hyper_input_kde_cl.kde_cl{class}.ikdeParams.N_eff = hyper_input_kde_cl.kde_cl{class}.ikdeParams.N_eff + 1;
% %                        hyper_input_kde_cl.kde_cl{class} = executeOperatorIKDE( hyper_input_kde_cl.kde_cl{class}, 'recalculate_bandwidth', ...
% %                                                                                 'input_data', [], 'add_input') ;                       
% %                        continue ; 
% %                     end                    
% %                 end
                
                
                if isequal(hyper_input_kde_cl.typeRecDescr,'dKDE')
                    % make negative data model
                    otherClasses = makeOtherClasses( hyper_input_kde_cl.kde_cl, class, size(data,2), use_equalimportance ) ; 
                elseif isequal(hyper_input_kde_cl.typeRecDescr,'oKDE')
                    otherClasses = {} ;
                elseif isequal(hyper_input_kde_cl.typeRecDescr,'AM')
                    otherClasses = {} ;
                elseif isequal(hyper_input_kde_cl.typeRecDescr,'dAM')
                    otherClasses = makeOtherClasses( hyper_input_kde_cl.kde_cl, class, size(data,2), use_equalimportance ) ; %input_kde_cl  output_kde_cl
                else
                    error('Unknown update rule! Either reconstructive or discriminative !') ;
                end
                
                sub_feats = [] ;
                if hyper_input_kde_cl.react_compression_to_feature_selection == 1 
                    sub_feats = hyper_input_kde_cl.sub_selected_features ;
                end
 
                % update the kde
                hyper_input_kde_cl.kde_cl{class} = ...
                                  executeOperatorIKDE( hyper_input_kde_cl.kde_cl{class}, 'input_data', ...
                                                       data, 'add_input', 'otherClasses', otherClasses,...
                                                       'selectSubDimensions', sub_feats,...
                                                       'kde_w_attenuation', kde_w_attenuation, ...
                                                       vforwvargin{:} ) ;                               
            end            
        end

        % search for degenerate kdes and reapproximate their bandwidths
        for i = 1 : length(hyper_input_kde_cl.kde_cl)
           if hyper_input_kde_cl.kde_cl{i}.ikdeParams.N_eff < hyper_input_kde_cl.Params.minNumDataPointsToFormKDE                  
                    otherClasses = makeOtherClasses( hyper_input_kde_cl.kde_cl, i, 0, use_equalimportance ) ; 
                    hyper_input_kde_cl.kde_cl{i} = ...
                                     executeOperatorIKDE( hyper_input_kde_cl.kde_cl{i}, 'set_auxiliary_bandwidth' ,...
                                     'otherClasses', otherClasses) ;
           end
            
        end
        
        hyper_output_kde_cl = hyper_input_kde_cl ;   
                        
%         if isempty(hyper_input_kde_cl.kde_cl)
%             output_kde_cl = {} ; 
%             for i = 1 : length(input_data)
%                 %         class = input_data{i}.class ;
%                 kde = executeOperatorIKDE( [], 'input_data', input_data{i}.data, 'add_input', vforwvargin{:}  ) ;
% 
%                 output_kde_cl = horzcat(output_kde_cl, kde) ;
%                 % store class labels
%                 hyper_input_kde_cl.class_labels = horzcat(hyper_input_kde_cl.class_labels, input_data{i}.class) ; 
%             end    
%             hyper_output_kde_cl = hyper_input_kde_cl ;  
%             hyper_output_kde_cl.kde_cl = output_kde_cl ;
%             hyper_output_kde_cl.unknown_model_value = prob_of_unknown( hyper_output_kde_cl.kde_cl ) ;
%             return ;
%         else
%             % store previous
%             hyper_output_kde_cl = hyper_input_kde_cl ;
%             input_kde_cl = hyper_input_kde_cl.kde_cl ;
%             output_kde_cl = input_kde_cl ;
%                      
%                 % add input to kdes
%                 for i = 1 : length(input_data)
%                     class = input_data{i}.class ;
%                     data = input_data{i}.data ;
%                     
%                     if isempty(data)
%                         continue ;
%                     end
%                     
%                     if isequal(hyper_input_kde_cl.typeRecDescr,'dKDE')
%                         % make negative data model
%                         otherClasses = makeOtherClasses( input_kde_cl, class, size(data,2), use_equalimportance ) ; %input_kde_cl  output_kde_cl
%                     elseif isequal(hyper_input_kde_cl.typeRecDescr,'oKDE')
%                         otherClasses = {} ;
%                     elseif isequal(hyper_input_kde_cl.typeRecDescr,'AM')
%                         otherClasses = {} ;    
%                     elseif isequal(hyper_input_kde_cl.typeRecDescr,'dAM')
%                         otherClasses = makeOtherClasses( input_kde_cl, class, size(data,2), use_equalimportance ) ; %input_kde_cl  output_kde_cl
%                     else
%                         error('Unknown update rule! Either reconstructive or discriminative !') ;
%                     end
%                     
%                     % update the kde
%                     kde = executeOperatorIKDE( input_kde_cl{class}, 'input_data', ...
%                         data, 'add_input',...
%                         'otherClasses', otherClasses, vforwvargin{:} ) ;
%                     output_kde_cl{class} = kde ;
%                 end
%             hyper_output_kde_cl.kde_cl = output_kde_cl ;
%         end
    case 'unlearn_with_input'         
        for i = 1 : length(input_data)
            class = input_data{i}.class ;
            data = input_data{i}.data ;     
            
            % create negative classes
            otherClasses = {} ;
            if isequal(hyper_input_kde_cl.typeRecDescr,'dKDE')
                % make negative data model
                otherClasses = makeOtherClasses( hyper_input_kde_cl.kde_cl,...
                               class, size(data,2), use_equalimportance ) ; %input_kde_cl              
            end

            % unlearn the kde
            hyper_input_kde_cl.kde_cl{class} =...
                        executeOperatorIKDE( hyper_input_kde_cl.kde_cl{class}, 'input_data', ...
                        data, 'unlearn_with_input', 'otherClasses', otherClasses, vforwvargin{:} ) ;
%             hyper_input_kde_cl.kde_cl{class}.pdf.smod.useVbw        
        end        
        hyper_output_kde_cl = hyper_input_kde_cl ; 
    case 'compress_pdf'
        % store previous
        input_kde_cl = hyper_input_kde_cl.kde_cl ;
        output_kde_cl = input_kde_cl ;
        for class = 1 : length(input_kde_cl)
            otherClasses = {} ;
            if isequal(hyper_input_kde_cl.typeRecDescr,'discriminative')
                % make negative data model
                otherClasses = makeOtherClasses( output_kde_cl, class ) ; %input_kde_cl
            end
            % update the kde
            kde = executeOperatorIKDE( input_kde_cl{class}, 'compress_pdf',...
                                       'otherClasses', otherClasses, vforwvargin{:} ) ;
            output_kde_cl{class} = kde ;
        end
        hyper_output_kde_cl.kde_cl = output_kde_cl ;
    case 'classifyData'
        input_kde_cl = hyper_input_kde_cl.kde_cl ;
        P = zeros(length(input_kde_cl),size(input_data,2)) ;
        p_pr = zeros(1,length(input_kde_cl)) ;
        for i = 1 : length(input_kde_cl)
            if sum(i == ignoreClasses) > 0
               continue ; 
            end
            p_pr(i) = input_kde_cl{i}.ikdeParams.N_eff ;
            
            sub_feats = [] ;
            if hyper_input_kde_cl.react_compression_to_feature_selection == 1
                sub_feats = hyper_input_kde_cl.sub_selected_features ;
            end
            
            
            if isequal(hyper_input_kde_cl.typeRecDescr,'AM') || isequal(hyper_input_kde_cl.typeRecDescr,'dAM')
                pdf_t = input_kde_cl{i}.pdf ;
%                 for ik = 1 : length(pdf_t.Cov)
%                     pdf_t.Cov{ik} = pdf_t.Cov{ik}+1e-2 ;                           
%                 end
                 reslt.evalpdf = evaluatePointsUnderPdf(pdf_t, input_data) ;        
            else            
                reslt = executeOperatorIKDE( input_kde_cl{i}, 'input_data',  input_data, 'evalPdfOnData',...
                                             'selectSubDimensions', sub_feats) ;                
            end
            P(i,:) = reslt.evalpdf ; %p ;
        end
        p_pr = p_pr / sum(p_pr) ;
        if classifyWithPosterior == 1
            for i = 1 : length(input_kde_cl)
                P(i,:) = P(i,:) * p_pr(i) ;
            end            
        end
    
        
        hyper_output_kde_cl = [] ;
        hyper_output_kde_cl.P = P ;
  
        % if we are using the unknown model then the set of classes is
        % augmented by an "unknown label" -1 !
        class_labels = hyper_input_kde_cl.class_labels ;
        if hyper_input_kde_cl.use_unknown_model == 1
            hyper_input_kde_cl.unknown_model_value = prob_of_unknown( hyper_input_kde_cl.kde_cl ) ;
            
            Rc = hyper_input_kde_cl.unknown_model_value*ones(1,size(P,2)) ;%   repmat(hyper_input_kde_cl.unknown_model_value, size(P,1), 1) ;
            for i = 1 : size(P,1)
               P(i,:) = P(i,:).*(1-Rc) ; 
            end
            
            P = [P; Rc] ;
            class_labels = horzcat(hyper_input_kde_cl.class_labels, -1) ;
        end
        
        sP = sum(P) ;
        for i = 1 : size(P,1)
            P(i,:) = P(i,:) ./ sP ;
        end
        
        hyper_output_kde_cl.P = P ;
        [vals, i_max] = max(P) ;        
        hyper_output_kde_cl.C = class_labels(i_max) ;
        
        if extensive_answer == 1
           % calculate and output additional statistics
           % 1. get entropy
           H = 2*(1-sum(P.*P)) ;
           
           % 2. get R1 and R2 score (likelihood ratio)
           srtP = sort(P,1, 'descend') ;
           R1 = srtP(1,:)./ (sum(srtP(2:size(srtP,1),:)) + 1e-30) ;
           R2 = srtP(1,:)./ (srtP(2,:)+1e-30)  ;
           
           hyper_output_kde_cl.H = H ;
           hyper_output_kde_cl.R1 = R1 ;
           hyper_output_kde_cl.R2 = R2 ;
        end
    case 'calculate_gains'
        % calculate gains for all input data-points
        hyper_output_kde_cl = {} ;
        for i = 1 : size(input_data,2)        
            % classify data_point
            rslt = executeOperatorIKDEClsfr( hyper_input_kde_cl, 'input_data', input_data(:,i), ...
                                             'classifyData', 'use_unknown_model', 1,...
                                             'extensive_answer', 1 ) ;
            
            I1 = rslt.H ; % .R1, .R2
            P1 = rslt.P ;
            C1 = rslt.C(1) ; 
            
            I2 = 0 ; I3 = 0 ;
            % after first update
            for j = 1 : length(hyper_input_kde_cl.kde_cl)              
                dat.class = j ;
                dat.data = input_data(:,i) ;           
                kde_cl_tmp = executeOperatorIKDEClsfr( hyper_input_kde_cl, 'input_data', {dat}, 'add_input', 'force_prevent_compression', 1 ) ;
                
                rslt = executeOperatorIKDEClsfr( kde_cl_tmp, 'input_data', input_data(:,i), ...
                                             'classifyData', 'use_unknown_model', 1,...
                                             'extensive_answer', 1 ) ;
                dI2 = rslt.H ;
                I2 = I2 + dI2*P1(j) ;
                P2 = rslt.P ;
                
                dI3 = 0 ;
                for k = 1 : length(hyper_input_kde_cl.kde_cl)
                    dat.class = k ;
                    dat.data = input_data(:,i) ;           
                    kde_cl_tmp2 = executeOperatorIKDEClsfr( kde_cl_tmp, 'input_data', {dat}, 'add_input', 'force_prevent_compression', 1 ) ;
                    rslt = executeOperatorIKDEClsfr( kde_cl_tmp2, 'input_data', input_data(:,i), ...
                                                    'classifyData', 'use_unknown_model', 1,...
                                                    'extensive_answer', 1 ) ;
                    ddI3 = rslt.H ;
                    dI3 = dI3 + ddI3*P2(k) ;
                end
                dI3 = dI3 / sum(P2(1:length(P2)-1)) ;
                I3 = I3 + dI3*P1(j) ;
            end
            I2 = I2 / sum(P1(1:length(P1)-1)) ; 
            I3 = I3 / sum(P1(1:length(P1)-1)) ; 
            
            rslt = [] ;
            rslt.I1 = I1 ;
            rslt.I2 = I2 ;
            rslt.I3 = I3 ;
            rslt.Gain1 = I1 - I2 ;
            rslt.Gain2 = I2 - I3 ;
            rslt.dGain = rslt.Gain2 - rslt.Gain1 ;
            rslt.C = C1 ;
            hyper_output_kde_cl = horzcat(hyper_output_kde_cl, rslt) ;
        end 
        
    case 'predict_missing'
        input_kde_cl = hyper_input_kde_cl.kde_cl ;
        hyper_output_kde_cl = {} ;
        for i = 1 : length(input_kde_cl)
            reslt = executeOperatorIKDE( input_kde_cl{i}, 'input_data', input_data, 'predictMissingVals', vforwvargin{:}  ) ;
            hyper_output_kde_cl = horzcat(hyper_output_kde_cl, reslt) ;
        end   
    case 'introspect'
        % generate samples from each model, classify them and compute the
        % confusion matrix Con_matrix
        N_samps = 100 ; 
        idx_unknown = length(hyper_input_kde_cl.kde_cl) + 1 ;
        Con_matrix = zeros(length(hyper_input_kde_cl.kde_cl), idx_unknown) ;
        for i = 1 : length(hyper_input_kde_cl.kde_cl)
            x_tmp = sampleGaussianMixture( hyper_input_kde_cl.kde_cl{i}.pdf, N_samps ) ;
            rslt = executeOperatorIKDEClsfr( hyper_input_kde_cl, 'input_data', x_tmp, 'classifyData', 'use_unknown_model', 1 ) ;%'use_unknown_model', 0
            for j = 1 : length(hyper_input_kde_cl.kde_cl)                               
               Con_matrix(i,j) = sum(rslt.C == j)/N_samps ;                
            end  
            Con_matrix(i,idx_unknown) = sum(rslt.C == -1)/N_samps ;
        end
        hyper_output_kde_cl.N_samps = N_samps ;
        hyper_output_kde_cl.Con_matrix = Con_matrix ;
    case 'showKDE_of_class_index'        
        if val_get > length(hyper_input_kde_cl.class_labels)
            return ;
        end        
      
        sub_feats = [] ;
        if hyper_input_kde_cl.react_compression_to_feature_selection == 1
            sub_feats = hyper_input_kde_cl.sub_selected_features ;
        end
        
        executeOperatorIKDE( hyper_input_kde_cl.kde_cl{val_get}, 'showKDE', 'selectSubDimensions', sub_feats,vforwvargin{:} ) ;
        hyper_output_kde_cl = hyper_input_kde_cl.class_labels_names(val_get) ;
        title(hyper_output_kde_cl) ;        
end

% -------------------------------------------------------------------- %
function otherClasses = makeOtherClasses( input_kde_cl, i_exclude, newadds, use_equalimportance )

otherClasses.pdfs = {} ; 
otherClasses.N_eff = [] ;
w_other = ones(1,length(input_kde_cl)-1) ;  
w_other = w_other / sum(w_other) ;
% otherClasses.priors = []; %(length(input_kde_cl)-1) / length(input_kde_cl) ;  
otherClasses.inner_priors = [] ; %w_other ;
for i = 1 : length(input_kde_cl) 
    if i ~= i_exclude
        otherClasses.inner_priors = horzcat(otherClasses.inner_priors, input_kde_cl{i}.ikdeParams.N_eff ) ;
        otherClasses.pdfs = horzcat(otherClasses.pdfs, input_kde_cl{i}.pdf) ;
        otherClasses.N_eff = horzcat(otherClasses.N_eff, input_kde_cl{i}.ikdeParams.N_eff ) ;
    end
end

otherClasses.inner_priors = otherClasses.inner_priors / sum([otherClasses.inner_priors,input_kde_cl{i_exclude}.ikdeParams.N_eff + newadds ]) ;
 
otherClasses.priors = sum(otherClasses.inner_priors) ;
if use_equalimportance == 1 
    otherClasses.inner_priors = w_other ;
    otherClasses.priors = (length(input_kde_cl)-1) / length(input_kde_cl) ;     
end
if isempty(otherClasses.pdfs)
    otherClasses = {}  ;
end

% ------------------------------------------------------------------ %
function P = prob_of_unknown( classes_kde_cl )

N = [] ;
for i = 1 : length(classes_kde_cl)
   N = [N , classes_kde_cl{1}.ikdeParams.N_eff ] ;     
end
n = sum(N) ;
p = N/n ;

csr = n/length(classes_kde_cl) ;%P = sum(((1-p).^n).*p) ;
P = sum(((1-p).^n).*p)+0.5* +0.5*exp(-csr/10) ;
P = 1e-4 ; %max([P,1e-3]) ;

% ----------------------------------------------------------------- %
function [data, class, class_name, class_exists] = parseClassData(hyper_input_kde_cl, input_data )

class_exists = 0 ;
class = [] ;
class_name = [] ;
if isfield(input_data,'class')
    class = input_data.class ;
end
if isfield(input_data,'class_name')
    class_name = input_data.class_name ;
end
data = input_data.data ;

% continue if the data is empty
if isempty(data)
    return ;
end

% check if the label already exists and check for errors
if ~isempty(class) && class <= length(hyper_input_kde_cl.class_labels )
    class_exists = 1 ;
    if ~isempty(class_name) && ~isequal(class_name, hyper_input_kde_cl.class_labels_names{class})
        error('Error in naming the class! The consecutive number does not match the naming!') ;
    end
    class_name = hyper_input_kde_cl.class_labels_names{class} ;
end

if isempty(class)
    if ~isempty(class_name)
        for i_cls = 1 : length(hyper_input_kde_cl.class_labels_names)
            if isequal(class_name, hyper_input_kde_cl.class_labels_names{i_cls})
                class_exists = 1 ;
                class = i_cls ;
                break ;
            end
        end
    else
        error('At least one of the variables "class" or "class_name" should be specified!') ;
    end
end
% classes should be identified now

if isempty(class_name)   
    class_name = num2str(class)  ;
end
 
if isempty(class)   
    class = length(hyper_input_kde_cl.class_labels ) + 1  ;
end
 