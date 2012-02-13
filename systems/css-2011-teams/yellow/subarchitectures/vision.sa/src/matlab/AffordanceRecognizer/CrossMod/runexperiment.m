% Data = loadsim1data('noise', true);
loadaffordancedata;
% loadirisdata;
% loadionospheredata;
% loadwinedata;
% loadwinequalityred;
% loadpimadata;
% Data = castasregression(Data);

% 'save', './results/results.mat',...

Experiment = LOOCVEvaluator('data', Data,...                            
                            'trials', 1,...
                            'epochs', 2,...
                            'test_interval_timesteps', Inf,...
                            'k', 10,...
                            'debug', 2);
                        
Experiment = Experiment.addLearner('name', 'SOM to HeurFORLVQ, Hellinger Distance',...
                                   'type', 'bimodal',...
                                   'normalization', 'all',...
                                   'normalization_method', 'range',... % var, range, log, logistic, histD, histC, eval
                                   'modality_types', {'codebook', 'codebook'},...
                                   'codebook_sizes', {[4 4], [4 4]},...
                                   'codebook_neighs', {'bubble', 'bubble'},...
                                   'codebook_lattices', {'hexa', 'hexa'},...
                                   'codebook_init_method', {'mean', 'mean'},... % rand, sample, mean
                                   'phase_shifts', {{0.5}, {NaN}},...
                                   'updaters', {{'SOM', 'HeurFORLVQ'}, {'SOM'}},...
                                   'alpha_types', {{'linear', 'constant'}, {'linear'}},...
                                   'alpha_inits', {{1, 0.1}, {1}},...
                                   'radius_types', {{'linear', 'linear'}, {'linear'}},...
                                   'radius_inits', {{5, 5}, {5}},...
                                   'radius_fins', {{1, 1}, {1}},...
                                   'window_sizes', {{NaN, 0.1}, {NaN}},...
                                   'alpha_feature_types', {{NaN, 'constant'}, {NaN}},...
                                   'alpha_feature_inits', {{NaN, 0.3}, {NaN}},...
                                   'auxdist_type', 'hellinger',...
                                   'feature_selection', 'nodestats_hard',...
                                   'feature_selection_feedback', false,...
                                   'metric', {'sumsquared', 'sumsquared'},...
                                   'classification_method', 'node');
                               
% Experiment = Experiment.addLearner('name', 'SOM to HeurFORLVQ, Hellinger Distance',...
%                                    'type', 'bimodal',...
%                                    'modality_types', {'codebook', 'codebook'},...
%                                    'codebook_sizes', {[4 4 4], [4 4 4]},...
%                                    'codebook_neighs', {'bubble', 'bubble'},...
%                                    'codebook_lattices', {'rect', 'rect'},...
%                                    'codebook_init_method', {'sample', 'sample'},...
%                                    'phase_shifts', {{0.5}, {NaN}},...
%                                    'updaters', {{'SOM', 'HeurFORLVQ'}, {'SOM'}},...
%                                    'alpha_types', {{'linear', 'constant'}, {'linear'}},...
%                                    'alpha_inits', {{1, 0.1}, {1}},...
%                                    'radius_types', {{'linear', 'linear'}, {'linear'}},...
%                                    'radius_inits', {{5, 5}, {5}},...
%                                    'radius_fins', {{1, 1}, {1}},...
%                                    'window_sizes', {{NaN, 0.1}, {NaN}},...
%                                    'alpha_feature_types', {{NaN, 'constant'}, {NaN}},...
%                                    'alpha_feature_inits', {{NaN, 0.3}, {NaN}},...
%                                    'auxdist_type', 'hellinger',...
%                                    'feature_selection', 'lda',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared', 'sumsquared'},...
%                                    'classification_method', 'node');
                            
% Experiment = Experiment.addLearner('name', 'SOM to HeurFLDAOLVQ3, Hellinger Distance',...
%                                    'type', 'bimodal',...
%                                    'normalization', 'all',...
%                                    'normalization_method', 'range',... % var, range, log, logistic, histD, histC, eval                                   
%                                    'modality_types', {'codebook', 'codebook'},...
%                                    'codebook_sizes', {[4 4], [4 4]},...
%                                    'codebook_neighs', {'bubble', 'bubble'},...
%                                    'codebook_lattices', {'hexa', 'hexa'},...
%                                    'codebook_init_method', {'sample', 'sample'},... % rand, sample, mean
%                                    'phase_shifts', {{0.25}, {NaN}},...
%                                    'updaters', {{'SOM', 'HeurFLDAOLVQ3'}, {'SOM'}},...
%                                    'alpha_types', {{'linear', 'constant'}, {'linear'}},...
%                                    'alpha_inits', {{1, 0.1}, {1}},...
%                                    'radius_types', {{'linear', 'linear'}, {'linear'}},...
%                                    'radius_inits', {{5, 5}, {5}},...
%                                    'radius_fins', {{1, 1}, {1}},...
%                                    'window_sizes', {{NaN, 0.1}, {NaN}},...
%                                    'alpha_feature_types', {{NaN, 'constant'}, {NaN}},...
%                                    'alpha_feature_inits', {{NaN, 0.3}, {NaN}},...
%                                    'auxdist_type', 'hellinger',...
%                                    'feature_selection', 'nodestats_hard',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared', 'sumsquared'},...
%                                    'classification_method', 'node');

% Experiment = Experiment.addLearner('name', 'LVQ1',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LVQ1'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'fuzzy',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
% 
% Experiment = Experiment.addLearner('name', 'OLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'OLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'fuzzy',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
% 
% Experiment = Experiment.addLearner('name', 'RLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'RLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},... 
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'fuzzy',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
%                                
% Experiment = Experiment.addLearner('name', 'RLVQ LDA',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'RLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},... 
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'lda',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                               
% Experiment = Experiment.addLearner('name', 'LVQ1-> RLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{0.5}},...
%                                    'updaters', {{'LVQ', 'RLVQ'}},...
%                                    'alpha_types', {{'constant', 'constant'}},...
%                                    'alpha_inits', {{0.1, 0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.01}},...
%                                    'feature_selection', 'fuzzy',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                               
% Experiment = Experiment.addLearner('name', 'LDALVQ1',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LDALVQ1'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'lda',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
% 
% Experiment = Experiment.addLearner('name', 'LCALDAOLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LCALDAOLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'lda',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                               
% Experiment = Experiment.addLearner('name', 'LVQ1-> LDALVQ1',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{0.5}},...
%                                    'updaters', {{'LVQ', 'LDALVQ1'}},...
%                                    'alpha_types', {{'constant', 'constant'}},...
%                                    'alpha_inits', {{0.1, 0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.01}},...
%                                    'feature_selection', 'fuzzy',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                                
% Experiment = Experiment.addLearner('name', 'GRLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'GRLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{1}},...
%                                    'feature_selection', 'hard',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
%                                
% Experiment = Experiment.addLearner('name', 'LDAGLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LDAGLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'fuzzy',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');

% Experiment = Experiment.addLearner('name', 'LDALVQ1_3',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LDALVQ1_3'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'lda',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
%                                 
% Experiment = Experiment.addLearner('name', 'LDAOLVQ3',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LDAOLVQ3'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'lda',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                               
% Experiment = Experiment.addLearner('name', 'LDAOLVQ with LA Classification',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LDAOLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'exp',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                               
%                                
% Experiment = Experiment.addLearner('name', 'LDAOLVQ2',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LDAOLVQ2'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'fuzzy',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                                
% Experiment = Experiment.addLearner('name', 'LDAOLVQ2 Classwise',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[24 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LDAOLVQ2'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'classwise',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                               
% Experiment = Experiment.addLearner('name', 'LALDALVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[10 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LALDALVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'exp',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
%                                
% Experiment = Experiment.addLearner('name', 'LALDAOLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LALDAOLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'exp',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');


% Experiment = Experiment.addLearner('name', 'GLVQ -> GRLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'codebook_init_method', {'sample'},...
%                                    'phase_shifts', {{0.5}},...
%                                    'updaters', {{'GLVQ', 'GRLVQ'}},...
%                                    'alpha_types', {{'constant', 'constant'}},...
%                                    'alpha_inits', {{0.1, 0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.01}},...
%                                    'feature_selection', 'rlvq',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                               
% Experiment = Experiment.addLearner('name', 'OLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'OLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'off',...
%                                    'feature_selection_feedback', false,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
%                                
% Experiment = Experiment.addLearner('name', 'RLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'RLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'fuzzy',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
%                                
% Experiment = Experiment.addLearner('name', 'LDAOLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'phase_shifts', {{NaN}},...
%                                    'updaters', {{'LDAOLVQ'}},...
%                                    'alpha_types', {{'constant'}},...
%                                    'alpha_inits', {{0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'lda',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                               
% Experiment = Experiment.addLearner('name', 'GLVQ -> LDALVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'phase_shifts', {{0.5}},...
%                                    'updaters', {{'GLVQ', 'LDALVQ'}},...
%                                    'alpha_types', {{'constant', 'constant'}},...
%                                    'alpha_inits', {{1, 1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{1}},...
%                                    'feature_selection', 'rlvq',...
%                                    'feature_selection_feedback', true,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
                               
% Experiment = Experiment.addLearner('name', 'SOM -> GLVQ',...
%                                    'type', 'unilayer',...
%                                    'modality_types', {'codebook'},...
%                                    'codebook_sizes', {[6 1]},...
%                                    'codebook_neighs', {'bubble'},...
%                                    'codebook_lattices', {'hexa'},...
%                                    'phase_shifts', {{0.5}},...
%                                    'updaters', {{'SOM', 'GLVQ'}},...
%                                    'alpha_types', {{'linear', 'constant'}},...
%                                    'alpha_inits', {{1, 0.1}},...
%                                    'radius_types', {{'linear'}},...
%                                    'radius_inits', {{5}},...
%                                    'radius_fins', {{1}},...
%                                    'window_sizes', {{NaN}},...
%                                    'alpha_feature_types', {{'constant'}},...
%                                    'alpha_feature_inits', {{0.1}},...
%                                    'feature_selection', 'off',...
%                                    'feature_selection_feedback', false,...
%                                    'metric', {'sumsquared'},...
%                                    'classification_method', 'node');
            
Experiment = Experiment.run();