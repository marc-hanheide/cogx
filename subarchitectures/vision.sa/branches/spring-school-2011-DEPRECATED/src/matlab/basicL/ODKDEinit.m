function mC=ODKDEinit
%ODKDE initialisation.
%mC: model of Cs



% - parameters of the online classifier
dim = 6;  % number of features
switchSelectionSeeds = 0 ;                      % (0) turn off approximative compression
typeRecDescr = 'dKDE' ;                         % 'dKDE', 'oKDE', switchess between discriminative and standard oKDE
% minNumDataPointsToFormKDE = dim + 1 ;           % (int) minimum number of points before forming a KDE
minNumDataPointsToFormKDE = dim;%(dim^2-dim)/2+dim+dim ; % (int) minimum number of points before forming a KDE
react_compression_to_feature_selection = 1 ;    % (1) apply compression in subdimension selected by feature selection
min_samps_per_model_feat_sel = dim;%(dim^2-dim)/2+dim+dim ; % (int) minimum number of samples per model observed before applying feature selection
min_th_feat_sel =0.15 ;0.07 ;                       % (double) threshold on importance below which a feature is removed
costThreshold.thReconstructive = 0.01 ;         % thresholds on reconstructive and discriminative compression
costThreshold.thDiscriminative =  0.002 ;
autoUpdateThres_upper = 0.1 ;                   % in self verified mode, this is the threshold on entropy for asking
autoUpdateThres_lower = 1e-2 ;                  % in self verified mode, this is the threshold on entropy for auto update
random_fselect_threshold = 1   ;               % probability of feature selection occuring when called
pair_dist_struct_use_approx = 1 ;               % switch for compression: test which classes shuld be taken into account during compression of i-th class
force_value_init_of_maxNumCompsBeforeCompression = 5 ; % determine initial num of components before compression
% - end of parameters


% create a classifier object with selected parameters

numSC=2;

for i=1:numSC
   mC{i} = executeOperatorIKDEClsfr( [], 'init', 'compressionClusterThresh', costThreshold,...
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
      'force_value_init_of_maxNumCompsBeforeCompression', force_value_init_of_maxNumCompsBeforeCompression) ;
end
