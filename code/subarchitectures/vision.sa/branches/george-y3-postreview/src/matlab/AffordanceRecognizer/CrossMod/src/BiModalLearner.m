classdef BiModalLearner < Learner
    % BIMODALLEARNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        %% ------- *** OBJECTS *** ----------------------------------------
        %******************************************************************
        %******************************************************************
        % Cell array of modality objects...
        Modalities = [];
        
        % Mapping between modalities...
        crossMapping = [];
        
        %% ------- *** FUNCTION HANDLES *** -------------------------------
        %******************************************************************
        %******************************************************************
        % Handle to a method that computes an auxiliary distance...
        computeauxdist = [];
                
    end
    
    properties (SetAccess = private, Hidden = true)
        
        % Default settings...
        
        % Usage message...
        UsageMessage = ['\nBiModalLearner Usage: '...
                        'Please see the function comments for more detail.\n\n'];
                
    end
    
    methods

        %% ------- *** CONSTRUCTOR *** ------------------------------------
        %******************************************************************
        %******************************************************************
        function obj = BiModalLearner(varargin)
                          
            % Default settings...
            Data = [];
            ModalityTypes = {'codebook', 'codebook'};
            CodebookSizes = {[10 10], [10 10]};
            CodebookNeighs = {'bubble', 'bubble'};
            CodebookLattices = {'hexa', 'hexa'};
            CodebookShapes = {'sheet', 'sheet'};
            CodebookInitMethods = {'rand', 'rand'};
            TrainingIndices = [];
            TestIndices =[];
            epochs = 1;
            normalization = 'all';
            normalization_method = 'range';
            
            
            % Loop through arguments...
            i = 1;
            iPassedArgs = 1;
            while i <= length(varargin), 
                argok = 1; 
                if ischar(varargin{i}), 
                    switch lower(varargin{i}),
                        case 'name', i=i+1; obj.Name = varargin{i};
                        case 'data', i=i+1; Data = varargin{i};
                        case 'trainingdata1epoch', i=i+1; obj.TrainingData1Epoch = varargin{i};
                        case 'trainingdata', i=i+1; obj.TrainingData = varargin{i};
                        case 'testdata', i=i+1; obj.TestData = varargin{i};
                        case 'modality_types', i=i+1; ModalityTypes = varargin{i};
                        case 'codebook_sizes', i=i+1; CodebookSizes = varargin{i};
                        case 'codebook_neighs', i=i+1; CodebookNeighs = varargin{i};
                        case 'codebook_lattices', i=i+1; CodebookLattices = varargin{i};
                        case 'codebook_shapes', i=i+1; CodebookShapes = varargin{i};
                        case 'codebook_init_method', i=i+1; CodebookInitMethods = varargin{i};
                        case 'trainingindices', i=i+1; TrainingIndices = varargin{i};
                        case 'testindices', i=i+1; TestIndices = varargin{i};
                        case 'epochs', i=i+1; epochs = varargin{i};
                        case 'normalization', i=i+1; normalization = varargin{i};
                        case 'normalization_method', i=i+1; normalization_method = varargin{i};
                        otherwise
                            PassedArgs{iPassedArgs} = varargin{i};
                            PassedArgs{iPassedArgs+1} = varargin{i+1};
                            i = i + 1;
                            iPassedArgs = iPassedArgs + 2;
                    end
                else
                    argok = 0;
                end

                if ~argok, 
                    disp(['BiModalLearner.BiModalLearner(): Ignoring invalid argument #' num2str(i)]);
                    % fprintf(obj.UsageMessage);
                end

                i = i + 1;
            end
            
            %% SET UP TRAINING AND/OR TEST DATA STRUCTS FOR 1 EPOCH -------
            %--------------------------------------------------------------
            if isempty(Data) && (isempty(obj.TrainingData1Epoch) || isempty(obj.TrainingData) || isempty(obj.TestData))
                % error(obj.UsageMessage);
                    return;
                    
            elseif isempty(obj.TrainingData) || isempty(obj.TestData)
                
                if ~isempty(TrainingIndices)
                    [obj.TrainingData1Epoch obj.TestData] =...
                        obj.setupdatastructs(Data,...
                                             'TrainingIndices', TrainingIndices,...
                                             'epochs', 1);
                elseif ~isempty(TestIndices)
                    [obj.TrainingData1Epoch obj.TestData] =...
                        obj.setupdatastructs(Data,...
                                             'TestIndices', TestIndices,...
                                             'epochs', 1);
                end
            end
            
            %% NORMALIZE DATA FOR 1 EPOCH ---------------------------------
            %--------------------------------------------------------------
            switch normalization                
                case 'all'
                    [TempData InModalityNorm] = obj.normalize([obj.TrainingData1Epoch.Modalities{1}.FeatureVectors...
                                                               obj.TestData.Modalities{1}.FeatureVectors]',...
                                                              normalization_method);
                                                          
                    % Record the important stuff...
                    obj.TrainingData1Epoch.Modalities{1}.NormedFeatureVectors =...
                        TempData(1:size(obj.TrainingData1Epoch.Modalities{1}.FeatureVectors,2),:)';
                    obj.TestData.Modalities{1}.NormedFeatureVectors =...
                        TempData(size(obj.TrainingData1Epoch.Modalities{1}.FeatureVectors,2)+1:end,:)';
                    
                    [TempData OutModalityNorm] = obj.normalize([obj.TrainingData1Epoch.Modalities{2}.FeatureVectors...
                                                               obj.TestData.Modalities{2}.FeatureVectors]',...
                                                              normalization_method);
                                                          
                    % Record the important stuff...
                    obj.TrainingData1Epoch.Modalities{2}.NormedFeatureVectors =...
                        TempData(1:size(obj.TrainingData1Epoch.Modalities{2}.FeatureVectors,2),:)';
                    obj.TestData.Modalities{2}.NormedFeatureVectors =...
                        TempData(size(obj.TrainingData1Epoch.Modalities{2}.FeatureVectors,2)+1:end,:)';
                    
                case {'train', 'training', 'training_data'}
                    [TempData InModalityNorm] = obj.normalize(obj.TrainingData1Epoch.Modalities{1}.FeatureVectors',...
                                                              normalization_method);
                                                          
                    % Record the important stuff...
                    obj.TrainingData1Epoch.Modalities{1}.NormedFeatureVectors =...
                        TempData(1:size(obj.TrainingData1Epoch.Modalities{1}.FeatureVectors,2),:)';
                    
                    [TempData OutModalityNorm] = obj.normalize(obj.TrainingData1Epoch.Modalities{2}.FeatureVectors',...
                                                              normalization_method);
                                                          
                    % Record the important stuff...
                    obj.TrainingData1Epoch.Modalities{2}.NormedFeatureVectors =...
                        TempData(1:size(obj.TrainingData1Epoch.Modalities{2}.FeatureVectors,2),:)';
               
                otherwise
                    obj.TrainingData1Epoch.Modalities{1}.NormedFeatureVectors =...
                        obj.TrainingData1Epoch.Modalities{1}.FeatureVectors;
               
                    obj.TestData.Modalities{1}.NormedFeatureVectors =...
                        obj.TestData.Modalities{1}.FeatureVectors;
                    
                    InModalityNorm = cell(size(obj.TrainingData1Epoch.Modalities{1}.FeatureVectors,1),1);
                    
                    obj.TrainingData1Epoch.Modalities{2}.NormedFeatureVectors =...
                        obj.TrainingData1Epoch.Modalities{2}.FeatureVectors;
               
                    obj.TestData.Modalities{2}.NormedFeatureVectors =...
                        obj.TestData.Modalities{2}.FeatureVectors;
                    
                    OutModalityNorm = cell(size(obj.TrainingData1Epoch.Modalities{2}.FeatureVectors,1),1);
            end 
            
%             %% SET UP TRAINING AND/OR TEST DATA STRUCTS -------------------
%             %--------------------------------------------------------------
%             if isempty(Data) && (isempty(obj.TrainingData) || isempty(obj.TestData))
%                 error(obj.UsageMessage);
%                     return;
%                     
%             elseif isempty(obj.TrainingData) || isempty(obj.TestData)
%                 [obj.TrainingData obj.TestData] =...
%                     obj.setupdatastructs(Data,...
%                                          'TrainingIndices', TrainingIndices,...
%                                          'TestIndices', TestIndices,...
%                                          'epochs', epochs);
%             end
%             
%             %% NORMALIZE TRAINING DATA ------------------------------------
%             %--------------------------------------------------------------
%             % Temporary data structs...
%             SOMDataTempIn = som_data_struct(obj.TrainingData.Modalities{1}.FeatureVectors',...
%                                           'comp_names', obj.TrainingData.Modalities{1}.FeatureNames');
%             SOMDataTempOut = som_data_struct(obj.TrainingData.Modalities{2}.FeatureVectors',...
%                                           'comp_names', obj.TrainingData.Modalities{2}.FeatureNames');
% 
%             % Normalize...
%             SOMDataTempIn = som_normalize(SOMDataTempIn, 'range');
%             SOMDataTempOut = som_normalize(SOMDataTempOut, 'range');
%             
%             % Record the important stuff...
%             obj.TrainingData.Modalities{1}.NormedFeatureVectors = SOMDataTempIn.data';
%             obj.TrainingData.Modalities{2}.NormedFeatureVectors = SOMDataTempOut.data';
%             InModalityNorm = SOMDataTempIn.comp_norm;
%             OutModalityNorm = SOMDataTempOut.comp_norm;
            
            %% INSTANTIATE MODALITY OBJECTS -------------------------------
            % Instantiate this BiModalLearner object's 2 modality objects...
            %--------------------------------------------------------------
            obj.Modalities{1} =...
                ModalityFactory.createModality(obj.TrainingData1Epoch.Modalities{1},...
                                               'modality_type', ModalityTypes{1},...
                                               'codebook_size', CodebookSizes{1},...
                                               'codebook_shape', CodebookShapes{1},...
                                               'codebook_lattice', CodebookLattices{1},...
                                               'codebook_neigh', CodebookNeighs{1},...
                                               'codebook_init_method', CodebookInitMethods{1},...
                                               'normalization_struct', InModalityNorm,...
                                               'normalization_method', normalization_method);
                                     
            obj.Modalities{2} =...
                ModalityFactory.createModality(obj.TrainingData1Epoch.Modalities{2},...
                                               'modality_type', ModalityTypes{2},...
                                               'codebook_size', CodebookSizes{2},...
                                               'codebook_shape', CodebookShapes{2},...
                                               'codebook_lattice', CodebookLattices{2},...
                                               'codebook_neigh', CodebookNeighs{2},...
                                               'codebook_init_method', CodebookInitMethods{2},...
                                               'normalization_struct', OutModalityNorm,...
                                               'normalization_method', normalization_method);
                                           
            %% SETUP TRAINING DATA FOR MULTIPLE EPOCHS --------------------            
            %--------------------------------------------------------------
            if isempty(Data) && (isempty(obj.TrainingData) || isempty(obj.TestData))
                % error(obj.UsageMessage);
                    return;
                    
            elseif isempty(obj.TrainingData) || isempty(obj.TestData)
                [obj.TrainingData obj.TestData] =...
                    obj.setupdatastructs(Data,...
                                         'TrainingIndices', TrainingIndices,...
                                         'TestIndices', TestIndices,...
                                         'epochs', epochs);
                                     
                if ~isempty(TrainingIndices)
                    [obj.TrainingData obj.TestData] =...
                        obj.setupdatastructs(Data,...
                                             'TrainingIndices', TrainingIndices,...
                                             'epochs', epochs);
                elseif ~isempty(TestIndices)
                    [obj.TrainingData obj.TestData] =...
                        obj.setupdatastructs(Data,...
                                             'TestIndices', TestIndices,...
                                             'epochs', epochs);
                end
            end
            
            %% NORMALIZE DATA ---------------------------------------------
            %--------------------------------------------------------------
            switch normalization                
                case 'all'
                    [TempData InModalityNorm] = obj.normalize([obj.TrainingData.Modalities{1}.FeatureVectors...
                                                               obj.TestData.Modalities{1}.FeatureVectors]',...
                                                              normalization_method);
                                                          
                    % Record the important stuff...
                    obj.TrainingData.Modalities{1}.NormedFeatureVectors =...
                        TempData(1:size(obj.TrainingData.Modalities{1}.FeatureVectors,2),:)';
                    obj.TestData.Modalities{1}.NormedFeatureVectors =...
                        TempData(size(obj.TrainingData.Modalities{1}.FeatureVectors,2)+1:end,:)';
                    
                    [TempData OutModalityNorm] = obj.normalize([obj.TrainingData.Modalities{2}.FeatureVectors...
                                                               obj.TestData.Modalities{2}.FeatureVectors]',...
                                                              normalization_method);
                                                          
                    % Record the important stuff...
                    obj.TrainingData.Modalities{2}.NormedFeatureVectors =...
                        TempData(1:size(obj.TrainingData.Modalities{2}.FeatureVectors,2),:)';
                    obj.TestData.Modalities{2}.NormedFeatureVectors =...
                        TempData(size(obj.TrainingData.Modalities{2}.FeatureVectors,2)+1:end,:)';
                    
                case {'train', 'training', 'training_data'}
                    [TempData InModalityNorm] = obj.normalize(obj.TrainingData.Modalities{1}.FeatureVectors',...
                                                              normalization_method);
                                                          
                    % Record the important stuff...
                    obj.TrainingData.Modalities{1}.NormedFeatureVectors =...
                        TempData(1:size(obj.TrainingData.Modalities{1}.FeatureVectors,2),:)';
                    
                    [TempData OutModalityNorm] = obj.normalize(obj.TrainingData.Modalities{2}.FeatureVectors',...
                                                              normalization_method);
                                                          
                    % Record the important stuff...
                    obj.TrainingData.Modalities{2}.NormedFeatureVectors =...
                        TempData(1:size(obj.TrainingData.Modalities{2}.FeatureVectors,2),:)';
               
                otherwise
                    obj.TrainingData.Modalities{1}.NormedFeatureVectors =...
                        obj.TrainingData.Modalities{1}.FeatureVectors;
               
                    obj.TestData.Modalities{1}.NormedFeatureVectors =...
                        obj.TestData.Modalities{1}.FeatureVectors;
                    
                    InModalityNorm = cell(size(obj.TrainingData.Modalities{1}.FeatureVectors,1),1);
                    
                    obj.TrainingData.Modalities{2}.NormedFeatureVectors =...
                        obj.TrainingData.Modalities{2}.FeatureVectors;
               
                    obj.TestData.Modalities{2}.NormedFeatureVectors =...
                        obj.TestData.Modalities{2}.FeatureVectors;
                    
                    OutModalityNorm = cell(size(obj.TrainingData.Modalities{2}.FeatureVectors,1),1);
            end

            
            %% INSTANTIATE MAPPING OBJECT ---------------------------------
            % Instantiate this CrossMod co-occurence mapping object...
            %--------------------------------------------------------------
            % obj.crossMapping = RawMapping();
            obj.crossMapping = HebbianMapping(obj.Modalities{1}, obj.Modalities{2});
            
            
            %% CREATE DEFAULT computeauxdist HANDLE -----------------------
            %--------------------------------------------------------------
            obj.computeauxdist = @obj.hellinger;
            
            %% PASS REMAINING ARGUMENTS TO SET METHOD ---------------------
            %--------------------------------------------------------------
            obj = obj.set(PassedArgs{1:end});
            
        end
        
        %% ------- *** SET PROPERTIES *** ---------------------------------
        %******************************************************************
        %******************************************************************
        function obj = set(obj, varargin)
        
            %% CHECK ARGUMENTS --------------------------------------------
            %--------------------------------------------------------------
            % default values
            epochs_value = 1;
            randomize_training_data = false;
            randomize_test_data = false;
            Updaters = {{'SOM', 'HeurORLVQ'}, {'SOM'}};
            PhaseShifts = {{0.5}, {NaN}};   
            AlphaTypes = {{'linear', 'constant'}, {'inv'}};
            AlphaInits = {{1, 0.3}, {1}};
            RadiusTypes = {{'linear', 'linear'}, {'linear'}};
            RadiusInits = {{1, 1}, {5}};
            RadiusFins = {{1, 1}, {1}};
            WindowSizes = {NaN 0.3};
            AlphaFeatureTypes = {{NaN, 'constant'}, {NaN}};
            AlphaFeatureInits = {{NaN, 0.1}, {NaN}};
            Metrics = {{'euclidean'}, {'euclidean'}};
            auxdist_type_value = 'hellinger';
                       
            
            % varargin
            i=1; 
            while i<=length(varargin), 
              argok = 1; 
              if ischar(varargin{i}), 
                switch varargin{i}, 
                    % argument IDs
                    case {'randomize_train', 'randomize_training_data',...
                          'randomizetrain', 'randomizetrainingdata'},...
                            i=i+1; randomize_training_data = varargin{i};
                    case {'randomize_test', 'randomize_test_data',...
                          'randomizetest', 'randomizetestdata'},...
                            i=i+1; randomize_test_data = varargin{i};
                    case {'updaters'},  i=i+1; Updaters = varargin{i};
                    case {'phaseshifts', 'phase_shifts'},  i=i+1; PhaseShifts = varargin{i};   
                    case {'alphatypes', 'alpha_types'},  i=i+1; AlphaTypes = varargin{i};
                    case {'alphainits', 'alpha_inits'},  i=i+1; AlphaInits = varargin{i};
                    case {'radiustypes', 'radius_types'},  i=i+1; RadiusTypes = varargin{i};
                    case {'radiusinits', 'radius_inits'},  i=i+1; RadiusInits = varargin{i};
                    case {'radiusfins', 'radius_fins'},  i=i+1; RadiusFins = varargin{i};
                    case {'windowsizes', 'window_sizes'}, i=i+1; WindowSizes = varargin{i};
                    case {'alphafeaturetypes', 'alpha_feature_types'}, i=i+1; AlphaFeatureTypes = varargin{i};
                    case {'alphafeatureinits', 'alpha_feature_inits'},  i=i+1; AlphaFeatureInits = varargin{i};
                    case {'auxdist', 'auxdist_type'}, i=i+1; auxdist_type_value = varargin{i};
                    case {'featureselection', 'feature_selection'}, i=i+1; obj.feature_selection = varargin{i};
                    case {'featureselectionmax', 'feature_selection_max'}, i=i+1; obj.feature_selection_max = varargin{i};
                    case {'featureselectionfeedback', 'feature_selection_feedback',...
                          'featureselectionintraining', 'feature_selection_in_training'},...
                            i=i+1; obj.feature_selection_feedback = varargin{i};
                    case {'classificationmethod', 'classification_method'},...
                            i=i+1; obj.classification_method = varargin{i};
                    case {'metric'}, i=i+1; Metrics = varargin{i};
                    case {'record'}, i=i+1; obj.record = varargin{i};
                    case {'display'}, i=i+1; obj.display = varargin{i};
                    % Ignore codebook related arguments.  They shouldn't be set
                    % independently of the constructor.
                    case {'codebook_sizes', 'codebook_lattices',...
                          'codebook_shapes', 'codebook_neighs'}, i=i+1;
                      
                 	otherwise, argok=0;
                end
              else
                argok = 0;
              end
              if ~argok, 
                disp(['BiModalLearner.set(): Ignoring invalid argument #' num2str(i+1)]); 
              end
              i = i+1;
            end
            
            %% CREATE computeauxdist HANDLE -------------------------------
            %--------------------------------------------------------------
            switch auxdist_type_value,
                case {'hellinger'},
                    obj.computeauxdist = @obj.hellinger;
                case {'chi squared', 'chisquared', 'chi_squared', 'chi-squared'},
                    obj.computeauxdist = @obj.chisquared;
                case {'kullback leibler', 'kullbackleibler', 'kullback_leibler', 'kullback-leibler',...
                      'cross entropy', 'crossentropy', 'cross-entropy', 'cross_entropy'},
                    obj.computeauxdist = @obj.kullbackleibler;
                case {'totalvariation', 'totalvariation', 'total_variation', 'total-variation'},
                    obj.computeauxdist = @obj.totalvariation;
                case {'cross correlation', 'crosscorrelation', 'cross_correlation', 'cross-correlation'},
                    obj.computeauxdist = @obj.crosscorrelation;
                case {'ridge'},
                    obj.computeauxdist = @obj.ridge;
                case {'earth movers', 'earthmovers', 'earth_movers', 'earth-movers'},
                    obj.computeauxdist = @obj.earthmovers;
                    
                otherwise
                    obj.computeauxdist = @obj.hellinger;
            end
            
            %% RANDOMIZE DATA SETS ----------------------------------------
            % Randomize the data if required...
            %--------------------------------------------------------------
            if randomize_training_data
                obj.TrainingData = obj.randomize(obj.TrainingData);
                obj.TrainingData.Randomized = true;
            end
            
            if randomize_test_data
                obj.TestData = obj.randomize(obj.TestData);
                obj.TestData.Randomized = true;
            end


            %% SETUP ------------------------------------------------------
            %--------------------------------------------------------------
            % Set IN modality SOM properties...
            obj.Modalities{1} = obj.Modalities{1}.set('metric', Metrics{1},...
                                                      'updaters', Updaters{1},...
                                                      'phase_shifts', PhaseShifts{1},...
                                                      'alpha_types', AlphaTypes{1},...
                                                      'alpha_inits', AlphaInits{1},...
                                                      'radius_types', RadiusTypes{1},...
                                                      'radius_inits', RadiusInits{1},...
                                                      'radius_fins', RadiusFins{1},...
                                                      'window_sizes', WindowSizes{1},...
                                                      'alpha_feature_types', AlphaFeatureTypes{1},...
                                                      'alpha_feature_inits', AlphaFeatureInits{1},...
                                                      'trainlen', size(obj.TrainingData.FeatureVectors,2),...
                                                      'nClasses', obj.TrainingData.Modalities{1}.nGroundTruths,...
                                                      'feature_selection_feedback', obj.feature_selection_feedback,...
                                                      'record', obj.record,...
                                                      'display', obj.display);
                                            
            
            % Decide how many auxiliary distances are to be calculated
            % depending on the algorithm...
            for i = 1:length(Updaters{1})
                switch lower(Updaters{1}{i})
                        case {'heurlvq3', 'heurmamrlvq', 'heurfmamrlvq'}
                            obj.Modalities{1}.nAuxDists = 2;
                end
            end
            
            % Set OUT modality SOM properties...
            obj.Modalities{2} = obj.Modalities{2}.set('metric', Metrics{2},...
                                                      'updaters', Updaters{2},...
                                                      'phase_shifts', PhaseShifts{2},...
                                                      'alpha_types', AlphaTypes{2},...
                                                      'alpha_inits', AlphaInits{2},...
                                                      'radius_types', RadiusTypes{2},...
                                                      'radius_inits', RadiusInits{2},...
                                                      'radius_fins', RadiusFins{2},...
                                                      'window_sizes', WindowSizes{2},...
                                                      'alpha_feature_types', AlphaFeatureTypes{2},...
                                                      'alpha_feature_inits', AlphaFeatureInits{2},...
                                                      'trainlen', size(obj.TrainingData.FeatureVectors,2),...
                                                      'record', obj.record,...
                                                      'display', obj.display);
                                                                                

            % Set up CurrentSample data struct...
            obj.CurrentSample.Modalities{1}.FeatureNames = obj.TrainingData.Modalities{1}.FeatureNames;
            obj.CurrentSample.Modalities{1}.ClassNames = obj.TrainingData.Modalities{1}.ClassNames;
            obj.CurrentSample.Modalities{1}.nGroundTruths = obj.TrainingData.Modalities{1}.nGroundTruths;
            obj.CurrentSample.Modalities{1}.GroundTruthLabelIndices = obj.TrainingData.Modalities{1}.GroundTruthLabelIndices;
            obj.CurrentSample.Modalities{2}.FeatureNames = obj.TrainingData.Modalities{2}.FeatureNames;
            obj.CurrentSample.Modalities{2}.ClassNames = obj.TrainingData.Modalities{2}.ClassNames;
            obj.CurrentSample.Modalities{2}.nGroundTruths = obj.TrainingData.Modalities{2}.nGroundTruths;
            obj.CurrentSample.Modalities{2}.GroundTruthLabelIndices = obj.TrainingData.Modalities{2}.GroundTruthLabelIndices;
            
            
        end
        
        %% ------- *** CLEAR CLASS LABELS *** -----------------------------
        %******************************************************************
        %******************************************************************
        function obj = clearclasslabels(obj, varargin)
            
            for iMod = 1:length(obj.Modalities)
                obj.Modalities{iMod}.ClassLabels = [];
            end
            
        end
        
        %% ------- *** TRAIN *** ------------------------------------------
        %******************************************************************
        %******************************************************************
        function obj = train(obj, varargin)
            
            % Set defaults...
            sample_size = size(obj.TrainingData.FeatureVectors,2);
            
            % Loop through arguments...
            i = 1;
            while i <= length(varargin), 
                argok = 1;
                if isnumeric(varargin{i})
                    sample_size = varargin{i};
                    
                elseif ischar(varargin{i}), 
                    % switch lower(varargin{i}), 
                    %     case 'som_size', i=i+1; obj.SOM_size = varargin{i};
                    % 
                    %     otherwise
                    %         argok = 0;
                    % end
                else
                    argok = 0;
                end

                if ~argok, 
                    disp(['BiModalLearner.train(): Ignoring invalid argument #' num2str(i)]);
                    fprintf(obj.UsageMessage);
                end

                i = i + 1;
            end
            
            
            
            %% TRAINING LOOP ----------------------------------------------
            %--------------------------------------------------------------
            if obj.display                
                fprintf(1, '\nTotal training samples:   %d', size(obj.TrainingData.FeatureVectors,2));
                fprintf(1, '\nCurrent training sample:     ');
            end
            
            for t = obj.t+1 : min(obj.t + sample_size, size(obj.TrainingData.FeatureVectors,2))
            % for t = 1:size(obj.TrainingData.FeatureVectors,2)
                
                obj.t = t;
                
                % Print progress...
                if obj.display
                    Backspaces = [];
                    for iBack = 1:ceil(log10(t+1))
                        Backspaces = [Backspaces '\b'];
                    end
                    fprintf(1, [Backspaces '%d'], obj.t);
                end
                
                
                %% GRAB CURRENT SAMPLE ------------------------------------
                %----------------------------------------------------------
                obj.CurrentSample.Modalities{1}.FeatureVectors(:,1) =...
                    obj.TrainingData.Modalities{1}.FeatureVectors(:,t);
                obj.CurrentSample.Modalities{1}.NormedFeatureVectors(:,1) =...
                    obj.TrainingData.Modalities{1}.NormedFeatureVectors(:,t);
                obj.CurrentSample.Modalities{1}.ClassLabels(:,1) =...
                    obj.TrainingData.Modalities{1}.ClassLabels(:,t);
                obj.CurrentSample.Modalities{1}.GroundTruthLabelIndices =...
                    obj.TrainingData.Modalities{1}.GroundTruthLabelIndices;
                obj.CurrentSample.Modalities{2}.FeatureVectors(:,1) =...
                    obj.TrainingData.Modalities{2}.FeatureVectors(:,t);
                obj.CurrentSample.Modalities{2}.NormedFeatureVectors(:,1) =...
                    obj.TrainingData.Modalities{2}.NormedFeatureVectors(:,t);
                obj.CurrentSample.Modalities{2}.ClassLabels(:,1) =...
                    obj.TrainingData.Modalities{2}.ClassLabels(:,t);
                obj.CurrentSample.Modalities{2}.GroundTruthLabelIndices =...
                    obj.TrainingData.Modalities{2}.GroundTruthLabelIndices;
                
                
                %% TRAIN OUTPUT MODALITY ----------------------------------
                %----------------------------------------------------------
                obj.Modalities{2} =...
                    obj.Modalities{2}.train(obj.CurrentSample.Modalities{2});
                                
                %% FIND INPUT MODALITY BMU --------------------------------
                %----------------------------------------------------------               
                obj.Modalities{1}.BMUs =...
                    obj.Modalities{1}.findbmus(obj.CurrentSample.Modalities{1}.NormedFeatureVectors');
                
                
                %% CALCULATE AUXILIARY DISTANCES --------------------------
                %----------------------------------------------------------
%                 obj.Modalities{1}.AuxDists(1) =...
%                     obj.computeauxdist(obj.crossMapping.Weights(obj.Modalities{1}.BMUs(1),:)',...
%                                        obj.Modalities{2}.Activations,...
%                                        obj.Modalities{2}.CostMatrix);
                                   
                for iBMU = 1:obj.Modalities{1}.nAuxDists
                    obj.Modalities{1}.AuxDists(iBMU) =...
                           obj.computeauxdist(obj.crossMapping.Weights(obj.Modalities{1}.BMUs(iBMU),:)',...
                                              obj.Modalities{2}.Activations,...
                                              obj.Modalities{2}.CostMatrix);
                end
                
                % Calculate it for the second BMU if required...
%                 if obj.Modalities{1}.nAuxDists > 1
%                         obj.Modalities{1}.AuxDists(2) =...
%                            obj.computeauxdist(obj.crossMapping.Weights(obj.Modalities{1}.BMUs(2),:)',...
%                                               obj.Modalities{2}.Activations,...
%                                               obj.Modalities{2}.CostMatrix);
%                 end

                %% CALCULATE AUXILIARY DISTANCE RUNNING STATISTICS --------
                %----------------------------------------------------------
                obj.Modalities{1}.AuxDistStats.push(obj.Modalities{1}.AuxDists(1));
                
                %% CALCULATE INPUT MODALITY SAMPLE-NODE DISTANCES ---------
                %----------------------------------------------------------
                % obj = obj.mixeddistances(CurrentSample);
                
                %% TRAIN INPUT MODALITY -----------------------------------
                %---------------------------------------------------------- 
                obj.Modalities{1} =...
                    obj.Modalities{1}.train(obj.CurrentSample.Modalities{1});
                
                %% VISUALIZATION ------------------------------------------
                %----------------------------------------------------------
                % if ~mod(t,5)
                %     subplot(1,2,1);
                %     som_grid(obj.Modalities{1}.SOM, 'Coord', obj.Modalities{1}.SOM.codebook);
                %     subplot(1,2,2);
                %     som_grid(obj.Modalities{2}.SOM, 'Coord', obj.Modalities{2}.SOM.codebook);
                %     drawnow
                % end
                
                %% TRAIN CROSS-MODAL MAPPING ----------------------------------
                %--------------------------------------------------------------
                % obj.crossMapping = obj.crossMapping.train(inMatches', outMatches');
                % fprintf('\nTraining cross-modal mappings...');
                % obj.Modalities{1}.Activations = zeros(size(obj.Modalities{1}.SOM.codebook,1),1);
                % obj.Modalities{1}.Activations(obj.Modalities{1}.BMUs(1)) = 1;
                obj.crossMapping = obj.crossMapping.train(obj.Modalities{1}.BMUs(1), obj.Modalities{2}.BMUs(1),...
                                                          obj.Modalities{1}.Activations, obj.Modalities{2}.Activations);
                                                      
                %% RECORD MODALITY TRAINING INFORMATION OVER TIME ---------
                %----------------------------------------------------------
                if obj.record                    
                    obj.Modalities{1}.AuxDistsRecord(t,:) = obj.Modalities{1}.AuxDists;
                    obj.Modalities{1}.AuxDistMeanRecord(t,:) = obj.Modalities{1}.AuxDistStats.mean();
                    obj.Modalities{1}.AuxDistStDRecord(t,:) = obj.Modalities{1}.AuxDistStats.std();
                    obj.Modalities{1}.GroundTruthRecord(t,:) =...
                        find(obj.CurrentSample.Modalities{1}.ClassLabels(...
                                obj.CurrentSample.Modalities{1}.GroundTruthLabelIndices,:));
                    obj.Modalities{1}.ActivationsRecord(t,:) = obj.Modalities{1}.Activations;
                    obj.Modalities{1}.BMURecord(t, :) = obj.Modalities{1}.BMUs;
                end
                
                                                      
                %% CLEAR MODALITY BMUs ------------------------------------
                %----------------------------------------------------------
                obj.Modalities{1} = obj.Modalities{1}.clearbmus();
                obj.Modalities{2} = obj.Modalities{2}.clearbmus();
                
            end            
            
            obj.is_trained = true;
            
        end
        
        
        %% ------- *** CLASSIFY *** ---------------------------------------
        %******************************************************************
        %******************************************************************
        function [TestData Mask] = classify(obj, varargin)
            
            % Defaults...
            switch obj.classification_method
                case {'svm'},...
                    Method = 'svm';
                case {'lda'},...
                    Method = 'lda';
                case {'naivebayes', 'naivebayes_linear', 'naivebayes_diaglinear', 'diaglinear'},
                    Method = 'diaglinear';
                case {'naivebayes_quadratic', 'naivebayes_diagquadratic', 'diagquadratic'},...
                    Method = 'diagquadratic';
                    
                otherwise, Method = 'node';
            end
            
            % The TestData struct...
            TestData = [];
            
            % A flag that lets us know if we're using the TestData struct
            % in the object, or test data that was passed as an argument...
            using_internal_testdata = false;
                        
            Metric = 'euclidean';
            hebbian = true;
            
            % FOR THE MOMENT, THE CLASSIFY METHOD WILL JUST CLASSIFY
            % INTERNAL TEST SAMPLE INPUT MODALITY VECTORS...
            % 
            % ToDo: Extend this!
            
            %% CHECK THAT WE HAVE A TRAINED CLASSIFIER --------------------
            %--------------------------------------------------------------
            if ~obj.is_trained
                fprintf(['\n\nThe classifier has not been trained yet!\n\n'...
                       'Please use CrossMod.train first!\n\n\n']);
                return;
            end
            
            %% CHECK ARGUMENTS --------------------------------------------
            %--------------------------------------------------------------
            % varargin
            i=1; 
            while i<=length(varargin), 
              argok = 1; 
              if ischar(varargin{i}), 
                switch varargin{i}, 
                    % argument IDs
                    case {'data', 'testdata'}, i=i+1; TestDataIn = varargin{i};
                    case {'display'}, display = true;
                    case {'svm'}, Method = 'svm';
                    case {'lda'}, Method = 'lda';
                    case {'lvq'}, Method = 'lvq';
                    case {'nodewise', 'node'}, Method = 'node';
                    case {'euclidean', 'euclid', 'euc'}, Metric = 'euclidean';
                    case {'hellinger', 'hell'}, Metric = 'hellinger';
                    case {'clusterwise', 'cluster'}, Method = 'cluster';
                    case {'nonhebbian'}, hebbian = false;
                      
                    otherwise, argok=0; 
                end
              elseif isstruct(varargin{i})
                  TestDataIn = varargin{i};  
              else
                argok = 0; 
              end
              if ~argok, 
                disp(['BiModalLearner.classify(): Ignoring invalid argument #' num2str(i+1)]); 
              end
              i = i+1; 
            end
            
            %% SET UP TEST DATA -------------------------------------------
            %--------------------------------------------------------------
            if isempty(TestDataIn)
                
                TestData = obj.TestData;
                using_internal_testdata = true;
               
            elseif isnumeric(TestDataIn)
               
                NumericData = TestDataIn;        
                TestData.FeatureVectors = NumericData;
                
                if size(TestData.FeatureVectors,1) == size(obj.Modalities{1}.SOM.codebook,2)
                    
                    TestData.Modalities{1}.FeatureVectors = TestData.FeatureVectors;
                    
                elseif size(TestData.FeatureVectors,1) > size(obj.Modalities{1}.SOM.codebook,2)
                    
                    fprintf(['Warning: Dimensionality of numeric input too large'...
                             ', using first %d dimensions.\n'], size(obj.Modalities{1}.SOM.codebook,2));
                    
                    TestData.Modalities{1}.FeatureVectors =...
                        TestData.FeatureVectors(1:size(obj.Modalities{1}.SOM.codebook,2),:);
                    
                else
                    
                    error('Error: Numeric input should be %d-dimensional!',...
                          size(obj.Modalities{1}.SOM.codebook,2));
                    
                end
                
            else
                
                TestData = TestDataIn;
                
            end                                    
            
            if isempty(obj.Modalities{1}.ClassLabels)                            

                %% CLUSTER OUTPUT MODALITY ------------------------------------
                %--------------------------------------------------------------
                fprintf('\nClustering OUTPUT modality...');
                obj.Modalities{2} = obj.Modalities{2}.cluster();
                
                
                %% USE OUT-MODALITY CLUSTERS AND CROSS-MODAL MAPPINGS TO LABEL
                %  IN-MODALITY NODES...
                %--------------------------------------------------------------
                % Calculate in-modality node weights for each out-modality
                % cluster...
                for iOutCluster = 1:obj.Modalities{2}.Clustering.OptimalK
                    InNodeToOutClusterWeights(:,iOutCluster) =...
                        sum(obj.crossMapping.Weights(:,obj.Modalities{2}.Clustering.Labels{obj.Modalities{2}.Clustering.OptimalK} == iOutCluster),2) ./...
                            sum(obj.Modalities{2}.Clustering.Labels{obj.Modalities{2}.Clustering.OptimalK} == iOutCluster);
                end

                % Ne1 = som_unit_neighs(obj.Modalities{1}.SOM.topol);

                % Use these weights to label the in-modality nodes...
                for iInNode = 1:size(obj.Modalities{1}.SOM.codebook,1)

                    % Old way using SOM neighbours...
                    % Neighs = find(Ne1(iInNode,:));

                    for iOutCluster = 1:size(InNodeToOutClusterWeights,2)
                        % NodeClusterScores(iInCluster) = sum(Weights(iOutCluster,Neighs));
                        NodeClusterScores(iOutCluster) = InNodeToOutClusterWeights(iInNode,iOutCluster);
                    end

                    [foo obj.Modalities{1}.ClassLabels(iInNode,:)] = max(NodeClusterScores);
                end
            end
            
            %% USE INDIVIDUAL NODE ALPHA VALUES (FROM OLVQ) TO ESTIMATE ---
            % THE RELEVANCE OF INDIVIDUAL NODES...
            %--------------------------------------------------------------
            % if ~isempty(obj.Modalities{1}.Alphas)
            %     NodeRelevances = 1  - obj.Modalities{1}.Alphas;
            %     Codebook = obj.Modalities{1}.SOM.codebook(NodeRelevances >= mean(NodeRelevances(:)), :);
            %     ClassLabels = obj.Modalities{1}.ClassLabels(NodeRelevances >= mean(NodeRelevances(:)));
            % else
                NodeRelevances = ones(size(obj.Modalities{1}.ClassLabels));
                Codebook = obj.Modalities{1}.SOM.codebook;
                ClassLabels = obj.Modalities{1}.ClassLabels;
            % end
            
            %% CLASSIFY TEST VECTORS IN INPUT MODALITY --------------------
            %  Classify the in modality test vectors in terms of out
            %  modality clusters.
            %--------------------------------------------------------------            
            switch Method
                
                case 'node'
                    %% SET UP FEATURE SELECTION -------------------------------------
                    %----------------------------------------------------------------
                    % If LDA-based feature selection is requested, we do
                    % the calculations here a-posteriori (after
                    % training)...
                    if ~isempty(findstr(obj.feature_selection, 'lda'))
                        
                        for iClass = 1:max(ClassLabels(:))

                            ClassData = Codebook(ClassLabels == iClass, :);

                            % Just in case the cluster only contains
                            % one node...
                            if size(ClassData,1) == 1
                                ClassMeans(iClass,:) = ClassData;
                                ClassVars(iClass,:) = zeros(size(ClassData));
                            else
                                ClassMeans(iClass,:) = mean(ClassData);
                                ClassVars(iClass,:) = var(ClassData);
                            end
                        end

                        % Fisher Criterion =
                        %  (Between Class Variance)
                        % --------------------------
                        %  (Within Class Variance)
                        if any(sum(ClassVars) == 0)
                            FisherCriterion = var(ClassMeans);
                        else
                            FisherCriterion = var(ClassMeans) ./ sum(ClassVars);
                        end

                        % Watch out for nasty NaNs...
                        FisherCriterion(isnan(FisherCriterion)) = 0;

                        % Make the mask a weight distribution (unit norm)...
                        Mask = (FisherCriterion ./ norm(FisherCriterion,1))';

                        % Save the mask for later...
                        % obj.Modalities{1}.SOM.mask = Mask;
                    
                    % LDA-based feature selection based on node statistics
                    % gathered during training, as opposed to the node
                    % weight vectors (as above)...
                    elseif ~isempty(findstr(obj.feature_selection, 'nodestats'))
                        
                        % Some of the notation in the following is derived                        
                        % from equations (18) in "Multivariate Online Kernel
                        % Density Estimation" by Kristan et al.
                                                                        
                        for iClass = 1:max(ClassLabels(:))
                            
                            ClassNodeIndices = find(ClassLabels==iClass);
                            
                            % Sum accuracy weights...
                            w_j = sum(obj.Modalities{1}.AccuracyHist(ClassNodeIndices));                                                        
                            
                            % Calculate the class mean...
                            mu_j = 0;
                            for iNode = 1:size(ClassNodeIndices,1)
                                
                                mu_i = obj.Modalities{1}.NodeStats{ClassNodeIndices(iNode)}.mean();
                                
                                if isnan(mu_i)
                                    mu_i = 0;
                                end
                                
                                mu_j = mu_j + (obj.Modalities{1}.AccuracyHist(ClassNodeIndices(iNode)) *...
                                               mu_i);
                            end                                    
                            mu_j = w_j^(-1) * mu_j;
                            
                            % Calculate the class variance...
                            sig_j = 0;
                            for iNode = 1:size(ClassNodeIndices,1)
                                
                                mu_i = obj.Modalities{1}.NodeStats{ClassNodeIndices(iNode)}.mean();
                                sig_i = obj.Modalities{1}.NodeStats{ClassNodeIndices(iNode)}.var();
                                
                                if isnan(mu_i)
                                    mu_i = 0;
                                end
                                
                                if isnan(sig_i)
                                    sig_i = 0;
                                end
                                
                                sig_j = sig_j + (obj.Modalities{1}.AccuracyHist(ClassNodeIndices(iNode)) *...
                                                 (sig_i + mu_i.^2));
                            end
                            sig_j = (w_j.^(-1) * sig_j) - mu_j.^2;
                            
                            % Save...
                            if isnan(mu_j)
                                ClassMeans(iClass,:) = zeros(1,size(obj.Modalities{1}.SOM.codebook,2));
                            else                                
                                ClassMeans(iClass,:) = mu_j;
                            end
                            
                            if isnan(mu_j)
                                ClassVars(iClass,:) = zeros(1,size(obj.Modalities{1}.SOM.codebook,2));
                            else
                                ClassVars(iClass,:) = sig_j;
                            end
                                                        
                        end
                        
                        % Fisher Criterion =
                        %  (Between Class Variance)
                        % --------------------------
                        %  (Within Class Variance)
                        if any(sum(ClassVars) == 0)
                            FisherCriterion = var(ClassMeans);
                        else
                            FisherCriterion = var(ClassMeans) ./ sum(ClassVars);
                        end

                        % Watch out for nasty NaNs...
                        FisherCriterion(isnan(FisherCriterion)) = 0;

                        % Make the mask a weight distribution (unit norm)...
                        Mask = (FisherCriterion ./ norm(FisherCriterion,1))';

                        % Save the mask for later...
                        % obj.Modalities{1}.SOM.mask = Mask;
                        
                    % Otherwise, just make sure the feature mask is
                    % normalized...
                    else
                        Mask = obj.Modalities{1}.SOM.mask ./ norm(obj.Modalities{1}.SOM.mask,1);
                    end
                    
                    %% CLASSIFY TEST VECTORS NODE-WISE IN INPUT MODALITY ------------
                    %----------------------------------------------------------------
                    % For hard feature selection, we pick out the most
                    % relevant features based on the mean feature weight
                    % and re-normalize...
                    if ~isempty(findstr(obj.feature_selection, 'hard'))
                        
                        if isempty(obj.feature_selection_max) || ischar(obj.feature_selection_max)
                            Mask(Mask < mean(Mask)) = 0;
                        else
                            [Foo Bar] = sort(Mask,'descend');
                            Mask(Bar(obj.feature_selection_max+1:end)) = 0;
                        end
                        
                        Mask = Mask ./ norm(Mask,1);
                        [TestDataInMatches TestData.Modalities{1}.NormedFeatureVectors] =...
                                    obj.Modalities{1}.classify(TestData.Modalities{1},...
                                                               'codebook', Codebook,...
                                                               'mask', Mask);
                    
                    % Query-based exponential feature weighting...
                    elseif ~isempty(findstr(obj.feature_selection, 'exp'))
                        
                        % First off, we have to find the BMUs for the test
                        % data...
                        [TestDataBMUs TestData.Modalities{1}.NormedFeatureVectors] =...
                                    obj.Modalities{1}.classify(TestData.Modalities{1},...
                                                               'codebook', Codebook,...
                                                               'mask', Mask,...
                                                               'whichbmus', 'all');
                                                           
                        % Next, based on the class of the first BMU of each
                        % test query, we must determine the closest BMU of
                        % an opposing class, so that we can calculate their
                        % seperating hyperplane.  Then the distance between
                        % the test query and the seperating hyperplane can
                        % be used to locally re-calculate the feature
                        % weights.
                        for iTestData = 1:size(TestDataBMUs,1)
                            
                            BMUClass1 = TestDataBMUs(iTestData,1);
                            
                            % Find the closest BMU of a different class...
                            for iBMU = 2:size(TestDataBMUs,2)                                
                                BMUClass2 = TestDataBMUs(iTestData, iBMU);
                                if obj.Modalities{1}.ClassLabels(BMUClass2) ~=...
                                        obj.Modalities{1}.ClassLabels(BMUClass1)
                                    break;
                                end                                
                            end
                            
                            % Let's calculate the distance between the test
                            % query point and the hyperplane seperating
                            % the BMUs of the different classes...
                            
                            % BMU vectors...
                            a = obj.Modalities{1}.SOM.codebook(BMUClass1,:)';
                            b = obj.Modalities{1}.SOM.codebook(BMUClass2,:)';
                            
                            % Point on the hyperplane...
                            q = a + ((b - a)/2);
                            
                            % Query point...
                            p = TestData.Modalities{1}.NormedFeatureVectors(:,iTestData);
                            
                            % Norm to the hyperplane...
                            n = (b - a);
                            
                            % Distance from point p to the hyperplane...
                            dist = abs(dot((p - q), n)) / norm(n,1);
                            
                            % Use this to calculate the exponential
                            % weighting factor...
                            C = 1 / dist;
                            
                            % Calculate the feature weights for this
                            % query...
                            QueryMask = exp(C * Mask) ./ sum(exp(C * Mask));
                                                        
                            % Classify
                            QueryMask(QueryMask < mean(QueryMask)) = 0;
                            QueryMask = QueryMask ./ norm(QueryMask,1);
                            TestDataInMatches(iTestData) =...
                                obj.Modalities{1}.classify(TestData.Modalities{1}.FeatureVectors(:,iTestData),...
                                                           'codebook', Codebook,...
                                                           'mask', QueryMask,...
                                                           'whichbmus', 'best');
                                                       
                            Mask = QueryMask;
                            
                        end
                        
                    % Fuzzy feature weighting...
                    elseif ~isempty(findstr(obj.feature_selection, 'fuzzy'))
                        
                        [TestDataInMatches TestData.Modalities{1}.NormedFeatureVectors] =...
                                    obj.Modalities{1}.classify(TestData.Modalities{1},...
                                                               'codebook', Codebook,...
                                                               'mask', Mask);
                                                           
                        
                    
                    % Otherwise, ignore feature weights...
                    else
                        [TestDataInMatches TestData.Modalities{1}.NormedFeatureVectors] =...
                                    obj.Modalities{1}.classify(TestData.Modalities{1},...
                                                               'codebook', Codebook);
                                                           
                        Mask = nan(size(obj.Modalities{1}.SOM.mask));
                        
                    end
                    
                    TestData.Results.InToOutClassification = ClassLabels(TestDataInMatches')';

                case 'cluster'
                    %% CLASSIFY TEST VECTORS CLUSTER-WISE IN INPUT MODALITY ---------
                    %----------------------------------------------------------------
                    [TestDataInMatches TestData.Modalities{1}.NormedFeatureVectors] =...
                        obj.Modalities{1}.classify(TestData.Modalities{1}, 'clusterwise');

                    for i = 1:length(TestDataInMatches)

                        for j = 1:obj.Modalities{2}.Clustering.OptimalK

                            % Find the in modality winning cluster nodes for
                            % this test sample...
                            InWinningClusterNodes =...
                                find(TestDataInMatches(i,obj.Modalities{1}.KNN_max_k) ==...
                                    obj.Modalities{1}.Clustering.Labels{obj.Modalities{1}.Clustering.OptimalK});

                            if hebbian
                                % Sum the weights over all the nodes in the in
                                % modality cluster...
                                Weights = sum(obj.crossMapping.Weights(InWinningClusterNodes,:),1);

                                % Calculate the total weight score between the
                                % in modality cluster and the current out modality
                                % cluster (j)
                                TestData.Results.InToOutScores(i,j) =...
                                        sum(Weights(obj.Modalities{2}.Clustering.Labels{obj.Modalities{2}.Clustering.OptimalK} == j)) /...
                                           sum(obj.Modalities{2}.Clustering.Labels{obj.Modalities{2}.Clustering.OptimalK} == j);
                            else

                                % Find most recent co-occurences...
                                RecentCoOccurences = obj.crossMapping.CoOccurences(end-100-1:end,:);

                                % Find the ones that match the in-winning
                                % cluster...                                
                                [RecentCoOccurenceMatches InWinningClusterMatches] = find(repmat(RecentCoOccurences(:,1),1,size(InWinningClusterNodes,1)) ==...
                                    repmat(InWinningClusterNodes', size(RecentCoOccurences,1),1));

                                % Find the corresponding cluster
                                TestData.Results.InToOutScores(i,j) =...
                                    sum(obj.Modalities{2}.Clustering.Labels{obj.Modalities{2}.Clustering.OptimalK}(RecentCoOccurences(RecentCoOccurenceMatches,2)) == j);
                            end
                        end

                        [foo TestData.Results.InToOutClassification(i)] =...
                                max(TestData.Results.InToOutScores(i,:));

                    end
                    
                case 'svm'
                    %% TRAIN AN SVM CLASSIFIER IN THE IN-MODALITY -----------------
                    %--------------------------------------------------------------
                    Classifier =...
                        svmtrain(Codebook,...
                                 ClassLabels,...
                                 'Kernel_Function', 'rbf');

                    %% CONVERT & NORMALIZE TEST DATA ------------------------------
                    %--------------------------------------------------------------
                    SOMTestData = som_data_struct(TestData.Modalities{1}.FeatureVectors');

                    % Normalize...
                    SOMTestData = som_normalize(SOMTestData, obj.Modalities{1}.Norm);
                    
                    % Record normalized feature vectors...
                    TestData.Modalities{1}.NormedFeatureVectors = SOMTestData.data';

                    %% CLASSIFICATION USING THE SVM CLASSIFIER --------------------
                    %--------------------------------------------------------------
                    TestData.Results.InToOutClassification =...
                        svmclassify(Classifier, SOMTestData.data);

                    TestData.Results.InToOutClassification = TestData.Results.InToOutClassification';
                    
                case 'lda'
                    %% CONVERT & NORMALIZE TEST DATA ------------------------------
                    %--------------------------------------------------------------
                    SOMTestData = som_data_struct(TestData.Modalities{1}.FeatureVectors');

                    % Normalize...
                    SOMTestData = som_normalize(SOMTestData, obj.Modalities{1}.Norm);
                    
                    % Record normalized feature vectors...
                    TestData.Modalities{1}.NormedFeatureVectors = SOMTestData.data';

                    %% CLASSIFICATION USING THE LDA CLASSIFIER --------------------
                    %--------------------------------------------------------------
                    TestData.Results.InToOutClassification =...
                        classify(SOMTestData.data, Codebook, ClassLabels);

                    TestData.Results.InToOutClassification = TestData.Results.InToOutClassification';
                    
                case 'lvq'
                    [TestDataInMatches TestData.Modalities{1}.NormedFeatureVectors] =...
                        obj.Modalities{1}.classify(TestData.Modalities{1});
                    
                    TestData.Results.InToOutClassification = obj.Modalities{1}.LVQLabels(TestDataInMatches)';
                    
            end
            
            % Save the results in the TestData struct...
            TestData.Results.InToOutClassification = ClassLabels(TestDataInMatches')';

            % If we were using the internal test data in the
            % class, we should overwrite it with the new results...
            if using_internal_testdata
                obj.TestData = TestData;
            end
                
        end
        
        
        %% ------- *** GROUND TRUTH CLASSIFY *** --------------------------
        %******************************************************************
        % Classify test samples in terms of output modality clusters, then
        % match that classification to ground truth labels.
        %******************************************************************
        function [TestData Mask] = gtclassify(obj, varargin)
            
            % The TestData struct...
            TestData = [];
            
            % A flag that lets us know if we're using the TestData struct
            % in the object, or test data that was passed as an argument...
            using_internal_testdata = false;
            
            % Defaults...
            display = false;
            
            %% CHECK ARGUMENTS --------------------------------------------
            %--------------------------------------------------------------
            % varargin
            i=1; 
            while i<=length(varargin), 
              argok = 1; 
              if ischar(varargin{i}), 
                switch varargin{i}, 
                  % argument IDs
                  case {'data', 'testdata'}, i=i+1; TestDataIn = varargin{i};
                      
                  otherwise, argok=0; 
                end
              elseif isstruct(varargin{i})
                  TestDataIn = varargin{i};
              else
                argok = 0; 
              end
              if ~argok, 
                disp(['BiModalLearner.gtclassify(): Ignoring invalid argument #' num2str(i+1)]); 
              end
              i = i+1; 
            end
            
             %% SET UP TEST DATA ------------------------------------------
            %--------------------------------------------------------------
            if isempty(TestDataIn)
               TestDataIn = obj.TestData;
               using_internal_testdata = true;
            end
            
            %% CLASSIFY IN TERMS OF OUTPUT MOD CLUSTERS -------------------
            %--------------------------------------------------------------
            % if isnumeric(TestData) || ~isfield(TestData, 'Results') || ~isfield(TestData.Results, 'InToOutClassification')
            [TestData Mask] = obj.classify('data', TestDataIn);
            
            %% LABEL THE TRAINED SOMS -------------------------------------
            %--------------------------------------------------------------
            if ~obj.Modalities{1}.SOM_is_labeled || ~obj.Modalities{2}.SOM_is_labeled
                obj.gtlabelsoms();
            end
            
            %% MATCH Mod2 CLUSTERS TO GROUND TRUTH ----------------
            % Count the number of ground-truth class labels
            % from the training data that appear in
            % each of the clusters in the Mod2 SOM...
            %--------------------------------------------------------------
            nClusters = obj.Modalities{2}.Clustering.OptimalK;
    
            nGroundTruths = length(obj.TrainingData.Modalities{2}.GroundTruthLabelIndices);
            GroundTruthLabels =...
                obj.TrainingData.Modalities{2}.ClassNames(obj.TrainingData.Modalities{2}.GroundTruthLabelIndices);

            % Row = Cluster
            % Col = Ground truth
            ClusterGroundTruthLabelCounts = zeros(nClusters,nGroundTruths);

            for i = 1:nClusters

                LabelsInThisCluster = obj.Modalities{2}.SOM.labels(obj.Modalities{2}.Clustering.Labels{nClusters} == i, :);

                for j = 1:nGroundTruths                
                    for k = 1:numel(LabelsInThisCluster)

                        if (findstr(LabelsInThisCluster{k}, GroundTruthLabels{j}) == 1)
                        ClusterGroundTruthLabelCounts(i,j) = ClusterGroundTruthLabelCounts(i,j) +...
                            str2double(regexp(LabelsInThisCluster{k}, '(\d*)', 'match'));
                        end
                    end
                end
            end

            % Create SOM cluster to ground truth mapping...
            for i = 1:nClusters
                [LabelCount Cluster_To_GT_Mapping(i,:)] = max(ClusterGroundTruthLabelCounts(i,:));
            end
            
            %% MATCH Mod1-CLASSIFIED TEST DATA TO Mod2-CLUSTER-GROUND-TRUTH --
            % Get the class labels for the
            % Mod1-classified test data
            % from the Mod2 clustering...
            %-----------------------------------------------------------------------
            for i = 1:length(TestData.Results.InToOutClassification)
                ClassifiedTestData_To_ClusterGT_Matches(i,1) =...
                    Cluster_To_GT_Mapping(TestData.Results.InToOutClassification(i));
            end
            
            %% SAVE THE RESULTS IN THE TESTDATA STRUCT ---------------------
            %--------------------------------------------------------------
            TestData.Results.GroundTruthClassification =...
                GroundTruthLabels(ClassifiedTestData_To_ClusterGT_Matches);
            
        end
        
  
        %% ------- *** EVALUATE *** ---------------------------------------
        %******************************************************************
        %******************************************************************
        function TestData = evaluate(obj, varargin)
            
            % The TestData struct...
            TestData = [];
            
            % A flag that lets us know if we're using the TestData struct
            % in the object, or test data that was passed as an argument...
            using_internal_testdata = false;
            
            % Defaults...
            display = false;
            
            %% CHECK ARGUMENTS --------------------------------------------
            %--------------------------------------------------------------
            % varargin
            i=1; 
            while i<=length(varargin), 
              argok = 1; 
              if ischar(varargin{i}), 
                switch varargin{i}, 
                  % argument IDs
                  case {'data', 'testdata'}, TestData = varargin{i}; i=i+1;
                  case 'display', i=i+1; DisplaySample = varargin{i}; display = true;
                      
                  otherwise, argok=0; 
                end
              elseif isstruct(varargin{i})
                  TestData = varargin{i};
              else
                argok = 0; 
              end
              if ~argok, 
                disp(['BiModalLearner.evaluate(): Ignoring invalid argument #' num2str(i+1)]); 
              end
              i = i+1; 
            end
            
             %% SET UP TEST DATA -------------------------------------------
            %--------------------------------------------------------------
            if isempty(TestData)
               TestData = obj.TestData;
               using_internal_testdata = true;
            end
            
            %% LABEL THE TRAINED SOMS -------------------------------------
            %--------------------------------------------------------------
            if ~obj.Modalities{1}.SOM_is_labeled || ~obj.Modalities{2}.SOM_is_labeled
                obj.gtlabelsoms();
            end
            
            
            %% MATCH Mod2 TEST VECTORS TO Mod2 CLUSTERS ---
            % Use KNN to determine what cluster the Mod2 vector
            % parts of the test samples belong to...
            %--------------------------------------------------------------
            TestDataMod2_To_Cluster_Matches = obj.Modalities{2}.classify(TestData.Modalities{2}, 'clusterwise');
                                    
            
            %% MATCH Mod2 CLUSTERS TO GROUND TRUTH ----------------
            % Count the number of ground-truth class labels
            % from the training data that appear in
            % each of the clusters in the Mod2 SOM...
            %--------------------------------------------------------------
            nClusters = obj.Modalities{2}.Clustering.OptimalK;
    
            nGroundTruths = length(obj.TrainingData.Modalities{2}.GroundTruthLabelIndices);
            GroundTruthLabels =...
                obj.TrainingData.Modalities{2}.ClassNames(obj.TrainingData.Modalities{2}.GroundTruthLabelIndices);

            % Row = Cluster
            % Col = Ground truth
            ClusterGroundTruthLabelCounts = zeros(nClusters,nGroundTruths);

            for i = 1:nClusters

                LabelsInThisCluster = obj.Modalities{2}.SOM.labels(obj.Modalities{2}.Clustering.Labels{nClusters} == i, :);

                for j = 1:nGroundTruths                
                    for k = 1:numel(LabelsInThisCluster)

                        if (findstr(LabelsInThisCluster{k}, GroundTruthLabels{j}) == 1)
                        ClusterGroundTruthLabelCounts(i,j) = ClusterGroundTruthLabelCounts(i,j) +...
                            str2double(regexp(LabelsInThisCluster{k}, '(\d*)', 'match'));
                        end
                    end
                end
            end

            % Create SOM cluster to ground truth mapping...
            for i = 1:nClusters
                [LabelCount Cluster_To_GT_Mapping(i,:)] = max(ClusterGroundTruthLabelCounts(i,:));
            end
            
            %% MATCH Mod1-CLASSIFIED TEST DATA TO Mod2-CLUSTER-GROUND-TRUTH --
            % Get the class labels for the
            % Mod1-classified test data
            % from the Mod2 clustering...
            %-----------------------------------------------------------------------
            for i = 1:length(TestData.Results.InToOutClassification)
                ClassifiedTestData_To_ClusterGT_Matches(i,1) =...
                    Cluster_To_GT_Mapping(TestData.Results.InToOutClassification(i));
            end

            %% FIND THE TEST SAMPLE GROUND TRUTH ---------------------------
            %---------------------------------------------------------------
            [Test_Data_Ground_Truth foo] =...
                find(TestData.ClassLabels(obj.TrainingData.Modalities{1}.GroundTruthLabelIndices,:));

            % Test data ground truth repmat'd for all KNN k's up to KNN_max_k...
            Test_Data_Ground_Truth_KNN =...
                repmat(Test_Data_Ground_Truth, 1, obj.Modalities{2}.KNN_max_k);
            
            if display
                    TestSampleInBMUs = obj.Modalities{1}.classify(TestData.Modalities{1});
                    TestSampleOutBMUs = obj.Modalities{2}.classify(TestData.Modalities{2});
                    
                    WinningClusterNodes = find((obj.Modalities{2}.Clustering.Labels{obj.Modalities{2}.Clustering.OptimalK} ==...
                                            TestData.Results.InToOutClassification(DisplaySample)));
                                        
                    
                    % 'colourinclustermaps',
                    % TestSampleInBMUs(DisplaySample), 100,...
                
                    obj.display('cluster', 'out',...
                                'classlabels',...
                                'colourinnodeweights', TestSampleInBMUs(DisplaySample),...
                                'markoutnode', TestSampleOutBMUs(DisplaySample),...
                                'markoutclusternodes', WinningClusterNodes,...
                                'groundtruthlabels');
            end
            
            
            %% CALCULATE EVALUATION SCORES ---------------------------------
            %---------------------------------------------------------------
            %---------------------------------------------------------------

            % Tools to work with from the above section...
            %
            %   TestDataMod2_To_Cluster_Matches
            %   Cluster_To_GT_Mapping
            %   ClassifiedTestData_To_ClusterGT_Matches
            %   Test_Data_Ground_Truth
            %   Test_Data_Ground_Truth_KNN
            %

            %% DOES THE Mod2 VECTOR CLUSTER-MATCH REFLECT THE GROUND TRUTH? -----
            % i.e. ignoring the Mod1 vector and finding the best
            % matching cluster for the Mod2 vector (using KNN), does the
            % test sample ground truth correspond to the cluster ground truth?
            %----------------------------------------------------------------------

            % Find the cluster ground truth for the Mod2-to-cluster KNN
            % matches...
            Mod2_To_Cluster_GT = Cluster_To_GT_Mapping(TestDataMod2_To_Cluster_Matches);

            % Test sample GT to Cluster GT matches for all KNN k's up to KNN_max_k......
            TestData.Results.Mod2_To_Cluster_GT_Matches =...
                (Test_Data_Ground_Truth_KNN == Mod2_To_Cluster_GT);

            % Scores for all KNN k's up to KNN_max_k......
            TestData.Results.Mod2_To_Cluster_GT_Scores =...
                sum(TestData.Results.Mod2_To_Cluster_GT_Matches, 1);

            % Percentages for all KNN k's up to KNN_max_k......
            TestData.Results.Mod2_To_Cluster_GT_Percent =...
                TestData.Results.Mod2_To_Cluster_GT_Scores /...
                    size(TestData.Modalities{2}.FeatureVectors,2);

            % Best KNN k...
            [TestData.Results.Mod2_To_Cluster_GT_Best_Percent...
                TestData.Results.Mod2_To_Cluster_GT_Best_KNN_k] =...
                max(TestData.Results.Mod2_To_Cluster_GT_Percent);

            %% DOES THE Mod1 VECTOR CLASSIFICATION REFLECT THE GROUND TRUTH? ---
            % i.e. ignoring the Mod2 vector and classifying the Mod1
            % vector, does the test sample ground truth correspond to the cluster
            % ground truth?
            %----------------------------------------------------------------------

            % Test sample GT to Cluster GT matches...
            TestData.Results.Mod1_To_Cluster_GT_Matches =...
                (Test_Data_Ground_Truth == ClassifiedTestData_To_ClusterGT_Matches);

            % Score...
            TestData.Results.Mod1_To_Cluster_GT_Score =...
                sum(TestData.Results.Mod1_To_Cluster_GT_Matches, 1);

            % Percentage...
            TestData.Results.Mod1_To_Cluster_GT_Percent =...
                TestData.Results.Mod1_To_Cluster_GT_Score /...
                    size(TestData.Modalities{1}.FeatureVectors,2);

            %% DOES CLASSIFIER SELECT CORRECT CLUSTER BASED ON ITS TRAINING? ------
            % i.e. does it, by classifying using the Mod1 test vector, select
            % the Mod2 cluster that best matches the Mod2 test vector?
            %----------------------------------------------------------------------
            % Matches 
            TestData.Results.Mod1_To_Cluster_Matches =...
                repmat(TestData.Results.InToOutClassification', 1, obj.Modalities{2}.KNN_max_k) ==...
                    TestDataMod2_To_Cluster_Matches;

            % Scores...
            TestData.Results.Mod1_To_Cluster_Scores =...
                sum(TestData.Results.Mod1_To_Cluster_Matches, 1);

            % Percentages...
            TestData.Results.Mod1_To_Cluster_Percent =...
                TestData.Results.Mod1_To_Cluster_Scores /...
                    size(TestData.Modalities{1}.FeatureVectors,2);

            % Best KNN k...
            [TestData.Results.Mod1_To_Cluster_Best_Percent...
                TestData.Results.Mod1_To_Cluster_Best_KNN_k] =...
                max(TestData.Results.Mod1_To_Cluster_Percent);

            %% IF CORRECT, DOES THAT CLUSTER CORRESPOND TO THE GROUND TRUTH? ------
            % i.e. ground truth based on no. of ground truth labels that gather
            % in the SOM clusters during training (see Cluster_To_GT_Mapping
            % calculation above)...
            %----------------------------------------------------------------------

            Foo = zeros(size(TestData.Results.Mod1_To_Cluster_Matches));
            Bar = repmat(TestData.Results.InToOutClassification, 1, obj.Modalities{2}.KNN_max_k);

            Foo(TestData.Results.Mod1_To_Cluster_Matches) =...
                Cluster_To_GT_Mapping(Bar(TestData.Results.Mod1_To_Cluster_Matches));

            % Matches for all KNN k's up to KNN_max_k...
            TestData.Results.Mod1_To_Cluster_Matches_Corresponding_To_GT =...
                (Test_Data_Ground_Truth_KNN == Foo);

            % Scores...
            TestData.Results.Mod1_To_Cluster_Scores_Corresponding_To_GT =...
                sum(TestData.Results.Mod1_To_Cluster_Matches_Corresponding_To_GT, 1);

            % Percentages...
            TestData.Results.Mod1_To_Cluster_Percent_Corresponding_To_GT =...
                TestData.Results.Mod1_To_Cluster_Scores_Corresponding_To_GT /...
                    size(TestData.Modalities{1}.FeatureVectors,2);

            % Best KNN k...
            [TestData.Results.Mod1_To_Cluster_Corresponding_To_GT_Best_Percent...
                TestData.Results.Mod1_To_Cluster_Corresponding_To_GT_Best_KNN_k] =...
                max(TestData.Results.Mod1_To_Cluster_Percent_Corresponding_To_GT);
            
            %% COLLATE MAIN EVALUATION SCORES -----------------------------
            %--------------------------------------------------------------
            %--------------------------------------------------------------
            % We use a KNN K of 1 for each of these...
            
            TestData.Results.Matches =...
                TestData.Results.Mod1_To_Cluster_Matches_Corresponding_To_GT(:,1);
            
            TestData.Results.Score =...
                TestData.Results.Mod1_To_Cluster_Scores_Corresponding_To_GT(1);
            
            TestData.Results.Percent =...
                TestData.Results.Mod1_To_Cluster_Percent_Corresponding_To_GT(1);
            
            % If we were using the internal test data in the
            % class, we should overwrite it with the new results...
            if using_internal_testdata
                obj.TestData = TestData;
            end
    
        end                
        
        %% ------- *** DISPLAY *** ----------------------------------------
        %******************************************************************
        %******************************************************************
        function display(obj, varargin)
            
            % Default settings...
            Cluster = 'no';
            ClusterInModality = false;
            ShowInModalityClassLabels = false;
            ClusterOutModality = false;
            ColourInNodeMaps = false;
            ColourOutNodeMaps = false;
            ColourInClusterMaps = false;
            ColourGroundTruthMaps = false;
            ColourGroundTruthMapHistory = false;
            ColourInNodeWeights = false;
            ColourOutNodeWeights = false;            
            GroundTruthLabels = false;
            NodeByNode = false;
            ShowMappings = true;
            
            % Loop through arguments...
            i = 1;
            while i <= length(varargin), 
                argok = 1; 
                if ischar(varargin{i}), 
                    switch lower(varargin{i}), 
                        case {'cluster', 'clusters'}, i=i+1; Cluster = varargin{i};
                        case {'classlabels'}, ShowInModalityClassLabels = true;
                        case 'k', i=i+1; InKMeansK = varargin{i};
                                         OutKMeansK = varargin{i};
                        case 'ink', i=i+1; InKMeansK = varargin{i};
                        case 'outk', i=i+1; OutKMeansK = varargin{i};
                        case 'colouroutclustermaps', ColourOutClusterMaps = true;
                        case 'colourinnodemaps', ColourInNodeMaps = true;
                        case 'colouroutnodemaps', ColourOutNodeMaps = true;
                        case 'colourinclustermaps',
                            i=i+1; InNode = varargin{i};
                            i=i+1; LastNCoOccurences = varargin{i};
                            ColourInClusterMaps = true;
                        case {'colourgroundtruthmaps' 'colourgtmaps'}, ColourGroundTruthMaps = true;
                        case {'colourgtmapshistory' 'colourgtmaphistory'}, ColourGroundTruthMapHistory = true;
                        case {'colourinnodeweights', 'innode'}, i=i+1;
                            InNode = varargin{i};
                            ColourInNodeWeights = true;
                        case {'colouroutnodeweights', 'outnode'}, i=i+1;
                            OutNode = varargin{i};
                            ColourOutNodeWeights = true;
                        case 'markinnode', i=i+1; InNodeToMark = varargin{i};
                        case 'markoutnode', i=i+1; OutNodeToMark = varargin{i};
                        case 'markoutclusternodes', i=i+1; WinningClusterNodes = varargin{i};
                        case 'groundtruthlabels', GroundTruthLabels = true;
                        case 'nodebynode', NodeByNode = true;
                        case {'nomappings', 'nomaps'}, ShowMappings = false;
                        
                        otherwise
                            argok = 0;
                    end
                else
                    argok = 0;
                end

                if ~argok, 
                    disp(['BiModalLearner.display(): Ignoring invalid argument #' num2str(i)]);
                    % fprintf(UsageMessage);
                end

                i = i + 1;
            end
            
            % Do we need to cluster over the modalities?
            switch lower(Cluster)
                case {'in', 'inmodality'}, ClusterInModality = true;
                case {'out', 'outmodality'}, ClusterOutModality = true;
                case {'both'}, ClusterInModality = true;
                               ClusterOutModality = true;
            end
            
            %% DRAW THE IN MODALITY... ------------------------------------
            %--------------------------------------------------------------
            if ClusterInModality
                % Cluster the map if necessary...
                if isempty(obj.Modalities{1}.Clustering)
                	obj.Modalities{1} = obj.Modalities{1}.cluster();
                end
                
                % Display the map...
                CrossMod_som_cplane(obj.Modalities{1}.SOM.topol.lattice,...
                                    obj.Modalities{1}.SOM.topol.msize,...
                                    obj.Modalities{1}.Clustering.Labels{obj.Modalities{1}.Clustering.OptimalK}, 1, [1 1 1]);
                                
                % Change colormap...
                % colormap(jet(KMeansK));
                
            elseif ShowInModalityClassLabels
                % Display the map...
                CrossMod_som_cplane(obj.Modalities{1}.SOM.topol.lattice,...
                                    obj.Modalities{1}.SOM.topol.msize,...
                                    obj.Modalities{1}.ClassLabels', 1, [1 1 1]);                
            else
                % Get U-Matrix...
                U = som_umat(obj.Modalities{1}.SOM);
                
                % Display the map...
                CrossMod_som_cplane([obj.Modalities{1}.SOM.topol.lattice, 'U'],...
                                    obj.Modalities{1}.SOM.topol.msize, U(:), 1, [1 1 1]);
            end
            
            hold on;
            
            %% DRAW THE OUT MODALITY... -----------------------------------
            %--------------------------------------------------------------
            if ClusterOutModality
                % Cluster the map if necessary...
                if isempty(obj.Modalities{2}.Clustering)
                	obj.Modalities{2} = obj.Modalities{2}.cluster();
                end
                
                % Display the map...
                CrossMod_som_cplane(obj.Modalities{2}.SOM.topol.lattice,...
                                    obj.Modalities{2}.SOM.topol.msize,...
                                    obj.Modalities{2}.Clustering.Labels{obj.Modalities{2}.Clustering.OptimalK}, 1, [1 1 10]);
                                
                % Change colormap...
                % colormap(jet(KMeansK));
            else
                % Get U-Matrix...
                U = som_umat(obj.Modalities{2}.SOM);
                
                % Display the map...
                CrossMod_som_cplane([obj.Modalities{2}.SOM.topol.lattice, 'U'],...
                                    obj.Modalities{2}.SOM.topol.msize, U(:), 1, [1 1 10]);
            end
            
            %    colormap(jet(i)), som_recolorbar % change colormap
                            
                            
            %% DISPLAY CROSS-MODAL MAPPINGS -------------------------------
            %--------------------------------------------------------------
            if ShowMappings
                
                inCoords = som_vis_coords(obj.Modalities{1}.SOM.topol.lattice, obj.Modalities{1}.SOM.topol.msize);
                outCoords = som_vis_coords(obj.Modalities{2}.SOM.topol.lattice, obj.Modalities{2}.SOM.topol.msize);

                
                if ColourInNodeMaps
                    UniqueInNodes = unique(obj.crossMapping.CoOccurences(:,1));

                    for i = 1:length(UniqueInNodes)

                        inMatches = obj.crossMapping.CoOccurences(obj.crossMapping.CoOccurences(:,1)==UniqueInNodes(i),1);
                        inMatchCoords = inCoords(inMatches,:);
                        outMatches = obj.crossMapping.CoOccurences(obj.crossMapping.CoOccurences(:,1)==UniqueInNodes(i),2);
                        outMatchCoords = outCoords(outMatches,:);
                        inZ = ones(1,size(inMatchCoords,1));
                        outZ = 10 * ones(1,size(inMatchCoords,1));

                        X = [inMatchCoords(:,1)'; outMatchCoords(:,1)'; nan(1,size(inMatchCoords,1))];
                        Y = [inMatchCoords(:,2)'; outMatchCoords(:,2)'; nan(1,size(inMatchCoords,1))];
                        Z = [inZ; outZ; nan(1,size(inMatchCoords,1))];

                        UniqueOutMatches = unique(outMatches);
                        RandR = rand;
                        RandG = rand;
                        RandB = rand;


                        if NodeByNode
                            for j = 1:length(UniqueOutMatches)
                                h = plot3( reshape( X, [1 3*size(X,2)]), reshape( Y, [1 3*size(Y,2)]), reshape( Z, [1 3*size(Z,2)]),...
                                           'color', [RandR RandG RandB],...
                                           'LineWidth', sum(outMatches == UniqueOutMatches(j)));
                            end
                            fprintf('Press a key to see the next node mappings...');
                            pause;
                            clf(h, 'reset');
                        else
                            plot3( reshape( X, [1 3*size(X,2)]), reshape( Y, [1 3*size(Y,2)]), reshape( Z, [1 3*size(Z,2)]), 'color', [rand rand rand] );
                        end
                    end

                elseif ColourOutNodeMaps
                    UniqueOutNodes = unique(obj.crossMapping.CoOccurences(:,2));

                    for i = 1:length(UniqueOutNodes)

                        inMatchCoords = inCoords(obj.crossMapping.CoOccurences(find(obj.crossMapping.CoOccurences(:,2)==UniqueOutNodes(i)),1),:);
                        outMatchCoords = outCoords(obj.crossMapping.CoOccurences(find(obj.crossMapping.CoOccurences(:,2)==UniqueOutNodes(i)),2),:);
                        inZ = ones(1,size(inMatchCoords,1));
                        outZ = 10 * ones(1,size(inMatchCoords,1));

                        X = [inMatchCoords(:,1)'; outMatchCoords(:,1)'; nan(1,size(inMatchCoords,1))];
                        Y = [inMatchCoords(:,2)'; outMatchCoords(:,2)'; nan(1,size(inMatchCoords,1))];
                        Z = [inZ; outZ; nan(1,size(inMatchCoords,1))];

                        if NodeByNode
                            h = plot3( reshape( X, [1 3*size(X,2)]), reshape( Y, [1 3*size(Y,2)]), reshape( Z, [1 3*size(Z,2)]), 'color', [rand rand rand] );
                            fprintf('Press a key to see the next node mappings...');
                            pause;
                            clf(h, 'reset');
                        else
                            plot3( reshape( X, [1 3*size(X,2)]), reshape( Y, [1 3*size(Y,2)]), reshape( Z, [1 3*size(Z,2)]), 'color', [rand rand rand] );
                        end
                    end
                    
                elseif ColourInNodeWeights
                    
                    inMatchCoords = repmat(inCoords(InNode,:), size(inCoords,1),1);
                    outMatchCoords = outCoords;
                    inZ = ones(1,size(inMatchCoords,1));
                    outZ = 10 * ones(1,size(inMatchCoords,1));
                    
                    X = [inMatchCoords(:,1)'; outMatchCoords(:,1)'; nan(1,size(inMatchCoords,1))];
                    Y = [inMatchCoords(:,2)'; outMatchCoords(:,2)'; nan(1,size(inMatchCoords,1))];
                    Z = [inZ; outZ; nan(1,size(inMatchCoords,1))];
                    
                    LineWeights = 10 * (obj.crossMapping.Weights(InNode,:) / max(obj.crossMapping.Weights(InNode,:)));
                    
                    % plot3( reshape( X, [1 3*size(X,2)]), reshape( Y, [1 3*size(Y,2)]), reshape( Z, [1 3*size(Z,2)]), 'r', 'LineWidth', ones(100));
                    for i = 1:length(X)
                        if LineWeights(i) > 0
                            plot3( X(1:2,i), Y(1:2,i), Z(1:2,i), 'color', 'r', 'LineWidth', LineWeights(i));
                        end
                    end
                    
                elseif ColourInClusterMaps
                    
                    InWinningCluster = obj.Modalities{1}.Clustering.Labels{obj.Modalities{1}.Clustering.OptimalK}(InNode);
                    
                    InWinningClusterNodes = find((obj.Modalities{1}.Clustering.Labels{obj.Modalities{1}.Clustering.OptimalK} == InWinningCluster));
                    
                    % Find most recent co-occurences...
                    RecentCoOccurences = obj.crossMapping.CoOccurences(end-LastNCoOccurences-1:end,:);

                    % Find the ones that match the in-winning
                    % cluster...                                
                    [RecentCoOccurenceMatches InWinningClusterMatches] = find(repmat(RecentCoOccurences(:,1),1,size(InWinningClusterNodes,1)) ==...
                        repmat(InWinningClusterNodes', size(RecentCoOccurences,1),1));
                    
                    inMatchCoords = inCoords(RecentCoOccurences(RecentCoOccurenceMatches,1),:);
                    outMatchCoords = outCoords(RecentCoOccurences(RecentCoOccurenceMatches,2),:);
                    inZ = ones(1,size(inMatchCoords,1));
                    outZ = 10 * ones(1,size(inMatchCoords,1));

                    X = [inMatchCoords(:,1)'; outMatchCoords(:,1)'; nan(1,size(inMatchCoords,1))];
                    Y = [inMatchCoords(:,2)'; outMatchCoords(:,2)'; nan(1,size(inMatchCoords,1))];
                    Z = [inZ; outZ; nan(1,size(inMatchCoords,1))];

                    plot3( reshape( X, [1 3*size(X,2)]), reshape( Y, [1 3*size(Y,2)]), reshape( Z, [1 3*size(Z,2)]), 'r' );
                
                elseif ColourGroundTruthMaps || ColourGroundTruthMapHistory
                    
                    inMatchCoords = inCoords(obj.crossMapping.CoOccurences(:,1),:);
                    outMatchCoords = outCoords(obj.crossMapping.CoOccurences(:,2),:);
                    
                    Colours = ['y' 'm' 'c' 'r' 'g' 'b'];
                    iColour = 1;
                                
                    for i = 1:obj.TrainingData.Modalities{1}.nGroundTruths
                        
                        GroundTruthIndices = find(obj.TrainingData.Modalities{1}.ClassLabels(...
                                                        obj.TrainingData.Modalities{1}.GroundTruthLabelIndices(i),:));
                                                    
                        inZ = ones(1,size(GroundTruthIndices,2));
                        outZ = 10 * ones(1,size(GroundTruthIndices,2));
                        
                        X = [inMatchCoords(GroundTruthIndices,1)'; outMatchCoords(GroundTruthIndices,1)'; nan(1,size(find(GroundTruthIndices),2))];
                        Y = [inMatchCoords(GroundTruthIndices,2)'; outMatchCoords(GroundTruthIndices,2)'; nan(1,size(find(GroundTruthIndices),2))];
                        Z = [inZ; outZ; nan(1,size(find(GroundTruthIndices),2))];

                        iColour = iColour + 1;
                        if iColour > length(Colours)
                            iColour = 1;
                        end
                        
                        if ColourGroundTruthMapHistory
                            for j = 1:length(X)
                                LineWidth = ceil((5 / size(obj.TrainingData.Modalities{1}.ClassLabels,2)) * GroundTruthIndices(j));
                                plot3( X(1:2,j), Y(1:2,j), Z(1:2,j), Colours(iColour), 'LineWidth', LineWidth);
                            end
                        else
                            plot3( reshape( X, [1 3*size(X,2)]), reshape( Y, [1 3*size(Y,2)]), reshape( Z, [1 3*size(Z,2)]), Colours(iColour));
                        end
                    end
                    
                else
                    inMatchCoords = inCoords(obj.crossMapping.CoOccurences(:,1),:);
                    outMatchCoords = outCoords(obj.crossMapping.CoOccurences(:,2),:);
                    inZ = ones(1,size(inMatchCoords,1));
                    outZ = 10 * ones(1,size(inMatchCoords,1));

                    X = [inMatchCoords(:,1)'; outMatchCoords(:,1)'; nan(1,size(inMatchCoords,1))];
                    Y = [inMatchCoords(:,2)'; outMatchCoords(:,2)'; nan(1,size(inMatchCoords,1))];
                    Z = [inZ; outZ; nan(1,size(inMatchCoords,1))];

                    plot3( reshape( X, [1 3*size(X,2)]), reshape( Y, [1 3*size(Y,2)]), reshape( Z, [1 3*size(Z,2)]), 'r' );
                end
            end
            
            %% MARK NODES -------------------------------------------------
            %--------------------------------------------------------------
            if exist('InNodeToMark', 'var')
                plot3(inCoords(InNodeToMark, 1), inCoords(InNodeToMark, 2), 1, 'w*');
            end
            
            if exist('OutNodeToMark', 'var')
                plot3(inCoords(OutNodeToMark, 1), inCoords(OutNodeToMark, 2), 10 + 0.1, 'w*');
            end
            
            %% MARK WINNING CLUSTER NODES ---------------------------------
            %--------------------------------------------------------------
            if exist('WinningClusterNodes', 'var')
                for i = 1:length(WinningClusterNodes)
                    plot3(outCoords(WinningClusterNodes(i), 1), outCoords(WinningClusterNodes(i), 2), 10 + 0.1, 'g+');
                end
            end
            
            %% DISPLAY GROUND TRUTH LABELS --------------------------------
            %--------------------------------------------------------------
            if GroundTruthLabels
                
                % In modality...
                nGroundTruths = length(obj.TrainingData.Modalities{1}.GroundTruthLabelIndices);
                GroundTruthLabels =...
                    obj.TrainingData.Modalities{1}.ClassNames(obj.TrainingData.Modalities{1}.GroundTruthLabelIndices);
                
                % Layers of text for the labels...
                Layers = ones(1,size(obj.Modalities{1}.SOM.labels,1)) - 0.5;
                
                for iGroundTruth = 1:nGroundTruths
                    for inode = 1:size(obj.Modalities{1}.SOM.labels,1)
                        for j = 1:size(obj.Modalities{1}.SOM.labels(inode,:),2)

                            if (findstr(obj.Modalities{1}.SOM.labels{inode,j}, GroundTruthLabels{iGroundTruth}) == 1)
                                
                                foo = regexp(obj.Modalities{1}.SOM.labels{inode,j}, '\w*:\s|\W\d*\W', 'split');
                                ClassWords = regexp(foo{2}, '\W*', 'split');
                                
                                Text = '';
                                
                                for iWords = 1:length(ClassWords)
                                    Text = strcat(Text, upper(ClassWords{iWords}(1)));
                                end
                                
                                Text = strcat(Text, '(', regexp(obj.Modalities{1}.SOM.labels{inode,j}, '(\d*)', 'match'), ')');
                                
                                % We found a ground truth label in the
                                % input modality SOM...
%                                 if iGroundTruth == 1
%                                     Text = strcat('NR(', regexp(obj.Modalities{1}.SOM.labels{inode,j}, '(\d*)', 'match'), ')');
%                                 else
%                                     Text = strcat('R(', regexp(obj.Modalities{1}.SOM.labels{inode,j}, '(\d*)', 'match'), ')');
%                                 end
                                
                                text(inCoords(inode,1), inCoords(inode,2), Layers(inode), Text, 'color', 'w');
                                
                                % Next time draw it lower...
                                Layers(inode) = Layers(inode) - 0.5;
                                % Height = Height - 0.5;
                            end
                        end
                    end
                end
                
                % Out modality...
                nGroundTruths = length(obj.TrainingData.Modalities{2}.GroundTruthLabelIndices);
                GroundTruthLabels =...
                    obj.TrainingData.Modalities{2}.ClassNames(obj.TrainingData.Modalities{2}.GroundTruthLabelIndices);
                
                % Layers of text for the labels...
                Layers = (10 * ones(1,size(obj.Modalities{2}.SOM.labels,1))) + 0.5;
                
                for iGroundTruth = 1:nGroundTruths
                    for inode = 1:size(obj.Modalities{2}.SOM.labels,1)
                        for j = 1:size(obj.Modalities{2}.SOM.labels(inode,:),2)

                            if (findstr(obj.Modalities{2}.SOM.labels{inode,j}, GroundTruthLabels{iGroundTruth}) == 1)
                                
                                foo = regexp(obj.Modalities{2}.SOM.labels{inode,j}, '\w*:\s|\W\d*\W', 'split');
                                ClassWords = regexp(foo{2}, '\W*', 'split');
                                
                                Text = '';
                                
                                for iWords = 1:length(ClassWords)
                                    Text = strcat(Text, upper(ClassWords{iWords}(1)));
                                end
                                
                                Text = strcat(Text, '(', regexp(obj.Modalities{2}.SOM.labels{inode,j}, '(\d*)', 'match'), ')');
                                
%                                 if iGroundTruth == 1
%                                     Text = strcat('NR(', regexp(obj.Modalities{2}.SOM.labels{inode,j}, '(\d*)', 'match'), ')');
%                                 else
%                                     Text = strcat('R(', regexp(obj.Modalities{2}.SOM.labels{inode,j}, '(\d*)', 'match'), ')');
%                                 end
                                
                                text(outCoords(inode,1), outCoords(inode,2), Layers(inode), Text, 'color', 'r');
                                
                                % Next time draw it height...                               
                                Layers(inode) = Layers(inode) + 0.5;
                                
                            end
                            
                        end
                    end
                end
                
            end
            
            view(3);
        end
        
    end
        
    methods (Access = private)
        
        %% ------- *** LABEL MODALITY SOM MAPS *** ---------------
        %******************************************************************
        %******************************************************************
        
        function gtlabelsoms(obj, varargin)
            
            % Mod1 SOM...
            InSOMData = som_data_struct(obj.TrainingData.Modalities{1}.NormedFeatureVectors',...
                                        'comp_names', obj.TrainingData.Modalities{1}.FeatureNames,...
                                        'label_names', obj.TrainingData.Modalities{1}.ClassNames);
            
            % Add labels to the SOM training data struct...
            for i=1:size(InSOMData.labels, 1)
                tagIndex = 1;
                for j=1:length(obj.TrainingData.Modalities{1}.ClassNames)
                    if obj.TrainingData.Modalities{1}.ClassLabels(j,i) == 1
                        InSOMData.labels{i,tagIndex} =...
                            obj.TrainingData.Modalities{1}.ClassNames{j};
                        tagIndex = tagIndex + 1;
                    end
                end
            end
            
            obj.Modalities{1}.SOM = som_autolabel(obj.Modalities{1}.SOM, InSOMData, 'freq');
            obj.Modalities{1}.SOM_is_labeled = true;
            
            % Mod2 SOM...
            OutSOMData = som_data_struct(obj.TrainingData.Modalities{2}.NormedFeatureVectors',...
                                        'comp_names', obj.TrainingData.Modalities{2}.FeatureNames,...
                                        'label_names', obj.TrainingData.Modalities{2}.ClassNames);
            
            for i=1:size(OutSOMData.labels, 1)
                tagIndex = 1;
                for j=1:length(obj.TrainingData.Modalities{2}.ClassNames)
                    if obj.TrainingData.Modalities{2}.ClassLabels(j,i) == 1
                        OutSOMData.labels{i,tagIndex} =...
                            obj.TrainingData.Modalities{2}.ClassNames{j};
                        tagIndex = tagIndex + 1;
                    end
                end
            end
            
            obj.Modalities{2}.SOM = som_autolabel(obj.Modalities{2}.SOM, OutSOMData, 'freq');
            obj.Modalities{2}.SOM_is_labeled = true;
            
        end
        
    end
        
    methods (Static)

        %% ------- *** HELLINGER DISTANCE *** -----------------------------
        %------------------------------------------------------------------
        function dist = hellinger(Distribution1, Distribution2, varargin)
            
            % Normalize distributions...
            NormDistribution1 = Distribution1 / sum(Distribution1);
            NormDistribution2 = Distribution2 / sum(Distribution2);
            
            % Calculate Hellinger distance...
            dist = sqrt(sum((sqrt(NormDistribution1) - sqrt(NormDistribution2)).^2));
            
            % Normalize Hellinger distance...
            % dist = dist / sqrt(2);
        end
        
        %% ------- *** CHI-SQUARED DISTANCE *** ---------------------------
        %------------------------------------------------------------------
        function dist = chisquared(Distribution1, Distribution2, varargin)
            
            % Add 1% noise...
            maxDist1 = max(Distribution1);
            maxDist2 = max(Distribution2);
            NormDistribution1 = Distribution1 +...
                                rand(size(Distribution1)) * 0.01 * max(maxDist1, xor(maxDist1, 1));
            NormDistribution2 = Distribution2 +...
                                rand(size(Distribution2)) * 0.01 * max(maxDist2, xor(maxDist2, 1));
            
            % Normalize distributions...
            NormDistribution1 = NormDistribution1 / sum(NormDistribution1);
            NormDistribution2 = NormDistribution2 / sum(NormDistribution2);
            
            % Calculate Chi-squared distance...
            dist = sum(((NormDistribution1 - NormDistribution2).^2)./NormDistribution2);
            
        end
        
        %% ------- *** KULLBACK-LEIBLER DIVERGENCE *** --------------------
        %------------------------------------------------------------------
        function dist = kullbackleibler(Distribution1, Distribution2, varargin)
            
            % Add 1% noise...
            maxDist1 = max(Distribution1);
            maxDist2 = max(Distribution2);
            NormDistribution1 = Distribution1 +...
                                rand(size(Distribution1)) * 0.01 * max(maxDist1, xor(maxDist1, 1));
            NormDistribution2 = Distribution2 +...
                                rand(size(Distribution2)) * 0.01 * max(maxDist2, xor(maxDist2, 1));
            
            % Normalize distributions...
            NormDistribution1 = NormDistribution1 / sum(NormDistribution1);
            NormDistribution2 = NormDistribution2 / sum(NormDistribution2);
            
            % Calculate Kullback-Leibler divergence...
            dist = sum(NormDistribution1 .* log(NormDistribution1./NormDistribution2));
            
        end
        
        %% ------- *** TOTAL VARIATION DISTANCE *** -----------------------
        %------------------------------------------------------------------
        function dist = totalvariation(Distribution1, Distribution2, varargin)
            
            % Normalize distributions...
            NormDistribution1 = Distribution1 / sum(Distribution1);
            NormDistribution2 = Distribution2 / sum(Distribution2);
            
            % Calculate total variation distance...
            dist = sum(abs(NormDistribution1 - NormDistribution2)) / 2;
            
        end
        
        %% -------------- *** CROSS CORRELATION *** -----------------------
        % WARNING: This is just a sandbox right now, not functional
        %          as a metric/distance.
        %------------------------------------------------------------------
        function dist = crosscorrelation(Distribution1, Distribution2, varargin)
            
            % Normalize distributions...
            NormDistribution1 = Distribution1 / sum(Distribution1);
            NormDistribution2 = Distribution2 / sum(Distribution2);
            
            % Calculate Cross-correlation distance...
            dist = 1 / sum(sum(xcorr2(NormDistribution1, NormDistribution2)));
            
        end
        
        %% ----------------- *** RIDGE DISTANCE *** -----------------------
        %------------------------------------------------------------------
        function dist = ridge(Distribution1, Distribution2, varargin)
            
            % Get the universal distance matrix from the arguments list...
            D = varargin{1};
            
            % Normalize distributions...
            NormDistribution1 = Distribution1 / sum(Distribution1);
            NormDistribution2 = Distribution2 / sum(Distribution2);
            
            % Calculate ridge distance...
            N = length(NormDistribution1);
            foo = repmat(NormDistribution1, 1, N);
            bar = repmat(NormDistribution2', N, 1);
            dist = sum(sum(foo * (bar .* D)));
            
        end
        
        %% ------------ *** EARTH MOVER'S DISTANCE *** --------------------
        %------------------------------------------------------------------
        function dist = earthmovers(Distribution1, Distribution2, varargin)
            
            % Get the universal distance matrix from the arguments list...
            D = varargin{1};
            
            % Normalize distributions...
            NormDistribution1 = Distribution1 / sum(Distribution1);
            NormDistribution2 = Distribution2 / sum(Distribution2);
            
            % Calculate ridge distance...
            if sum(isnan(NormDistribution1)) > 0 || sum(isnan(NormDistribution2)) > 0
                dist = NaN;
            else
                dist = emd_mex(NormDistribution1', NormDistribution2', D);
            end
            
        end
        
    end
    
end
