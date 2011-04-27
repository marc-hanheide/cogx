classdef UniLayeredLearner < Learner
    % UNILAYEREDLEARNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        %% ------- *** OBJECTS *** ----------------------------------------
        %******************************************************************
        %******************************************************************
        % Cell array of modality objects...
        Modalities = [];
        
    end
    
    properties (SetAccess = private, Hidden = true)
        
        % Default settings...
        
        % Usage message...
        UsageMessage = ['\nUniLayeredLearner Usage: '...
                        'Please see the function comments for more detail.\n\n'];
                
    end
    
    methods

        %% ------- *** CONSTRUCTOR *** ------------------------------------
        %******************************************************************
        %******************************************************************
        function obj = UniLayeredLearner(varargin)
                          
            % Default settings...
            Data = [];
            ModalityTypes = {'codebook'};
            CodebookSizes = {[10 10]};
            CodebookNeighs = {'bubble'};
            CodebookLattices = {'hexa'};
            CodebookShapes = {'sheet'};
            CodebookInitMethods = {'rand'};
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
                    disp(['UniLayeredLearner.UniLayeredLearner(): Ignoring invalid argument #' num2str(i)]);
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
                [obj.TrainingData1Epoch obj.TestData] =...
                    obj.setupdatastructs(Data,...
                                         'TrainingIndices', TrainingIndices,...
                                         'TestIndices', TestIndices,...
                                         'epochs', 1);
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
                    
                case {'train', 'training', 'training_data'}
                    [TempData InModalityNorm] = obj.normalize(obj.TrainingData1Epoch.Modalities{1}.FeatureVectors,...
                                                              normalization_method);
                                                          
                    % Record the important stuff...
                    obj.TrainingData1Epoch.Modalities{1}.NormedFeatureVectors =...
                        TempData(1:size(obj.TrainingData1Epoch.Modalities{1}.FeatureVectors,2),:)';
               
                otherwise
                    obj.TrainingData1Epoch.Modalities{1}.NormedFeatureVectors =...
                        obj.TrainingData1Epoch.Modalities{1}.FeatureVectors;
               
                    obj.TestData.Modalities{1}.NormedFeatureVectors =...
                        obj.TestData.Modalities{1}.FeatureVectors;
                    
                    InModalityNorm = cell(size(obj.TrainingData1Epoch.Modalities{1}.FeatureVectors,1),1);
            end            
            
            
            %% INSTANTIATE MODALITY OBJECTS -------------------------------
            % Instantiate this UniLayeredLearner object's 2 modality objects...
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
                    
                case {'train', 'training', 'training_data'}
                    [TempData InModalityNorm] = obj.normalize(obj.TrainingData.Modalities{1}.FeatureVectors,...
                                                              normalization_method);
                                                          
                    % Record the important stuff...
                    obj.TrainingData.Modalities{1}.NormedFeatureVectors =...
                        TempData(1:size(obj.TrainingData.Modalities{1}.FeatureVectors,2),:)';
               
                otherwise
                    obj.TrainingData.Modalities{1}.NormedFeatureVectors =...
                        obj.TrainingData.Modalities{1}.FeatureVectors;
               
                    obj.TestData.Modalities{1}.NormedFeatureVectors =...
                        obj.TestData.Modalities{1}.FeatureVectors;
                    
                    InModalityNorm = cell(size(obj.TrainingData.Modalities{1}.FeatureVectors,1),1);
            end
            
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
            Updaters = {{'SOM'}};
            PhaseShifts = {{NaN}};   
            AlphaTypes = {{'linear'}};
            AlphaInits = {{1}};
            RadiusTypes = {{'linear'}};
            RadiusInits = {{5}};
            RadiusFins = {{1}};
            WindowSizes = {NaN};
            AlphaFeatureTypes = {{NaN}};
            AlphaFeatureInits = {{NaN}};
            Metrics = {{'euclidean'}};
                       
            
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
                    case {'featureselection', 'feature_selection'},...
                            i=i+1; obj.feature_selection = varargin{i};                            
                    case {'feature_selection_threshold'},...
                            i=i+1; obj.feature_selection_threshold = varargin{i};
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
                disp(['UniLayeredLearner.set(): Ignoring invalid argument #' num2str(i+1)]); 
              end
              i = i+1;
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
                                                                    
            % Set up CurrentSample data struct...
            obj.CurrentSample.Modalities{1}.FeatureNames = obj.TrainingData.Modalities{1}.FeatureNames;
            obj.CurrentSample.Modalities{1}.ClassNames = obj.TrainingData.Modalities{1}.ClassNames;
            obj.CurrentSample.Modalities{1}.nGroundTruths = obj.TrainingData.Modalities{1}.nGroundTruths;
            obj.CurrentSample.Modalities{1}.GroundTruthLabelIndices = obj.TrainingData.Modalities{1}.GroundTruthLabelIndices;
            
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
                    disp(['UniModalLearner.train(): Ignoring invalid argument #' num2str(i)]);
                    fprintf(obj.UsageMessage);
                end

                i = i + 1;
            end
            
            
            
            %% TRAINING LOOP ----------------------------------------------
            %--------------------------------------------------------------                        
            % for t = obj.t+1 : min(obj.t + sample_size, size(obj.TrainingData.FeatureVectors,2))
            % for t = 1:size(obj.TrainingData.FeatureVectors,2)                                
                
                
                %% GRAB CURRENT SAMPLE ------------------------------------
                %----------------------------------------------------------
%                 obj.CurrentSample.Modalities{1}.FeatureVectors(:,1) =...
%                     obj.TrainingData.Modalities{1}.FeatureVectors(:,t);
%                 obj.CurrentSample.Modalities{1}.NormedFeatureVectors(:,1) =...
%                     obj.TrainingData.Modalities{1}.NormedFeatureVectors(:,t);
%                 obj.CurrentSample.Modalities{1}.ClassLabels(:,1) =...
%                     obj.TrainingData.Modalities{1}.ClassLabels(:,t);
%                 obj.CurrentSample.Modalities{1}.GroundTruthLabelIndices =...
%                     obj.TrainingData.Modalities{1}.GroundTruthLabelIndices;
                
                %% TRAIN MODALITY -----------------------------------
                %---------------------------------------------------------- 
%                 obj.Modalities{1} =...
%                     obj.Modalities{1}.train(obj.CurrentSample.Modalities{1});
                
                obj.Modalities{1} =...
                    obj.Modalities{1}.train(obj.TrainingData.Modalities{1});
                                                      
                %% RECORD MODALITY TRAINING INFORMATION OVER TIME ---------
                %----------------------------------------------------------
%                 if obj.record                    
%                     obj.Modalities{1}.GroundTruthRecord(t,:) =...
%                         find(obj.CurrentSample.Modalities{1}.ClassLabels(...
%                                 obj.CurrentSample.Modalities{1}.GroundTruthLabelIndices,:));
%                     obj.Modalities{1}.ActivationsRecord(t,:) = obj.Modalities{1}.Activations;
%                     obj.Modalities{1}.BMURecord(t, :) = obj.Modalities{1}.BMUs;
%                 end
                
                                                      
                %% CLEAR MODALITY BMUs ------------------------------------
                %----------------------------------------------------------
                % obj.Modalities{1} = obj.Modalities{1}.clearbmus();
                
            % end            
            
            obj.is_trained = true;
            
        end
        
        
        %% ------- *** CLASSIFY *** ---------------------------------------
        %------------------------------------------------------------------
        %******************************************************************
        %******************************************************************
        function TestData = classify(obj, varargin)
            
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
                    case {'data', 'testdata'}, i=i+1; TestData = varargin{i};
                    case {'display'}, display = true;
                    case {'svm'}, Method = 'svm';
                    case {'lda'}, Method = 'lda';
                    case {'naivebayes', 'naivebayes_linear', 'naivebayes_diaglinear', 'diaglinear'},...
                         Method = 'diaglinear';
                    case {'naivebayes_quadratic', 'naivebayes_diagquadratic', 'diagquadratic'},...
                         Method = 'diagquadratic';
                    case {'lvq'}, Method = 'lvq';
                    case {'nodewise', 'node'}, Method = 'node';
                    case {'euclidean', 'euclid', 'euc'}, Metric = 'euclidean';
                    case {'hellinger', 'hell'}, Metric = 'hellinger';
                    case {'clusterwise', 'cluster'}, Method = 'cluster';
                    case {'nonhebbian'}, hebbian = false;
                      
                    otherwise, argok=0; 
                end
              elseif isstruct(varargin{i})
                  TestData = varargin{i};
              else
                argok = 0; 
              end
              if ~argok, 
                disp(['Ignoring invalid argument #' num2str(i+1)]); 
              end
              i = i+1; 
            end
            
            %% SET UP TEST DATA -------------------------------------------
            %--------------------------------------------------------------
            if isempty(TestData)
               TestData = obj.TestData;
               using_internal_testdata = true;
            end                
            
            %% USE INDIVIDUAL NODE ALPHA VALUES (FROM OLVQ) TO ESTIMATE ---
            % THE RELEVANCE OF INDIVIDUAL NODES...
            %--------------------------------------------------------------
            % if ~isempty(obj.Modalities{1}.Alphas)
            %     NodeRelevances = 1  - obj.Modalities{1}.Alphas;
            % else
            %     NodeRelevances = 0;
            % end
            % 
            % if sum(NodeRelevances) > 0
            %     NodeRelevances = 1  - obj.Modalities{1}.Alphas;
            %     Codebook = obj.Modalities{1}.SOM.codebook(NodeRelevances >= mean(NodeRelevances(:)), :);
            %     ClassLabels = obj.Modalities{1}.ClassLabels(NodeRelevances >= mean(NodeRelevances(:)));
            % else
                NodeRelevances = ones(size(obj.Modalities{1}.ClassLabels));
                Codebook = obj.Modalities{1}.SOM.codebook;
                ClassLabels = obj.Modalities{1}.ClassLabels;
            % end
            
            %% CLASSIFY TEST VECTORS IN MODALITY -------------------
            %  Classify the in modality test vectors in terms of out
            %  modality clusters.
            %--------------------------------------------------------------
            
            switch Method

                case 'svm',
                    %% TRAIN AN SVM CLASSIFIER IN THE IN-MODALITY -----------------
                    %--------------------------------------------------------------
                    if obj.SOM_based
                        Classifier =...
                            svmtrain(obj.Modalities{1}.SOM.codebook,...
                                     obj.Modalities{1}.ClassLabels,...
                                     'Kernel_Function', 'rbf')';
                    else
                        [ClassLabels bar] = find(obj.TrainingData.Modalities{1}.CategoryLabels(...
                                                 obj.TrainingData.Modalities{1}.AffordanceLabelIndices,1:obj.t));
                        
                        Classifier =...
                            svmtrain(obj.TrainingData.Modalities{1}.NormedFeatureVectors(:,1:obj.t)',...
                                     ClassLabels,...
                                     'Kernel_Function', 'rbf');
                    end

                    %% CONVERT & NORMALIZE TEST DATA ------------------------------
                    %--------------------------------------------------------------
                    SOMTestData = som_data_struct(TestData.Modalities{1}.FeatureVectors');

                    % Normalize...
                    SOMTestData = som_normalize(SOMTestData, obj.Norm);

                    %% CLASSIFICATION USING THE SVM CLASSIFIER --------------------
                    %--------------------------------------------------------------
                    TestData.Results.InToOutClassification =...
                        svmclassify(Classifier, SOMTestData.data);
                    
                    
                case {'diaglinear', 'diagquadratic'},
                    %% CONVERT & NORMALIZE TEST DATA ------------------------------
                    %--------------------------------------------------------------
                    SOMTestData = som_data_struct(TestData.Modalities{1}.FeatureVectors');

                    % Normalize...
                    SOMTestData = som_normalize(SOMTestData, obj.Norm);

                    %% CLASSIFICATION USING THE LDA CLASSIFIER --------------------
                    %--------------------------------------------------------------
                    if obj.SOM_based
                        TestData.Results.InToOutClassification =...
                            classify(SOMTestData.data, obj.Modalities{1}.SOM.codebook, obj.Modalities{1}.ClassLabels)';
                    else
                        [ClassLabels bar] = find(obj.TrainingData.Modalities{1}.CategoryLabels(...
                                                 obj.TrainingData.Modalities{1}.AffordanceLabelIndices,1:obj.t));
                        
                        TestData.Results.InToOutClassification =...
                            classify(SOMTestData.data,...
                                     obj.TrainingData.Modalities{1}.NormedFeatureVectors(:,1:obj.t)',...
                                     ClassLabels,...
                                     Method);
                    end
                    
                
                case 'lda',
                    %% CONVERT & NORMALIZE TEST DATA ------------------------------
                    %--------------------------------------------------------------
                    SOMTestData = som_data_struct(TestData.Modalities{1}.FeatureVectors');

                    % Normalize...
                    SOMTestData = som_normalize(SOMTestData, obj.Norm);

                    %% CLASSIFICATION USING THE LDA CLASSIFIER --------------------
                    %--------------------------------------------------------------
                    if obj.SOM_based
                        TestData.Results.InToOutClassification =...
                            classify(SOMTestData.data, obj.Modalities{1}.SOM.codebook, obj.Modalities{1}.ClassLabels)';
                    else
                        [ClassLabels bar] = find(obj.TrainingData.Modalities{1}.CategoryLabels(...
                                                 obj.TrainingData.Modalities{1}.AffordanceLabelIndices,1:obj.t));
                        
                        TestData.Results.InToOutClassification =...
                            classify(SOMTestData.data, obj.TrainingData.Modalities{1}.NormedFeatureVectors(:,1:obj.t)',...
                                                       ClassLabels);
                    end
                    
                case {'node', 'lvq'},
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
                        obj.Modalities{1}.SOM.mask = Mask;
                    
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
                        
                        if ~isempty(obj.feature_selection_threshold)                           
                            
                            [foo bar] = sort(Mask,'descend');
                            
                            QueryMask = zeros(size(Mask));
                            QueryMask(bar(1:obj.feature_selection_threshold)) = 1;
                            
                        else
                            
                            Mask(Mask < mean(Mask)) = 0;
                            QueryMask = Mask ./ norm(Mask,1);
                            
                        end
                        
                        [TestDataInMatches TestData.Modalities{1}.NormedFeatureVectors] =...
                                        obj.Modalities{1}.classify(TestData.Modalities{1},...
                                                                   'codebook', Codebook,...
                                                                   'mask', QueryMask);
                    
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
                            
                            TestSample = [];
                            TestSample.NormedFeatureVectors = TestData.Modalities{1}.NormedFeatureVectors(:,iTestData);
                                                        
                            % Classify
                            % QueryMask(QueryMask < mean(QueryMask)) = 0;
                            % QueryMask = QueryMask ./ norm(QueryMask,1);
                            TestDataInMatches(iTestData) =...
                                obj.Modalities{1}.classify(TestSample,...
                                                           'codebook', Codebook,...
                                                           'mask', QueryMask,...
                                                           'whichbmus', 'best');
                            
                        end
                        
                    % Classwise query-based LDA feature weighting...
                    elseif ~isempty(findstr(obj.feature_selection, 'classwise'))
                        
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
                            
                            Class1 = obj.Modalities{1}.ClassLabels(BMUClass1);
                            Class2 = obj.Modalities{1}.ClassLabels(BMUClass2);
                            
                            ClassMeans(1,:) = obj.Modalities{1}.ClassStats{Class1}.mean();
                            ClassMeans(2,:) = obj.Modalities{1}.ClassStats{Class2}.mean();
                            ClassVars(1,:) = obj.Modalities{1}.ClassStats{Class1}.var();
                            ClassVars(2,:) = obj.Modalities{1}.ClassStats{Class2}.var();
                            
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
                            QueryMask = (FisherCriterion ./ norm(FisherCriterion,1))';

                                                        
                            % Classify
                            % QueryMask(QueryMask < mean(QueryMask)) = 0;
                            % QueryMask = QueryMask ./ norm(QueryMask,1);
                            TestDataInMatches(iTestData) =...
                                obj.Modalities{1}.classify(TestData.Modalities{1}.FeatureVectors(:,iTestData)',...
                                                           'codebook', Codebook,...
                                                           'mask', QueryMask,...
                                                           'whichbmus', 'best');
                        end
                        
                    % Nodewise query-based LDA feature weighting...
                    elseif ~isempty(findstr(obj.feature_selection, 'nodewise'))
                        
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
                            
                            %% Relevance determination...
                            ClassMeans(1,:) = obj.Modalities{1}.NodeStats{TestDataBMUs(iTestData,1)}.mean();

                            if isnan(ClassMeans(1,:))
                                return;
                            else
                                OtherVar = obj.Modalities{1}.NodeStats{TestDataBMUs(iTestData,1)}.var();

                                if ~any(isnan(OtherVar))
                                    ClassVars(1,:) = OtherVar;
                                else
                                    ClassVars(1,:) = zeros(size(ClassMeans(1,:)));
                                end
                            end

                            OtherClassNodes = TestDataBMUs(iTestData,obj.Modalities{1}.ClassLabels(TestDataBMUs(iTestData,:)) ~= obj.Modalities{1}.ClassLabels(TestDataBMUs(iTestData,1)));

                            for iOther = 1:size(OtherClassNodes,2)

                                OtherMean = obj.Modalities{1}.NodeStats{OtherClassNodes(iOther)}.mean();

                                if ~any(isnan(OtherMean))

                                    ClassMeans(2,:) = OtherMean;

                                    OtherVar = obj.Modalities{1}.NodeStats{OtherClassNodes(iOther)}.var();

                                    if ~any(isnan(OtherVar))
                                        ClassVars(2,:) = OtherVar;
                                    else
                                        ClassVars(2,:) = zeros(size(ClassMeans(2,:)));
                                    end

                                    break;

                                elseif iOther >= size(OtherClassNodes,2)
                                    return;
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
                            QueryMask = (FisherCriterion ./ norm(FisherCriterion,1))';

                                                        
                            % Classify
                            % QueryMask(QueryMask < mean(QueryMask)) = 0;
                            % QueryMask = QueryMask ./ norm(QueryMask,1);
                            TestDataInMatches(iTestData) =...
                                obj.Modalities{1}.classify(TestData.Modalities{1}.FeatureVectors(:,iTestData)',...
                                                           'codebook', Codebook,...
                                                           'mask', QueryMask,...
                                                           'whichbmus', 'best');
                        end
                        
                    % Fuzzy feature weighting...
                    elseif ~isempty(findstr(obj.feature_selection, 'fuzzy'))
                        
                        if ~isempty(obj.feature_selection_threshold)                           
                            
                            [foo bar] = sort(Mask,'descend');
                            
                            QueryMask = zeros(size(Mask));
                            QueryMask(bar(1:obj.feature_selection_threshold)) = 1;
                            QueryMask(find(QueryMask)) = Mask(find(QueryMask));
                            QueryMask = QueryMask ./ norm(QueryMask,1);
                            
                        else
                            
                            QueryMask = Mask ./ norm(Mask,1);
                            
                        end
                        
                        [TestDataInMatches TestData.Modalities{1}.NormedFeatureVectors] =...
                                    obj.Modalities{1}.classify(TestData.Modalities{1},...
                                                               'codebook', Codebook,...
                                                               'mask', QueryMask);
                    
                    % Otherwise, ignore feature weights...
                    else
                        [TestDataInMatches TestData.Modalities{1}.NormedFeatureVectors] =...
                                    obj.Modalities{1}.classify(TestData.Modalities{1},...
                                                               'codebook', Codebook);
                    end
                    
                    % Save the results in the TestData struct...
                    TestData.Results.InToOutClassification = ClassLabels(TestDataInMatches')';
                    
                    % If we were using the internal test data in the
                    % class, we should overwrite it with the new results...
                    if using_internal_testdata
                        obj.TestData = TestData;
                    end
                
            end
                
        end        
  
        
        %% ------- *** EVALUATE *** ---------------------------------------
        %------------------------------------------------------------------
        %******************************************************************
        %******************************************************************
        function TestData = evaluate(obj, varargin)
            
            % Defaults...
            % display = false;
            
            % The TestData struct...
            TestData = [];
            
            % A flag that lets us know if we're using the TestData struct
            % in the object, or test data that was passed as an argument...
            using_internal_testdata = false;
            
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
                  % case 'display', i=i+1; DisplaySample = varargin{i}; display = true;
                      
                  otherwise, argok=0; 
                end
              elseif isstruct(varargin{i})
                  TestData = varargin{i};
              else
                argok = 0; 
              end
              if ~argok, 
                disp(['Ignoring invalid argument #' num2str(i+1)]); 
              end
              i = i+1; 
            end
            
            %% SET UP TEST DATA -------------------------------------------
            %--------------------------------------------------------------
            if isempty(TestData)
               TestData = obj.TestData;
               using_internal_testdata = true;
            end            
            
            %% CALCULATE EVALUATION SCORES ---------------------------------
            %---------------------------------------------------------------
            %---------------------------------------------------------------

            [foo bar] = find(TestData.Modalities{1}.ClassLabels(...
                                obj.TrainingData.Modalities{1}.GroundTruthLabelIndices, :));
                            
            TestData.Results.Matches =...
                (foo' == TestData.Results.InToOutClassification);
            
            TestData.Results.Score =...
                sum(TestData.Results.Matches, 2);
            
            TestData.Results.Percent =...
                TestData.Results.Score /...
                    size(TestData.Modalities{1}.FeatureVectors,2);
                
            % If we were using the internal test data in the
            % class, we should overwrite it with the new results...
            if using_internal_testdata
                obj.TestData = TestData;
            end
    
        end

        
    end
        
        
    
    
end
