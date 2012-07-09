classdef Learner < handle
    % LEARNER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        %% ------- *** PROPERTIES *** -------------------------------------
        %******************************************************************
        %******************************************************************
        % Learner name...
        Name = [];
        
        % Training & test data...
        TrainingData1Epoch = [];
        TrainingData = [];
        TestData = [];        
        
        % Current timestep...
        t = 0;
        
        % Current sample...
        CurrentSample = [];
        
        % Has the learner been trained?
        is_trained = false;
        
        % Record statistics switch...
        record = false;
        
        % Display command-line output etc...
        display = true;
        
        % Feature selection switch...
        feature_selection = 'off';
        
        % Feature selection max features...
        feature_selection_max = [];
        
        % Feature selection feedback in training...
        feature_selection_feedback = false;
        
        % Default classification method...
        classification_method = 'node';
        
    end

    methods (Abstract)
        
        %% ------- *** SET PROPERTIES *** ---------------------------------
        %******************************************************************
        %******************************************************************
        obj = set(obj, varargin)
        
        %% ------- *** TRAIN *** ------------------------------------------
        %******************************************************************
        %******************************************************************
        obj = train(obj, varargin)
            
        %% ------- *** CLASSIFY *** ---------------------------------------
        %******************************************************************
        %******************************************************************
        obj = classify(obj, varargin)
            
        %% ------- *** EVALUATE *** ---------------------------------------
        %******************************************************************
        %******************************************************************
        obj = evaluate(obj, varargin)

        
    end        
    
    methods (Static = true)
        
        %% ------- *** SETUPDATASTRUCTS *** -------------------------------
        % *****************************************************************
        % *****************************************************************
        function [TrainingData TestData] = setupdatastructs(Data, varargin)
            
            % Defaults...
            TrainingData = [];
            TestData = [];
            TrainingIndices = [];
            TestIndices = [];
            epochs = 1;
            
            % Loop through arguments...
            i = 1;
            iPassedArgs = 1;
            while i <= length(varargin), 
                argok = 1; 
                if ischar(varargin{i}), 
                    switch lower(varargin{i}), 
                        case 'trainingindices', i=i+1; TrainingIndices = varargin{i};
                        case 'testindices', i=i+1; TestIndices = varargin{i};
                        case 'epochs', i=i+1; epochs = varargin{i};
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
                    disp(['Learner.setupdatastructs(): Ignoring invalid argument #' num2str(i)]);
                    % fprintf(UsageMessage);
                end

                i = i + 1;
            end
            
            % Grab the ground truth labels --------------------------------
            %--------------------------------------------------------------            
            nGroundTruths = size(Data.GroundTruthClassIndices,2);
            GroundTruthLabelIndices = Data.GroundTruthClassIndices;
            
            if isempty(TrainingIndices) && isempty(TestIndices)
                TrainingIndices = 1:length(Data.FeatureVectors);
            elseif isempty(TrainingIndices) && ~isempty(TestIndices) &&...
                   length(TestIndices) < length(Data.FeatureVectors)
                TrainingIndices = find(~ismember(1:length(Data.FeatureVectors), TestIndices));
            elseif isempty(TestIndices) && ~isempty(TrainingIndices) &&...
                   length(TrainingIndices) < length(Data.FeatureVectors)
                TestIndices = find(~ismember(1:length(Data.FeatureVectors), TrainingIndices));
            end

            % Get the field names from the Data struct...
            DataFieldNames = fieldnames(Data);

            % If training data vector indices were specified...
            if ~isempty(TrainingIndices)

                % Copy everything from Data...        
                for i = 1:length(DataFieldNames)
                    if ~strcmp(DataFieldNames{i}, 'Modalities')
                        TrainingData.(DataFieldNames{i}) =...
                            Data.(DataFieldNames{i});
                    end
                end

                % Change the feature vectors and category labels as necessary...
                TrainingData.ClassLabels = Data.ClassLabels(:,TrainingIndices);
                TrainingData.FeatureVectors = Data.FeatureVectors(:,TrainingIndices);
                % nGroundTruths = nGroundTruths;
                % GroundTruthLabelIndices = GroundTruthLabelIndices;
                
                % If modality feature indices were provided, use them
                % to set up modality data structs in the main data
                % struct...
                if isfield(Data, 'Modalities')
                    for i = 1:length(Data.Modalities)
                        TrainingData.Modalities{i}.FeatureNames = TrainingData.FeatureNames(:,Data.Modalities{i}.FeatureIndices);
                        TrainingData.Modalities{i}.ClassNames = TrainingData.ClassNames;
                        TrainingData.Modalities{i}.FeatureVectors =...
                            TrainingData.FeatureVectors(Data.Modalities{i}.FeatureIndices, :);
                        TrainingData.Modalities{i}.ClassLabels = TrainingData.ClassLabels;
                        TrainingData.Modalities{i}.nGroundTruths = nGroundTruths;
                        TrainingData.Modalities{i}.GroundTruthLabelIndices = GroundTruthLabelIndices;                            
                    end
                end
                
            else
                error('No training indices specified!');
            end

            % If test data vector indices were specified...
            if ~isempty(TestIndices)

                % Copy everything from Data...
                for i = 1:length(DataFieldNames)
                    if ~strcmp(DataFieldNames{i}, 'Modalities')
                        TestData.(DataFieldNames{i}) =...
                            Data.(DataFieldNames{i});
                    end
                end

                % Change the feature vectors and category labels as necessary...
                TestData.ClassLabels = Data.ClassLabels(:,TestIndices);
                TestData.FeatureVectors = Data.FeatureVectors(:,TestIndices);
                % TestData.nGroundTruths = nGroundTruths;
                % TestData.GroundTruthLabelIndices = GroundTruthLabelIndices;
                
                % If modality feature indices were provided, use them
                % to set up modality data structs in the main data
                % struct...
                if isfield(Data, 'Modalities')
                    for i = 1:length(Data.Modalities)
                        TestData.Modalities{i}.FeatureNames = TestData.FeatureNames(:,Data.Modalities{i}.FeatureIndices);
                        TestData.Modalities{i}.ClassNames = TestData.ClassNames;
                        TestData.Modalities{i}.FeatureVectors =...
                            TestData.FeatureVectors(Data.Modalities{i}.FeatureIndices, :);
                        TestData.Modalities{i}.ClassLabels = TestData.ClassLabels;
                        TrainingData.Modalities{i}.nGroundTruths = nGroundTruths;
                        TrainingData.Modalities{i}.GroundTruthLabelIndices = GroundTruthLabelIndices;
                    end
                end
                
            end
            
            %% EXPAND TRAINING DATA STRUCT FOR MULTIPLE EPOCHS ------------
            % If the training data should be used for multiple training
            % epochs, multiply the training data struct appropriately...
            %--------------------------------------------------------------
            if epochs > 1
                TempTrainingData = TrainingData;
                
                for i = 2:epochs
                    
                    TrainingData.FeatureVectors =...
                        [TrainingData.FeatureVectors TempTrainingData.FeatureVectors];
                    TrainingData.ClassLabels =...
                        [TrainingData.ClassLabels TempTrainingData.ClassLabels];
                    
                    % If modality feature indices were provided, use them
                    % to set up modality data structs in the main data
                    % struct...
                    if isfield(Data, 'Modalities')
                        for iMod = 1:length(Data.Modalities)
                            TrainingData.Modalities{iMod}.FeatureVectors =...
                                [TrainingData.Modalities{iMod}.FeatureVectors TempTrainingData.Modalities{iMod}.FeatureVectors];
                            if isfield(TrainingData.Modalities{iMod}, 'NormedFeatureVectors')
                                TrainingData.Modalities{iMod}.NormedFeatureVectors =...
                                [TrainingData.Modalities{iMod}.NormedFeatureVectors TempTrainingData.Modalities{iMod}.NormedFeatureVectors];
                            end
                            TrainingData.Modalities{iMod}.ClassLabels =...
                                [TrainingData.Modalities{iMod}.ClassLabels TempTrainingData.Modalities{iMod}.ClassLabels];
                        end
                    end
                                                            
                end
                
                clear TempTrainingData;
            end
            
            
        end
        
        %% ------- *** NORMALIZE *** --------------------------------------
        % *****************************************************************
        % *****************************************************************
        function [NormedData NormStruct] = normalize(Data, varargin)
            
            % Defaults...
            Method = 'range';
            
            % Loop through arguments...
            i = 1;
            while i <= length(varargin), 
                argok = 1; 
                if ischar(varargin{i}), 
                    switch lower(varargin{i}), 
                        case 'method', i=i+1; Method = varargin{i};
                    end
                else
                    argok = 0;
                end

                if ~argok, 
                    disp(['Learner.normalize(): Ignoring invalid argument #' num2str(i)]);
                    % fprintf(UsageMessage);
                end

                i = i + 1;
            end
            
            % Were we passed a data struct, or raw data?
            if isstruct(Data)
                TempData = Data.FeatureVectors;
            else
                TempData = Data;
            end
            
            % Normalize...
            switch Method                
                
                case {'1', '2', '3', '4'}
                    TempData = normalize(TempData, str2num(Method));
                    
                    NormStruct = cell(size(obj.TrainingData1Epoch.Modalities{1}.FeatureVectors,1),1);
                    
                otherwise
                    TempData1 = som_data_struct(TempData);                    
                    TempData1 = som_normalize(TempData1, Method);
                    
                    TempData = TempData1.data;                    
                    NormStruct = TempData1.comp_norm;
            end
            
            % Were we passed a data struct, or raw data?
            if isstruct(Data)
                NormedData = Data;
                NormedData.NormedFeatureVectors = TempData;
            else
                NormedData = TempData;
            end
            
            % Are there modalities?
            if isstruct(Data)
                if isfield(Data, 'Modalities')
                    
                    for iMod = 1:length(Data.Modalities)
                        
                        TempData = Data.Modalities{iMod}.FeatureVectors;
                        
                        % Normalize...
                        switch Method                

                            case {'1', '2', '3', '4'}
                                TempData = normalize(TempData, str2num(Method));

                                NormStruct = cell(size(obj.TrainingData1Epoch.Modalities{1}.FeatureVectors,1),1);

                            otherwise
                                TempData1 = som_data_struct(TempData);                    
                                TempData1 = som_normalize(TempData1, Method);

                                TempData = TempData1.data;                    
                                NormStruct = TempData1.comp_norm;
                        end
                        
                        Data.Modalities{iMod}.NormedFeatureVectors = TempData;
                        
                    end                        
                        
                end
            end                       
            
        end
        
        %% ------- *** RANDOMIZE *** --------------------------------------
        % *****************************************************************
        % *****************************************************************
        function RandomizedData = randomize(Data)
            
            % rand('twister',sum(size(Data.FeatureVectors,2)*clock));

            RandIndices = randperm(size(Data.FeatureVectors,2));

            RandomizedData = Data;
            for i = 1:size(RandIndices,2)
                
                % RandomizedData.ClassLabels(:,i) = Data.ClassLabels(:,RandIndices(i));
                % RandomizedData.FeatureVectors(:,i) = Data.FeatureVectors(:,RandIndices(i));
                
                % If the data struct also contains modality data structs,
                % randomize them too using the same randomization...
                if isfield(RandomizedData, 'Modalities')
                    for j = 1:length(RandomizedData.Modalities)
                        
                        if isfield(RandomizedData.Modalities{j}, 'FeatureVectors')
                            RandomizedData.Modalities{j}.FeatureVectors(:,i) =...
                                Data.Modalities{j}.FeatureVectors(:,RandIndices(i));
                        end
                        
                        if isfield(RandomizedData.Modalities{j}, 'NormedFeatureVectors')
                            RandomizedData.Modalities{j}.NormedFeatureVectors(:,i) =...
                                Data.Modalities{j}.NormedFeatureVectors(:,RandIndices(i));
                        end
                        
                        if isfield(RandomizedData.Modalities{j}, 'ClassLabels')
                            RandomizedData.Modalities{j}.ClassLabels(:,i) =...
                                Data.Modalities{j}.ClassLabels(:,RandIndices(i));
                        end
                    end
                end
                
            end
            
        end
        
    end
    
    methods
        
        %% ------- *** COPY *** -------------------------------------------
        % *****************************************************************
        % Make a copy of a handle object.
        % *****************************************************************
        function new = copy(this)
            % From the file exchange (doesn't fully work):
            
            % % Instantiate new object of the same class.
            % new = feval(class(this));
            % 
            % % Copy all non-hidden properties.
            % p = fieldnames(struct(this));
            % for i = 1:length(p)
            %     new.(p{i}) = this.(p{i});
            % end
            
            % My way...
            Filename = 'temp';
            iFilenum = 1;
            while exist([Filename iFilenum '.mat']) >= 2
                iFilenum = iFilenum + 1;
            end
            save([Filename iFilenum '.mat'], 'this');
            Foo = load([Filename iFilenum '.mat']);
            new = Foo.this;
            delete([Filename iFilenum '.mat']);
        end
        
    end
    
end