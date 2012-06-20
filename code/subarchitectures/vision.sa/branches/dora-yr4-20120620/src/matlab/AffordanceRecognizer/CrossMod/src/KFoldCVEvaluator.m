classdef KFoldCVEvaluator < Evaluator & handle
    % KFoldCVEvaluator
    %
    %   K-Fold Cross Validation
    %
    
    properties
        
        %% ------- *** PROPERTIES *** -------------------------------------
        %******************************************************************
        %******************************************************************
        
        % Number of folds for the cross validation...
        K = [];
        
    end
    
    methods
        
        %% ------- *** CONSTRUCTOR *** ------------------------------------
        %******************************************************************
        %******************************************************************
        function obj = KFoldCVEvaluator(varargin)
            obj.set(varargin{1:end});
        end                
        
        %% ------- *** SET PROPERTIES *** ---------------------------------
        %******************************************************************
        %******************************************************************
        function obj = set(obj, varargin)
            
            % Defaults
            obj.K = 10;
            
            % Loop through arguments...
            i = 1;
            iPassedArgs = 1;
            while i <= length(varargin), 
                argok = 1; 
                if ischar(varargin{i}), 
                    switch lower(varargin{i}), 
                        case 'k', i=i+1; obj.K = varargin{i};                        
                        
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
                    disp(['KFoldCVEvaluator.set(): Ignoring invalid argument #' num2str(i)]);
                    % fprintf(obj.UsageMessage);
                end

                i = i + 1;
            end
            
            % Call the superclass set method...
            obj = set@Evaluator(obj, PassedArgs{1:end});                        
            
        end
        
        %% ------- *** RUN *** --------------------------------------------
        %******************************************************************
        %******************************************************************
        function obj = run(obj, varargin)
            
            % Defaults...
            doTraining = true;
            
            % Loop through arguments...
            i = 1;
            iPassedArgs = 1;
            while i <= length(varargin), 
                argok = 1; 
                if ischar(varargin{i}), 
                    switch lower(varargin{i}), 
                        case {'train', 'dotraining'}, i=i+1; doTraining = varargin{i};
                        
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
                    disp(['KFoldCVEvaluator.run(): Ignoring invalid argument #' num2str(i)]);
                    % fprintf(obj.UsageMessage);
                end

                i = i + 1;
            end            
            
            %% K-Fold Cross Validation...
            fprintf('\n\n%d-Fold Cross Validation...\n\n', obj.K);
            
            % Start matlabpool if required...
            if obj.doParallel
                fprintf('\nOpening matlabpool...');
                if obj.nCores == Inf
                    matlabpool local;
                else
                    matlabpool('local', obj.nCores);
                end
            end
            
            % Main loop...
            try
                for iTrial = 1:obj.nTrials
                    
                    % Grab the data...
                    Data = obj.Data;
                    
                    % Make sure we only train (and test) with allowed
                    % training classes...
                    AllowedIndices = find(sum(Data.ClassLabels(Data.AllowedTrainingClassIndices',:),1));
                    Data.FeatureVectors = Data.FeatureVectors(:,AllowedIndices);
                    Data.ClassLabels = Data.ClassLabels(:,AllowedIndices);
                    
                    % Randomize the data...
                    v = ver('matlab');
                    if str2double(v.Version) < 7.7
                        rand('twister', sum(100*clock));
                    else
                        RandStream.setDefaultStream(RandStream('mrg32k3a', 'seed', sum(100*clock)));
                    end
                    Data = Learner.randomize(Data);


                    % Run the evaluation on each fold...
                    for iFold = 1:obj.K

                        %% Create learners for this trial/fold...
                        if doTraining
                            
                            if iFold < obj.K
                                TestIndices =...
                                    (iFold-1) * floor(size(Data.FeatureVectors,2)/obj.K) + 1 :...
                                        iFold * floor(size(Data.FeatureVectors,2) / obj.K);
                            else
                                TestIndices =...
                                    (iFold-1) * floor(size(Data.FeatureVectors,2)/obj.K) + 1 :...
                                        size(Data.FeatureVectors,2);
                            end
                            
                            [TrainingData1Epoch TestData] = Learner.setupdatastructs(Data,...
                                                                                     'TestIndices', TestIndices,...
                                                                                     'epochs', 1);

                            % Set up training and test data structs...
                            [TrainingData TestData] = Learner.setupdatastructs(Data,...
                                                                               'TestIndices', TestIndices,...
                                                                               'epochs', obj.nEpochs);
                                                                                                                                                                                  
                            
                            % If the test intervals were specified in epochs,
                            % convert to timesteps...                                               
                            if ~isempty(obj.test_interval_epochs)
                                epoch_length = size(TrainingData.FeatureVectors,2) / obj.nEpochs;
                                obj.test_interval_timesteps = obj.test_interval_epochs * epoch_length;
                            end
                            
                            % Calculate the number of evaluation
                            % intervals...
                            if size(obj.test_interval_timesteps,2) > 1                                
                                nIntervals = size(obj.test_interval_timesteps,2);
                                
                            elseif obj.test_interval_timesteps == Inf                                
                                nIntervals = 1;                                                            
                                
                            else
                                nIntervals = ceil(size(TrainingData.FeatureVectors,2) / obj.test_interval_timesteps);
                                
                            end
                            
                            % Grab a seed...
                            seed = sum(100*clock);

                            % Create the Learner objects...
                            clear LearnerObjects;
                            for iLearner = 1:length(obj.Learners)

                                % Re-seed the random number generator with the
                                % same start point and seed for each learner...
                                if str2double(v.Version) < 7.7
                                    rand('twister', seed);
                                else
                                    RandStream.setDefaultStream(RandStream('mrg32k3a', 'seed', seed));
                                end
                                
                                % Turn off command-line display if running
                                % in parallel mode...
                                if obj.doParallel
                                    display_param = false;
                                else
                                    display_param = true;
                                end

                                % Create learner objects...
                                LearnerObjects{iLearner} =...
                                    obj.createLearner('Name', obj.LearnerNames{iLearner},...
                                                      'TrainingData1Epoch', TrainingData1Epoch,...
                                                      'TrainingData', TrainingData,...
                                                      'TestData', TestData,...
                                                      'display', display_param,...
                                                      obj.Learners{iLearner}{1:end});
                                                  
                                % Save the category name...
                                obj.Results{iLearner}.Trials{iTrial}.Classes{iFold}.ClassName =...
                                    ['Fold ' num2str(iFold)];

                            end

                        else
                            % Leaving a dummy here for now...
                        end

                        %% Train/Classify/Evaluate...       
                        for iTime = 1:nIntervals

                            % Print the category name to screen...
                            fprintf('\n\nTrial %d:', iTrial);
                            fprintf('\n\nFold %d:', iFold);

                            % Train them...
                            if obj.doParallel
                                
                                iRetry = 1;
                                restart_required = true;                                                                
                                
                                % If the parfor loop crashes, keep restarting the cores
                                % and retrying (3 attempts max)...
                                while restart_required
                                    try
                                        
                                        fprintf('\n\nEntering parfor loop for TRAINING, CLASSIFICATION & EVALUATION...');
                                        fprintf('\nEvaluation interval: %d', iTime);                                        
                                        fprintf('\nMaintaining full radio silence until finished...');
                                        
                                        % Parallel loop...
                                        parfor iLearner = 1:length(obj.Learners)

                                            % if ~isempty(obj.LearnerNames{iLearner})
                                                % fprintf('\nLEARNER: %s', obj.LearnerNames{iLearner});
                                            % end

                                            LearnerObjects{iLearner} =...
                                                obj.computelearnerjob(LearnerObjects{iLearner},...
                                                                      iTime);
                                        end
                                        
                                        fprintf('\nRecording learner states...');
                                        
                                        % Use a regular for loop to save
                                        % the state...
                                        for iLearner = 1:length(obj.Learners)
                                            
                                            obj.Results{iLearner} =...
                                                obj.recordlearnerstate(LearnerObjects{iLearner},...
                                                                       obj.Results{iLearner},...
                                                                       iTrial,...
                                                                       iFold,...
                                                                       iTime,...
                                                                       nIntervals);
                                        end
                                        
                                        fprintf('\nSaving experimental state to disk...');
                                        
                                        % Save to disk...
                                        obj.save();
                                        
                                        restart_required = false;
                                        
                                    catch MyError
                                        % Shut down the parallel processors...                                    
                                        matlabpool close force;
                                        pause(5);
                                        
                                        if iRetry <= 3
                                            
                                            % Print warning message...
                                            fprintf('\n\n\nWARNING: Parallel processing failure!  Restarting matlabpool and retrying...\n\n\n');
                                            fprintf('Retry attempt %d\n\n\n', iRetry);                                                                                                                                    
                                            
                                            fprintf('\nOpening matlabpool...');
                                            if obj.nCores == Inf
                                                matlabpool local;
                                            else
                                                matlabpool('local', obj.nCores);
                                            end
                                            
                                            iRetry = iRetry + 1;
                                            restart_required = true;
                                        else
                                            restart_required = false;
                                            
                                            rethrow(MyError);
                                        end
                                    end
                                end

                            else                            
                                % Regular loop...
                                for iLearner = 1:length(obj.Learners)

                                    if ~isempty(obj.LearnerNames{iLearner})
                                        fprintf('\nLEARNER: %s', obj.LearnerNames{iLearner});
                                    end

                                    LearnerObjects{iLearner} =...
                                        obj.computelearnerjob(LearnerObjects{iLearner},...
                                                              iTime);
                                                          
                                    obj.Results{iLearner} =...
                                                obj.recordlearnerstate(LearnerObjects{iLearner},...
                                                                     obj.Results{iLearner},...
                                                                     iTrial,...
                                                                     iFold,...
                                                                     iTime,...
                                                                     nIntervals);                                    
                                end
                                
                                fprintf('\nSaving experimental state to disk...');
                                
                                % Save to disk...
                                obj.save();

                            end
                        end

                    end
                end
                
            catch MyError
                
                % Stop matlabpool...
                if obj.doParallel

                    matlabpool close;

                end
                
                rethrow(MyError);
            end
            
            % Stop matlabpool...
            if obj.doParallel
                
                matlabpool close;
                
            end            

            fprintf('\n\n...FINISHED!\n\n');
            
        end
            
    end
        
end
