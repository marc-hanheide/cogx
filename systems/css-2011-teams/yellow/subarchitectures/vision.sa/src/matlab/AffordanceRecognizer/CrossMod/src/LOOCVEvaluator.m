classdef LOOCVEvaluator < Evaluator & handle
    % LOOCVEvaluator Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        %% ------- *** PROPERTIES *** -------------------------------------
        %******************************************************************
        %******************************************************************
        
        % Number of classes the cross validation will be run over...
        nClasses = 1;
        
    end
    
    methods
        
        %% ------- *** CONSTRUCTOR *** ------------------------------------
        %******************************************************************
        %******************************************************************
        function obj = LOOCVEvaluator(varargin)
            obj.set(varargin{1:end});
        end                
        
        %% ------- *** SET PROPERTIES *** ---------------------------------
        %******************************************************************
        %******************************************************************
        function obj = set(obj, varargin)
            
            % Call the superclass set method...
            obj = set@Evaluator(obj, varargin{1:end});
            
            % Set the number of classes the cross validation will be run
            % over...
            obj.nClasses = length(obj.Data.AllowedTrainingClassIndices);
            
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
                    disp(['LOOCVEvalutator.run(): Ignoring invalid argument #' num2str(i)]);
                    % fprintf(obj.UsageMessage);
                end

                i = i + 1;
            end            
            
            %% Leave-One-Out Cross Validation...
            fprintf('\n\nLeave-One-Out Cross Validation...\n\n');
            
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

                    % Run the evaluation on each test class...
                    for iClass = 1:obj.nClasses

                        %% Create learners for this trial/class...
                        if doTraining
                            % Find the current test category data vector indices...
                            TestClass = obj.Data.AllowedTrainingClassIndices(iClass);
                            TestIndices = find(obj.Data.ClassLabels(TestClass,:));

                            % Find the current training category data vector indices...
                            TrainingClasses = obj.Data.AllowedTrainingClassIndices(~ismember(obj.Data.AllowedTrainingClassIndices,TestClass));
                            TrainingIndices = [];
                            for iTrainingClass = 1:length(TrainingClasses)
                                TrainingIndices = [TrainingIndices find(obj.Data.ClassLabels(TrainingClasses(iTrainingClass),:))];
                            end

                            % Calculate the number of evaluation intervals...
                            if obj.interval_size == Inf
                                nIntervals = 1;
                            else
                                nIntervals = ceil(size(TrainingIndices,2) / obj.interval_size);
                            end

                            % Set up training and test data structs...
                            [TrainingData1Epoch TestData] = Learner.setupdatastructs(obj.Data,...
                                                                                     'TrainingIndices', TrainingIndices,...
                                                                                     'TestIndices', TestIndices,...
                                                                                     'epochs', 1);

                            % Set up training and test data structs...
                            [TrainingData TestData] = Learner.setupdatastructs(obj.Data,...
                                                                               'TrainingIndices', TrainingIndices,...
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


                            % Randomize the training data...
                            v = ver('matlab');
                            if str2double(v.Version) < 7.7
                                rand('twister', sum(100*clock));
                            else
                                RandStream.setDefaultStream(RandStream('mrg32k3a', 'seed', sum(100*clock)));
                            end
                            TrainingData = Learner.randomize(TrainingData);

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
                                obj.Results{iLearner}.Trials{iTrial}.Classes{iClass}.ClassName =...
                                    obj.Data.ClassNames(TestClass);

                            end

                        else
                            % Leaving a dummy here for now...
                        end

                        %% Train/Classify/Evaluate...       
                        for iTime = 1:nIntervals

                            % Print the category name to screen...
                            fprintf('\n\nTrial %s:', num2str(iTrial));
                            fprintf('\n*** %s ***\n', char(obj.Data.ClassNames(TestClass)));

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
                                                                       iClass,...
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
                                                                     iClass,...
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
