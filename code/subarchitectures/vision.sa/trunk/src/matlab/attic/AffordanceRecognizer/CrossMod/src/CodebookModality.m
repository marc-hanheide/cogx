classdef CodebookModality < Modality
    % CODEBOOKMODALITY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        %% ------- *** OBJECTS *** ----------------------------------------
        %******************************************************************
        %******************************************************************
        % SOM Codebook
        %------------------
        % Ideally this would be an interface to a 'Codebook' object,
        % but it would be tricky to decouple all the somtoolbox stuff,
        % so for now, it will remain a somtoolbox struct...
        SOM = [];
        
        
        %% ------- *** FUNCTION HANDLES *** -------------------------------
        %******************************************************************
        %******************************************************************
        % Handle to a method that finds the best-matching unit(s) (BMUs)
        % in the SOM codebook.
        metric = [];
        metric_withmask = [];
        
        
        %% ------- *** PROPERTIES *** -------------------------------------
        %******************************************************************
        %******************************************************************
        % Current training step
        %-----------------------
        % Overall...
        t = 0;
        % A counter for the no. of timesteps we updated at...
        t_nonnull = 1;
        
        % Codebook initialization method...
        %-----------
        InitMethod = [];
        
        % Distances
        %-----------
        % Sample-to-SOM-node distances at current training step,
        % i.e. sum-squared distances...
        Distances = [];
        
        % Auxiliary distances at current training step.
        % This is used to store distances calculated
        % outside of the modality object, i.e. distances related to other
        % modalities (e.g. cross-modal Hellinger distances) which are
        % subsequently used to make calculations inside the object.
        AuxDists = [];
        nAuxDists = 1;
        
        % A place to record running statistics about the auxiliary
        % distance...
        AuxDistStats = [];
        
        % Class labels
        %--------------
        ClassLabels = [];
        nClasses = [];
        
        % Best matching units (BMUs)...
        %--------------------------------------
        % Number of BMUs we should look for...
        nBMUs = 1;
        % BMUs for current timestep...
        BMUs = [];
        
        % Map activations over time
        %---------------------------
        Activations = [];
        
        % Map activations over time
        %---------------------------
        AccuracyHist = [];
        
        % Mask statistics (RunningStat object)...
        %---------------------------
        MaskStats = [];
        
        % Class statistics (RunningStat object)...
        %---------------------------
        ClassStats = [];
        
        % Node statistics (Cell array of RunningStat objects)...
        NodeStats = [];
        
        % Clustering
        %------------
        Clustering = [];
        
        % Recorded information over training time...
        % (if the record parameter is passed)
        %-------------------------------------------
        AuxDistsRecord = [];
        AuxDistMeanRecord = [];
        AuxDistStDRecord = [];
        GroundTruthRecord = [];
        BMURecord = [];
        ActivationsRecord = [];
        
        %% SOM settings & state...
        %-------------------------
        % Normalization struct...
        NormStruct = [];
        
        % Normalization method...
        NormMethod = 'range';
        
        % Has the SOM been labeled?
        SOM_is_labeled = false;
        
        % Structure...
        Size = [10 10];
        Lattice = 'hexa';
        Shape = 'sheet';
        KNN_max_k = 20;
        
        % Feature dimensions...
        dim = [];
        
        % Neighborhood type...
        Neigh = 'bubble';
        
        % Neighborhood radius...
        radius_type = 'linear';
        rini = 5;
        rfin = 1;

        % Training length...
        trainlen = 0;
        
        % Metric...
        metric_type = 'euclidean';
        
        % Feature selection feedback in training...
        feature_selection_feedback = false;
        
        % Useful stuff...
        Ud = []; % Distance between map units on the grid...
        CostMatrix = []; % Cost matrix for grid units...
        mu_x_1 = [];     % This is used pretty often...
        ZeroMask = [];
        OnesMask = [];
        
        Dx = [];
        
        % Current data sample...
        x = [];
        % Its known components...
        known = [];
        
        % Current alpha learning rate value...
        a = 0.2;
        % Current feature selection alpha learning rate value...
        a_f = 0.2;
        % Current window size for LVQ2.1-style algorithms...
        epsilon = 0.3;
        % Matrix of individual alphas for each codebook vector, used for
        % optimized-learning rate algorithms...
        Alphas = [];
        % Current radius...
        r = 5;        
        % Current neighbourhood...
        h = [];

        % Display updates...
        display = false;
        
    end
    
    methods
        
        %% ------- *** CONSTRUCTOR *** ------------------------------------
        %******************************************************************
        %******************************************************************
        function obj = CodebookModality(Data, varargin)
            
            % Loop through arguments...
            obj = obj.set(varargin{:});
            
            %% GET NORMALIZATION STRUCT FROM ARGUMENT OR INITIAL DATA -----
            %--------------------------------------------------------------
            if ~isfield(Data, 'NormedFeatureVectors')
                
                % Temporary data struct...
                SOMDataTemp = som_data_struct(Data.FeatureVectors',...
                                              'comp_names', Data.FeatureNames');

                % Normalize...
                % SOMDataTemp = som_normalize(SOMDataTemp, obj.NormMethod);
                % obj.NormStruct = SOMDataTemp.comp_norm;
                
                switch obj.NormMethod
                    case {'1', '2', '3', '4'}                                
                        SOMDataTemp.data = normalize(SOMDataTemp.data, str2num(obj.NormMethod));
                    otherwise
                        SOMDataTemp = som_normalize(SOMDataTemp, obj.NormMethod);
                end
                
                % Create randomized SOM struct...
                obj.SOM = som_randinit(SOMDataTemp,...
                                       'msize', obj.Size,...
                                       'lattice', obj.Lattice,...
                                       'shape', obj.Shape);
                                   
            else
                
                % Create randomized SOM struct...
                obj.SOM = som_randinit(Data.NormedFeatureVectors',...
                                       'msize', obj.Size,...
                                       'lattice', obj.Lattice,...
                                       'shape', obj.Shape);
                
            end
            
            
            

            %% INITIALIZE CODEBOOK ----------------------------------------
            % The codebook may be initialized in different ways depending
            % on what argument was passed.  Certain LVQ algorithms (e.g.
            % GLVQ) are sensitive to codebook initialization; randomized
            % codebooks can cause such algorithms to fail catastrophically.
            %--------------------------------------------------------------
            switch lower(obj.InitMethod)
                
                case 'rand_sample'

                    % Set up class labels for the codebook nodes...
                    obj.ClassLabels = zeros(size(obj.SOM.codebook,1),1);
                    increment = floor(size(obj.SOM.codebook,1) / Data.nGroundTruths);
                    for j = 1:Data.nGroundTruths
                        obj.ClassLabels( ((j-1) * increment) + 1 : max(j * increment, size(obj.SOM.codebook,1))) = j;                                       
                    end
                    
                    % For each codebook node of each class, randomly
                    % sample a data vector...
                    for iNode = 1:size(obj.ClassLabels,1)
                        ClassDataIndices = find(Data.ClassLabels(Data.GroundTruthLabelIndices(:,obj.ClassLabels(iNode)),:));
                        iRandomClassSample = ceil(rand * size(find(Data.ClassLabels(Data.GroundTruthLabelIndices(:,obj.ClassLabels(iNode)),:)),2));
                        iSample = ClassDataIndices(iRandomClassSample);
                        obj.SOM.codebook(iNode,:) = Data.NormedFeatureVectors(:,iSample)';
                    end

                case {'dist_sample', 'dist_mean'}
                    
                    % Set up class labels for the codebook nodes...
                    obj.ClassLabels = zeros(size(obj.SOM.codebook,1),1);
                    increment = floor(size(obj.SOM.codebook,1) / Data.nGroundTruths);
                    for j = 1:Data.nGroundTruths
                        obj.ClassLabels( ((j-1) * increment) + 1 : max(j * increment, size(obj.SOM.codebook,1))) = j;                                       
                    end
                    
                    % For each cluster of each class, use randomly
                    % sampled data vectors to initialize codebook
                    % vectors...
                    iNode = 1;
                    
                    for iClass = 1:size(Data.GroundTruthLabelIndices,2)
                        
                        % Grab the sample indices for this class...
                        ClassDataIndices = find(Data.ClassLabels(Data.GroundTruthLabelIndices(:,iClass),:));
                        
                        % Sometimes the clustering runs amok, so if an
                        % error is caught here, we redo the clustering...
                        iRetry = 1;
                        reclustering_required = true;
                        
                        while reclustering_required
                            
                            try
                                % Cluster that data...
                                ClassData = Data.NormedFeatureVectors(:,ClassDataIndices);
                                [foo Centroids Indices Errors OptimalK KValidityInfo] = obj.cluster('data', ClassData');

                                % Check how many codebook vectors (nodes) we can
                                % assign to this class...
                                nClassNodes = sum(obj.ClassLabels == iClass);                        

                                iClassNode = 1;
                                iCluster = 1;                       

                                % Loop assigning nodes to clusters in
                                % this class until we run out of class nodes...
                                while iClassNode <= nClassNodes && iNode <= size(obj.ClassLabels,1)

                                    ClusterData = ClassData(:,Indices{OptimalK} == iCluster);                                    
                                    
                                    if size(unique(Indices{OptimalK}),1) ~= OptimalK
                                        error('Clustering failed.');
                                    end
                                    
                                    switch lower(obj.InitMethod)
                                        case 'dist_sample'
                                            % Assign class codebook vectors based on the
                                            % cluster distribution...
                                            obj.SOM.codebook(iNode,:) = ClusterData(:,ceil(rand * size(ClusterData,2)))';
                                            
                                        case 'dist_mean'
                                            obj.SOM.codebook(iNode,:) = mean(ClusterData,2);
                                    end

                                    iNode = iNode + 1;
                                    iClassNode = iClassNode + 1;
                                    iCluster = iCluster + 1;
                                    if iCluster > OptimalK
                                        iCluster = 1;
                                    end
                                end
                                
                                reclustering_required = false;
                                
                            catch MyError
                                
                                if iRetry <= 10                                                                                
                                    iRetry = iRetry + 1;                                    
                                    reclustering_required = true;
                                else
                                    reclustering_required = false;
                                    rethrow(MyError);
                                end
                                
                            end
                        end
                    end
            end
             

            %% SET UP -----------------------------------------------------
            %--------------------------------------------------------------
            % neighborhood radius
            obj.rini = max(obj.SOM.topol.msize)/2;

            % Useful stuff...
            obj.Ud = som_unit_dists(obj.SOM.topol); % distance between map units on the grid
            obj.CostMatrix = ((obj.Ud+1).^2) / max(max((obj.Ud+1).^2));
            [munits obj.dim] = size(obj.SOM.codebook);
            obj.mu_x_1 = ones(munits,1);     % this is used pretty often
            obj.ZeroMask = zeros(size(obj.SOM.mask));
            obj.OnesMask = ones(size(obj.SOM.mask));
            
            obj.KNN_max_k = min(obj.KNN_max_k, numel(obj.Size));
            
            % initialize random number generator
            % rand('twister',sum(100*clock));
            
            % Accuracy histogram initialization...
            obj.AccuracyHist = zeros(size(obj.SOM.codebook,1),1);
            
            %% CREATE AuxDistStats OBJECT from RunningStat CLASS ----------
            %--------------------------------------------------------------
            obj.AuxDistStats = RunningStat();
            
        end
        
        %% ------- *** SET PROPERTIES *** ---------------------------------
        %******************************************************************
        %******************************************************************
        function obj = set(obj, varargin)
            
            % Loop through arguments...
            i = 1;
            iPassedArgs = 1;
            while i <= length(varargin), 
                argok = 1; 
                if ischar(varargin{i}), 
                    switch lower(varargin{i}), 
                        case {'codebook_size', 'size'},...
                                i=i+1; obj.Size = varargin{i}; obj.rini = max(obj.Size)/2;
                        case {'codebook_lattice', 'lattice'}, i=i+1; obj.Lattice = varargin{i};
                        case {'codebook_shape', 'shape'}, i=i+1; obj.Shape = varargin{i};
                        case {'codebook_neigh', 'neigh'}, i=i+1; obj.Neigh = varargin{i};
                        case {'codebook_mask', 'mask'}, i=i+1; obj.SOM.mask = varargin{i};
                        case {'codebook_init_method'}, i=i+1; obj.InitMethod = varargin{i};
                        case 'radius_type',  i=i+1; obj.radius_type = varargin{i};
                        case {'metric_type', 'metric'}, i=i+1; obj.metric_type = lower(varargin{i});                        
                        case 'trainlen', i=i+1; obj.trainlen = varargin{i};
                        case 'knn_max_k', i=i+1; obj.KNN_max_k = varargin{i};
                        case {'norm', 'comp_norm', 'normalization_struct'}, i=i+1; obj.NormStruct = varargin{i};
                        case {'normalization_method'}, i=i+1; obj.NormMethod = varargin{i};
                        case 'nclasses', i=i+1; obj.nClasses = varargin{i};
                        case {'featureselectionfeedback', 'feature_selection_feedback',...
                              'featureselectionintraining', 'feature_selection_in_training'},...
                                i=i+1; obj.feature_selection_feedback = varargin{i};
                        case 'record', i=i+1; obj.record = varargin{i};
                        case 'display', i=i+1; obj.display = varargin{i};
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
                    disp(['CodebookModality.set(): Ignoring invalid argument #' num2str(i)]);
                    fprintf(obj.UsageMessage);
                end

                i = i + 1;
            end
            
            %% CREATE metric HANDLE --------------------------------------
            %--------------------------------------------------------------
            if iscell(obj.metric_type)
                metric_type = obj.metric_type{1};
            else
                metric_type = obj.metric_type;
            end
            
            switch metric_type,
                case {'squared', 'sumsquared'},
                    if obj.feature_selection_feedback
                        obj.metric = @obj.sumsquared_metric_withmask;
                    else
                        obj.metric = @obj.sumsquared_metric;
                    end
                    
                    obj.metric_withmask = @obj.sumsquared_metric_withmask;
                    
                case {'euclidean'},
                    if obj.feature_selection_feedback
                        obj.metric = @obj.euclidean_metric_withmask;
                    else
                    	obj.metric = @obj.euclidean_metric;
                    end
                    
                    obj.metric_withmask = @obj.euclidean_metric_withmask;
                    
                otherwise
                    if obj.feature_selection_feedback
                        obj.metric = @obj.sumsquared_metric_withmask;
                    else
                    	obj.metric = @obj.sumsquared_metric;
                    end
                    
                    obj.metric_withmask = @obj.sumsquared_metric_withmask;
            end
            
            %% CREATE ALGORITHM OBJECT ------------------------------------
            %--------------------------------------------------------------
            if exist('PassedArgs', 'var')
                obj.Algo = obj.createAlgorithm('record', obj.record, PassedArgs{1:end});
            end
            
        end
        
        %% ------- *** TRAIN *** ------------------------------------------
        %******************************************************************
        %******************************************************************
        function obj = train(obj, Data, varargin)
            
            obj.Algo.run(obj, Data, varargin);
            
        end
        
        %% ------- *** CLASSIFY *** ---------------------------------------
        %******************************************************************
        %******************************************************************
        function [Matches NormedFeatureVectors] = classify(obj, Data, varargin)
            
            % Defaults...
            wise = 'nodewise';
            Codebook = obj.SOM;
            Mask = [];
            WhichBMUs = 'best';
            
            % Loop through arguments...
            i = 1;
            while i <= length(varargin), 
                argok = 1; 
                if ischar(varargin{i}), 
                    switch lower(varargin{i}), 
                        % argument IDs
                        case {'nodewise', 'node_wise'}, i=i+1; wise = 'nodewise';
                        case {'clusterwise', 'cluster_wise'}, i=i+1; wise = 'clusterwise';
                        case {'codebook'},  i=i+1; Codebook = varargin{i};
                        case {'mask', 'featuremask', 'feature_mask'},  i=i+1; Mask = varargin{i};
                        case {'whichbmus'},  i=i+1; WhichBMUs = varargin{i};
                            
                        otherwise
                            argok = 0;
                    end
                else
                    argok = 0;
                end

                if ~argok, 
                    disp(['CodebookModality.classify(): Ignoring invalid argument #' num2str(i)]);
                    fprintf(obj.UsageMessage);
                end

                i = i + 1;
            end
            
            %% CONVERT & NORMALIZE TEST DATA ------------------------------
            %--------------------------------------------------------------
            if isstruct(Data)
                if isfield(Data, 'NormedFeatureVectors')
                    SOMTestData = som_data_struct(Data.NormedFeatureVectors');
                else                    
                    SOMTestData = som_data_struct(Data.FeatureVectors');
                    SOMTestData = som_normalize(SOMTestData, obj.NormStruct);
                end
            else
                SOMTestData = som_data_struct(Data');
                SOMTestData = som_normalize(SOMTestData, obj.NormStruct);
            end
            
            % Return normalized feature vectors...
            NormedFeatureVectors = SOMTestData.data';
            
            switch wise
                case 'nodewise'
                    
                    %% NORMALIZE THE FEATURE MASK IF NECESSARY ------------
                    %------------------------------------------------------
                    if isempty(Mask)
                        Mask = ones(size(obj.SOM.mask)) ./ size(obj.SOM.mask, 1);
                    end
            
                    %% FIND BEST MATCHING UNITS IN SOM --------------------
                    %------------------------------------------------------
                    switch WhichBMUs
                        case 'best', nBMUs = 1;
                        case 'all', nBMUs = size(Codebook, 1);
                    end

                    for iTestData = 1:size(SOMTestData.data,1)
                        Matches(iTestData,:) = obj.findbmus_static(Codebook, SOMTestData.data(iTestData,:), nBMUs, Mask);
                    end
                    
                case 'clusterwise'
                    
                    %% FIND TEST DATA TO NODE DISTANCES --------------------
                    %------------------------------------------------------
                    % Distances between each of the test samples and the
                    % map nodes...
                    TestDataMapDistances = som_eucdist2(Codebook, SOMTestData);
                    
                    % TestSampleWinningClusters
                    % This KNN method messes up sometimes when there's only
                    % one class, so we bypass it if that is the case.
                    % More efficient that way anyway.
                    Unique = unique(obj.Clustering.Labels{obj.Clustering.OptimalK});
                    if length(Unique) > 1
                        [Matches,P] =...
                            knn(TestDataMapDistances',...
                                obj.Clustering.Labels{obj.Clustering.OptimalK},...
                                obj.KNN_max_k);
                    else
                        Matches = ones(size(TestDataMapDistances',1), obj.KNN_max_k) * Unique(1);
                    end
            end
            
        end
        
        %% ------- *** CLUSTER *** ----------------------------------------
        %******************************************************************
        %******************************************************************
        function [obj Centroids Indices Errors OptimalK KValidityInfo] = cluster(obj, varargin)
            
            % Defaults...
            Data = obj.SOM.codebook;
            save_cluster_info = true;
            
            % Loop through arguments...
            i = 1;
            while i <= length(varargin), 
                argok = 1; 
                if ischar(varargin{i}), 
                    switch lower(varargin{i}), 
                        % argument IDs
                        case {'data'}, i=i+1;
                            Data = varargin{i};
                            save_cluster_info = false;
                            
                        otherwise
                            argok = 0;
                    end
                else
                    argok = 0;
                end

                if ~argok, 
                    disp(['CodebookModality.cluster(): Ignoring invalid argument #' num2str(i)]);
                    fprintf(obj.UsageMessage);
                end

                i = i + 1;
            end
            
            [Centroids, Indices, Errors, DB_Indices] =...
                kmeans_clusters(Data); % find clusterings
% 
%             [dummy,best_DB_k] = min(ind); % find the cluster with smallest Davies-Bouldin index
%             oparcObject.Classifier.KMEANS_DB_best_k = best_DB_k;

            Dmatrix = similarity_euclid(Data);

            for k = 1:length(Indices)

                [DB(k),CH(k),Dunn(k),KL(k),Han(k),st] =...
                    valid_internal_deviation(Data, Indices{k}, 1);
                
%                 S = ind2cluster(Indices{k});                
%                 [Hom(k), Sep(k), Cindex(k), wtertra(k)] = ... %, Dunn(k), DB(k)
%                      valid_internal_intra(Dmatrix, S, 1, false);
                 
                Silly = silhouette(Data, Indices{k}, 'euclidean');
                Sil(k) = mean(Silly);
                
            end
            %% Davies-Bouldin
            % Minimum value determines the optimal number of clusters [Bolshakova et al. 2003; Dimitriadou et al. 2002].
            [foo bar] = min(DB(2:length(DB)));
            Winners(1) = bar + 1;
            KValidityInfo{1}.Test = 'Davies-Bouldin';
            KValidityInfo{1}.Result = bar + 1;
            
            %% Calinski-Harabasz
            % Maximum value indicates optimal NC [Dudoit et al. 2002].
            [foo bar] = max(CH(2:length(CH)));
            Winners(2) = bar + 1;
            KValidityInfo{2}.Test = 'Calinski-Harabasz';
            KValidityInfo{2}.Result = bar + 1;
            
            %% Dunn index
            % Maximum value indicates optimal NC [Bolshakova et al. 2003; Halkidi et al. 2001].
            [foo bar] = max(Dunn(2:length(Dunn)));
            Winners(3) = bar + 1;
            KValidityInfo{3}.Test = 'Dunn';
            KValidityInfo{3}.Result = bar + 1;
            
            %% Krzanowski-Lai index
            % maximum value indicates optimal NC [Dudoit et al. 2002].
            [foo bar] = max(KL(2:length(KL)));
            Winners(4) = bar + 1;
            KValidityInfo{4}.Test = 'Krzanowski-Lai';
            KValidityInfo{4}.Result = bar + 1;
            
            %% C index (Hubert-Levin)
            % Minimal C-index indicates optimal NC [Bolshakova et al. 2003; Bolshakova et al. 2006; Dimitriadou et al. 2002].
%             [foo bar] = min(Cindex(2:length(Cindex)));
%             Winners(5) = bar + 1;
%             KValidityInfo{5}.Test = 'C index (Hubert-Levin)';
%             KValidityInfo{5}.Result = bar + 1;

            %% Silhouette index (overall average silhouette)
            % The largest silhouette indicates the optimal NC [Dudoit et al. 2002; Bolshakova et al. 2003].
            [foo bar] = max(Sil(2:length(Sil)));
            Winners(5) = bar + 1;
            KValidityInfo{5}.Test = 'Silhouette';
            KValidityInfo{5}.Result = bar + 1;
            
            %% Weighted inter-intra index
            % Search forward (k=2,3,4,...) and stop at the first down-tick
            % of the index, which indicates optimal NC [Strehl 2002].
%             [foo bar] = max(wtertra(2:length(wtertra)));
%             Winners(6) = bar + 1;
%             KValidityInfo{6}.Test = 'Weighted inter-intra index';
%             KValidityInfo{6}.Result = bar + 1;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% SELECT THE K-VALUE THAT WON MOST FREQUENTLY...
            OptimalK = mode(Winners);
            
            
            %% SAVE THIS STUFF IN THE OBJECT...
            if save_cluster_info
                obj.Clustering.Centroids = Centroids;
                obj.Clustering.Labels = Indices;
                obj.Clustering.Errors = Errors;
                obj.Clustering.OptimalK = OptimalK;
                obj.Clustering.ValidityInfo = KValidityInfo;
            end
            
        end
        
        
        %% ------- *** CREATEALGORITHM *** --------------------------------
        % *****************************************************************
        % Factory method for algorithm creation.
        % *****************************************************************
        function Algo = createAlgorithm(obj, varargin)
            
            % Defaults...
            Updaters = {'SOM'};
            
            % Loop through arguments...
            i = 1;
            iPassedArgs = 1;
            while i <= length(varargin), 
                argok = 1; 
                if ischar(varargin{i}), 
                    switch lower(varargin{i}), 
                        case {'updaters', 'updater'},...
                                i=i+1; Updaters = varargin{i};
                        
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
                    disp(['CodebookModality.createAlgorithm(): Ignoring invalid argument #' num2str(i)]);
                    fprintf(obj.UsageMessage);
                end

                i = i + 1;
            end
            
            %% NORMALIZE THE FEATURE MASK ---------------------------------
            %--------------------------------------------------------------
            obj.SOM.mask = obj.SOM.mask ./ norm(obj.SOM.mask, 1);
            
            %% SET UP MODALITY CONDITIONS FOR VARIOUS ALGORITHMS ----------
            %--------------------------------------------------------------
            for i = 1:length(Updaters)
                
                % For supervised algorithms, we need to pre-label
                % the codebook vectors...
                switch lower(Updaters{i})                                        
                    case {'lvq1', 'lvq', 'olvq1', 'olvq',...
                          'rlvq', 'rlvq1', 'orlvq1', 'orlvq',...
                          'laorlvq', 'ldalvq1', 'lvq3',...
                          'glvq', 'grlvq',...
                          'ldalvq1', 'ldalvq1_3', 'ldaolvq', 'ldaolvq2', 'ldaolvq3',...
                          'ldaglvq', 'ldaglvq_3', 'ldaoglvq', 'ldaoglvq_3',...
                          'laldaolvq', 'lcaldaolvq'},
                        obj.ClassLabels = zeros(size(obj.SOM.codebook,1),1);
                        increment = floor(size(obj.SOM.codebook,1) / obj.nClasses);
                        for j = 1:obj.nClasses
                            obj.ClassLabels( ((j-1) * increment) + 1 : max(j * increment, size(obj.SOM.codebook,1))) = j;                                       
                        end
                end
                    
                % For algorithms that require the first 2 BMUs,
                % we need to alter the findbmu method appropriately...
                switch lower(Updaters{i})
                    case {'lvq3', 'heurlvq3', 'heurmamrlvq', 'heurfmamrlvq'},...
                        obj.nBMUs = 2;                        
                end
                    
                % Similarly, for GLVQ-style algorithms and some locally
                % adaptive algorithms, we grab ALL of
                % the BMUs, since such algorithms require both the BMU
                % for the correct class and the BMU for the closest
                % incorrect class...
                switch lower(Updaters{i})
                    case {'glvq', 'grlvq', 'ldaglvq', 'ldaglvq_3', 'ldalvq1_3', 'ldaolvq3', 'ldaoglvq', 'ldaoglvq_3',...
                          'laldaglvq', 'laldaolvq', 'lcaldaolvq',...
                          'heurfldaolvq', 'heurfldaolvq3', 'heurgrlvq'}
                        obj.nBMUs = size(obj.SOM.codebook, 1);
                end

                % Some update rules require setting up
                % a RunningStat object for recording a
                % running mean and variance for each class...
                switch lower(Updaters{i})                        
                    case {'ldaolvq2'}                                                                        
                        for iClass = 1:obj.nClasses
                            obj.ClassStats{iClass} = RunningStat();
                        end
                end
                
                % Some cross-modal heuristic update rules will require
                % auxiliary distances for each node...
                switch lower(Updaters{i})
                    case {'heurfldaolvq', 'heurfldaolvq3', 'heurgrlvq'}
                        obj.nAuxDists = size(obj.SOM.codebook, 1);
                end
                
                % Some update rules require setting up
                % RunningStat objects for recording a
                % running mean and variance for each node...
                switch lower(Updaters{i})
                    case {'ldalvq1', 'ldalvq1_3', 'ldaolvq3',...
                          'heurfldaolvq3', 'heurforlvq', 'heurorlvq1', 'heurrlvq1',...
                          'ldaglvq_3', 'ldaoglvq_3',...
                          'lcaldaolvq'}
                        for iNode = 1:size(obj.SOM.codebook,1)
                            obj.NodeStats{iNode} = RunningStat();
                        end
                end                                                                                                    
            
                % Set up a RunningStat object for recording a
                % running average of the feature mask...
                obj.MaskStats = RunningStat();
                
            end
                        
            %% CREATE ALGORITHM OBJECT ------------------------------------
            %--------------------------------------------------------------
            Algo = CodebookAlgorithm(obj, 'updaters', Updaters, PassedArgs{1:end});
            
        end
        
        %% ------- *** FAST METHODS FOR FINDING BMU *** -------------------
        %******************************************************************
        % WARNING: The following methods assume that 'Data' is a single
        % feature vector and is already normalized!
        %******************************************************************
        
        function BMUs = findbmus(obj, Data)
            
            obj.Dx = obj.SOM.codebook - Data(obj.mu_x_1, :);
            
            % findbmus doesn't know if feature selection feedback will be
            % used, so we just use the general metric function pointer...
            obj.Distances = obj.metric(obj.Dx, obj.SOM.mask);
            
            %% Find & save BMUs (faster than full sort?)...
            TempDists = obj.Distances;
            
            for iBMU = 1:obj.nBMUs               
                [qerr bmu] = min(TempDists);  % Find BMU i
                BMUs(iBMU) = bmu;             % Return BMU i...
                
                TempDists(bmu) = Inf;
            end
            
        end
        
        function BMUs = findbmus_static(obj, Codebook, Data, nBMUs, Mask)
            
            Dx = Codebook - Data(ones(size(Codebook,1),1), :);
            
            % findbmus_static definitely gets passed a mask, so we should
            % use the metric_withmask function pointer...
            Distances = obj.metric_withmask(Dx, Mask);
            
            %% Find & save BMUs (faster than full sort?)...            
            for iBMU = 1:nBMUs               
                [qerr bmu] = min(Distances);  % Find BMU i
                BMUs(iBMU) = bmu;             % Return BMU i...
                
                Distances(bmu) = Inf;
            end
            
        end
        
        %% ------- *** CLEARBMUS *** --------------------------------------
        % *****************************************************************
        % *****************************************************************
        function obj = clearbmus(obj)
            
            obj.BMUs = [];
            
        end
    end
    
    methods (Access = private)                
        
        function Dists = sumsquared_metric(obj, Dx, varargin)
            Dists = Dx.^2 * obj.OnesMask;
        end
        
        function Dists = sumsquared_metric_withmask(obj, Dx, Mask)
            Dists = Dx.^2 * Mask;
        end
        
        function Dists = euclidean_metric(obj, Dx, varargin)
            Dists = sqrt(Dx.^2 * obj.OnesMask);
        end
        
        function Dists = euclidean_metric_withmask(obj, Dx, Mask)
            Dists = sqrt(Dx.^2 * Mask);
        end                
        
    end
    
end
