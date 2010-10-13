% process3Dshapefeatures
%

function [Features, FeatureNames] = process3Dshapefeatures(FileSpec, varargin)

    % Defaults...
    save_ascii = false;
    save_mat = false;

    % Loop through arguments...
    i = 1;
    iPassedArgs = 1;
    while i <= length(varargin), 
        argok = 1; 
        if ischar(varargin{i}), 
            switch lower(varargin{i}), 
                case {'savetxt', 'saveascii', 'save_txt', 'save_ascii'},...
                        i=i+1; save_ascii = varargin{i};
                case {'savemat', 'save_mat'},...
                        i=i+1; save_mat = varargin{i};
                    
            end
        else
            argok = 0;
        end

        if ~argok,
            [ST,I] = dbstack;
            disp([ST.name '(): Ignoring invalid argument #' num2str(i)]);
            % fprintf(obj.UsageMessage);
        end

        i = i + 1;
    end

    if ischar(FileSpec)
        
        [FileName,DirName] = getfilename(FileSpec);
        
        if isempty(findstr(FileName, '.'))
            
            FileNames = dir([DirName FileName '*']);
            
            for iFile = 1:length(FileNames)
                [Foo Patches{iFile}] = hdrload([DirName FileNames(iFile).name]);
            end
            
        else            
            [h1 Points]= hdrload(FileSpec);            
        end
        
    else
        DirName = './';
        FileName = 'Points';
        
        Points = FileSpec;
        
        nPatches = Points(1,1);
        iPatchPointer = 1+nPatches+1;
        for iPatch = 1:nPatches            
            nPatchPoints(iPatch) = Points(1+iPatch,1);
            Patches{iPatch} = Points(iPatchPointer:iPatchPointer+nPatchPoints(iPatch),:);
            iPatchPointer = iPatchPointer+nPatchPoints(iPatch)+1;
        end
    end
    
    % Find the dominant patch...
    most_points = 0;
    for iPatch = 1:length(Patches)
        if size(Patches{iPatch},1) > most_points
            most_points = size(Patches{iPatch},1);
            Points = Patches{iPatch};
        end
    end
    
    % Fit a plane to the patch points, then rotate level with the
    % Z-plane...
    PointsNorm = normalizepoints_no_bg(Points(:,(end-2):end));
    
    % Re-scale the points in the Z-direction to account for the
    % Bumblebee-captured data that the classifier was trained
    % with...
    PointsNormRescaled = PointsNorm;
    PointsNormRescaled(:,3) = PointsNorm(:,3) * 2.9730;
    
    % Remove any remaining outliers (points that deviate from the mean
    % by 3 standard deviations or more)
    Outliers = (PointsNormRescaled(:,1) < mean(PointsNormRescaled(:,1)) - 3*std(PointsNormRescaled(:,1))) |...
               (PointsNormRescaled(:,1) > mean(PointsNormRescaled(:,1)) + 3*std(PointsNormRescaled(:,1))) |...
               (PointsNormRescaled(:,2) < mean(PointsNormRescaled(:,2)) - 3*std(PointsNormRescaled(:,2))) |...
               (PointsNormRescaled(:,2) > mean(PointsNormRescaled(:,2)) + 3*std(PointsNormRescaled(:,2))) |...
               (PointsNormRescaled(:,3) < mean(PointsNormRescaled(:,3)) - 3*std(PointsNormRescaled(:,3))) |...
               (PointsNormRescaled(:,3) > mean(PointsNormRescaled(:,3)) + 3*std(PointsNormRescaled(:,3)));
           
    PointsNormRescaledEroded = PointsNormRescaled(~Outliers,:);
    
    % Let's do it twice...
    Outliers = (PointsNormRescaledEroded(:,1) < mean(PointsNormRescaledEroded(:,1)) - 3*std(PointsNormRescaledEroded(:,1))) |...
               (PointsNormRescaledEroded(:,1) > mean(PointsNormRescaledEroded(:,1)) + 3*std(PointsNormRescaledEroded(:,1))) |...
               (PointsNormRescaledEroded(:,2) < mean(PointsNormRescaledEroded(:,2)) - 3*std(PointsNormRescaledEroded(:,2))) |...
               (PointsNormRescaledEroded(:,2) > mean(PointsNormRescaledEroded(:,2)) + 3*std(PointsNormRescaledEroded(:,2))) |...
               (PointsNormRescaledEroded(:,3) < mean(PointsNormRescaledEroded(:,3)) - 3*std(PointsNormRescaledEroded(:,3))) |...
               (PointsNormRescaledEroded(:,3) > mean(PointsNormRescaledEroded(:,3)) + 3*std(PointsNormRescaledEroded(:,3)));
           
    PointsNormRescaledEroded = PointsNormRescaledEroded(~Outliers,:);

    [Foo Features] = fitsurface(PointsNormRescaledEroded);
    
    Features = Features';
    
    FeatureNames = {'EigenValue1', 'EigenValue2'};

    if save_ascii
        save(strcat(DirName, FileName, '_3DShapeFeatures.txt'), '-ascii',...
             'Features', 'FeatureNames');
    end

    if save_mat
        save(strcat(DirName, FileName, '_3DShapeFeatures.mat'), '-mat',...
             'Features', 'FeatureNames');
    end
