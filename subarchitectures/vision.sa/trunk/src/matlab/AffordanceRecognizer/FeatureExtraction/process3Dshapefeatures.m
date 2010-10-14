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
    
    % Default to the first patch as the dominant patch...
    iDominantPatch = 1;
    
    % Which patch is dominant, i.e. contains the most points?
    most_points = 0;
    
    % Loop through the patches, gathering features etc.
    for iPatch = 1:length(Patches)
        
        % Find the dominant patch...
        if size(Patches{iPatch},1) > most_points
            most_points = size(Patches{iPatch},1);
            iDominantPatch = iPatch;
        end
            
        % Fit a plane to the patch points, then rotate level with the
        % Z-plane...
        PointsNorm{iPatch} = normalizepoints_no_bg(Patches{iPatch}(:,(end-2):end));

        % Re-scale the points in the Z-direction to account for the
        % Bumblebee-captured data that the classifier was trained
        % with...
        PointsNormRescaled{iPatch} = PointsNorm{iPatch};
        PointsNormRescaled{iPatch}(:,3) = PointsNorm{iPatch}(:,3) * 2.9730;

        % Remove any remaining outliers (points that deviate from the mean
        % by 3 standard deviations or more)
        Outliers = (PointsNormRescaled{iPatch}(:,1) < mean(PointsNormRescaled{iPatch}(:,1)) - 3*std(PointsNormRescaled{iPatch}(:,1))) |...
                   (PointsNormRescaled{iPatch}(:,1) > mean(PointsNormRescaled{iPatch}(:,1)) + 3*std(PointsNormRescaled{iPatch}(:,1))) |...
                   (PointsNormRescaled{iPatch}(:,2) < mean(PointsNormRescaled{iPatch}(:,2)) - 3*std(PointsNormRescaled{iPatch}(:,2))) |...
                   (PointsNormRescaled{iPatch}(:,2) > mean(PointsNormRescaled{iPatch}(:,2)) + 3*std(PointsNormRescaled{iPatch}(:,2))) |...
                   (PointsNormRescaled{iPatch}(:,3) < mean(PointsNormRescaled{iPatch}(:,3)) - 3*std(PointsNormRescaled{iPatch}(:,3))) |...
                   (PointsNormRescaled{iPatch}(:,3) > mean(PointsNormRescaled{iPatch}(:,3)) + 3*std(PointsNormRescaled{iPatch}(:,3)));

        PointsNormRescaledEroded{iPatch} = PointsNormRescaled{iPatch}(~Outliers,:);

        % Let's do it twice...
        Outliers = (PointsNormRescaledEroded{iPatch}(:,1) < mean(PointsNormRescaledEroded{iPatch}(:,1)) - 3*std(PointsNormRescaledEroded{iPatch}(:,1))) |...
                   (PointsNormRescaledEroded{iPatch}(:,1) > mean(PointsNormRescaledEroded{iPatch}(:,1)) + 3*std(PointsNormRescaledEroded{iPatch}(:,1))) |...
                   (PointsNormRescaledEroded{iPatch}(:,2) < mean(PointsNormRescaledEroded{iPatch}(:,2)) - 3*std(PointsNormRescaledEroded{iPatch}(:,2))) |...
                   (PointsNormRescaledEroded{iPatch}(:,2) > mean(PointsNormRescaledEroded{iPatch}(:,2)) + 3*std(PointsNormRescaledEroded{iPatch}(:,2))) |...
                   (PointsNormRescaledEroded{iPatch}(:,3) < mean(PointsNormRescaledEroded{iPatch}(:,3)) - 3*std(PointsNormRescaledEroded{iPatch}(:,3))) |...
                   (PointsNormRescaledEroded{iPatch}(:,3) > mean(PointsNormRescaledEroded{iPatch}(:,3)) + 3*std(PointsNormRescaledEroded{iPatch}(:,3)));
               
        PointsNormRescaledEroded{iPatch} = PointsNormRescaledEroded{iPatch}(~Outliers,:);
        
        [Foo PatchFeatures{iPatch}] = fitsurface(PointsNormRescaledEroded{iPatch});
    
        PatchFeatures{iPatch} = PatchFeatures{iPatch}';
    end
    
    Features = PatchFeatures{iDominantPatch};
   
    FeatureNames = {'EigenValue1', 'EigenValue2'};

    if save_ascii
        save(strcat(DirName, FileName, '_3DShapeFeatures.txt'), '-ascii',...
             'Features', 'FeatureNames');
    end

    if save_mat
        save(strcat(DirName, FileName, '_3DShapeFeatures.mat'), '-mat',...
             'Features', 'FeatureNames');
    end
