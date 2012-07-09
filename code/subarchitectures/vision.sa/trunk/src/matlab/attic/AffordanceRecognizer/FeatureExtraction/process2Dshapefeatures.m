% process2Dshapefeatures
%
% Load binary image containing an object, process it for shape features
% and save the features in both a MAT file and a text file in its directory.
%
% Possible syntax:
% process2Dshapefeatures(Filespec)
%
% Input: Filespec = Filename of 2D Binary Image containing an object.
%

function [Features, FeatureNames] = process2Dshapefeatures(FileSpec, varargin)

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

        Image = imread(FileSpec);
    else
        Image = FileSpec;
    end

    [Features, FeatureNames] = getallshapefeatures(Image);

    if save_ascii
        save(strcat(DirName, FileName, '_2DShapeFeatures.txt'), '-ascii',...
             'Features', 'FeatureNames');
    end

    if save_mat
        save(strcat(DirName, FileName, '_2DShapeFeatures.mat'), '-mat',...
             'Features', 'FeatureNames');
    end
