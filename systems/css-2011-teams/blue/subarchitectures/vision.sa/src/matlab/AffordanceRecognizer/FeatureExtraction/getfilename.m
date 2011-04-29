function [FileName, DirName] = getfilename(FileSpec)

    if ischar(FileSpec)
        
        Slashes = findstr(FileSpec,'/');
        
        if (isempty(Slashes))
            FileName = FileSpec;
            DirName = './';
        else
            FileName = FileSpec(Slashes(end)+1:size(FileSpec,2));
            DirName = FileSpec(1:Slashes(end));
        end
    else
        error('Please enter a file or directory name!');
    end