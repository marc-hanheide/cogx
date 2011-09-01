close all;
files = dir;
images = cell(0);

for i=1:length(files)
    if(strcmp(files(i).name(max(end-4, 1):end), '.jpeg'))
        images{end+1} = files(i).name;
    end
end

img1 = imread(images{1});
tic
Desc1 = daisy(img1);
toc


img2 = imread(images{10});
tic
Desc2 = daisy(img2);
toc

for i=2:length(images)
    
    
end
