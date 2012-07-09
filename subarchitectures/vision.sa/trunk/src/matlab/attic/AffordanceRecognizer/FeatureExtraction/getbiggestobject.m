% getbiggestobject
%
% Grabs the biggest object from an image.
%
% Possible syntax:
% Object = getbiggestobject(Image);
%
% Input: Image = 2D Binary Image
%
% Output: Object = 2D Binary Image containing largest object
%                  from the original input image.
%

function Object = getbiggestobject(Image)

    if ndims(Image) ~= 2
        Image = rgb2gray(Image);
    end
    
    % Otsu threshold...
    threshold = graythresh(Image);
    Image = im2bw(Image, threshold);
    
    % Remove all objects containing <30 pixels...
    Image = bwareaopen(Image, 30);
    
    % Fill holes...
    % Image = imfill(Image,'holes');
    
    % Label the image...
    Image = bwlabel(Image);
    
    % Grab the labeled portion with largest area...
    for i=1:max(max(Image))
        LabeledAreas(i) = sum(sum(Image==i));
    end
    [largestobjectsize, largestobjectindex] = max(LabeledAreas);
    
    Object = (Image==largestobjectindex);
    
    