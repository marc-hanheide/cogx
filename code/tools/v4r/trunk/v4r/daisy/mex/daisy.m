function [desc] = daisy(I, varargin)
% DAISY Calculates daisy descriptors
%  
% DESC = DAISY(I, OPTIONS) calculates the daisy descriptors of the
% image stored at the location I using the implementation of Engin
% Tola.
%
% Options:
%   'R'     radius of the descriptor at the initial scale. default: 15
%   'RQ'    number of pieces the radial range will be divided. default: 3
%   'THQ'   number of orientations. default: 8;
%   'HQ'    number of pieces the gradient histogram will be divided.
%           default: 8
%
% Returns:
%   desc    a dense descriptor for each pixel of the image. The dimension
%           of desc is L x W x H where 
%           L   is the length of the descriptor: ((RQ * THQ)+1)*HQ
%           W   the number of columns of the image
%           H   the number of rows of the image
%
% N.B.: R has to be a numeric; RQ, OQ, and HQ have to be integer.
%
% Usage example:
% desc = daisy('test.png','R',18.5,'RQ',3,'OQ',10,'HQ',8)
% or
% I = imread('test.png')
% desc = daisy(I,'R',18.5,'RQ',3,'OQ',10,'HQ',8)
%
% For details on the DAISY descriptor see:
%   E. Tola, V. Lepetit, P. Fua, (2007) "A Fast Local Descriptor for Dense
%   Matching" (TR EPFL/CVLAB2007.08), École Polytechnique Fédérale de
%   Lausanne, Switzerland.
%
%   E. Tola, V. Lepetit, P. Fua, (2008) "A Fast Local Descriptor for Dense
%   Matching", Proc. of the IEEE Conf. on Computer Vision and Image
%   Processing (CVPR08), Anchorage, Alaska, USA.
%
% Author: pb (Philipp Blauensteiner)
% Version: 0.1
%
% Author: mb (Markus Bader)
% Version: 0.2
% Dealing directly with matlab images

    % set default
    sOpt.R = 15;
    sOpt.RQ = 3;
    sOpt.THQ = 8;
    sOpt.HQ = 8;

    %process input
    if (nargin < 1)
        error('Usage error! Please specify image file');
    end
    
    if (ischar(I))
        if (~exist(I,'file'))
            error(['Image File ' I 'does not exist.']);
        end
        I = imread(I);
    end

    argCnt = length(varargin);
    idx = 1;
    while (idx < argCnt)
        switch (lower(varargin{idx}))
            case 'r'
                idx = idx+1;
                if (~isnumeric(varargin{idx}))
                    error('the value of R must be numeric');
                end
                sOpt.R = varargin{idx};
            case 'rq'
                idx = idx+1;
                if (~isnumeric(varargin{idx})||round(varargin{idx})~=varargin{idx})
                    error('the value of RQ must be an integer');
                end
                sOpt.RQ = varargin{idx};
            case 'thq'
               idx = idx+1;
                if (~isnumeric(varargin{idx})||round(varargin{idx})~=varargin{idx})
                    error('the value of THQ must be an integer');
                end
                sOpt.THQ = varargin{idx};
            case 'hq'
                idx = idx+1;
                if (~isnumeric(varargin{idx})||round(varargin{idx})~=varargin{idx})
                    error('the value of HQ must be an integer');
                end
                sOpt.HQ = varargin{idx};
        end
        idx = idx+1;
    end
    
    desc = mex_daisy(I, sOpt);
end