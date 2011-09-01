% Author: Markus Bader 2010
% email: markus.bader@tuwien.ac.at



classdef NaoQi < handle
    %NAOQI Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    properties (GetAccess='public', SetAccess='private')
        ptrNaoQi;
    end
    
    methods
        function obj=NaoQi(ip, port)
            AK_MATLAB = [getenv('AK_DIR') '/build/local/matlab/'];
            AK_MEX = [AK_MATLAB '/mex/nao'];
            addpath(AK_MEX);
            obj.ptrNaoQi = mex_naoqi(ip, port);
        end
        function delete(obj)
            mex_naoqi_close(obj.ptrNaoQi);
        end 
        function close(obj)
            mex_naoqi_close(obj.ptrNaoQi);
            obj.ptrNaoQi = 0;
        end
        function reconnect(obj, ip, port)
            obj.ptrNaoQi = mex_naoqi_connect(obj.ptrNaoQi, ip, port);
        end
        function setStiffnesses(obj, varargin)
            % arguments are in pairs of two
            % name1, value1, name2, value2, ...
            if (nargin < 3)   
                error('NaoQi:argChk', 'Wrong number of input arguments')
            end
            mex_setStiffnesses(obj.ptrNaoQi,varargin{:})
        end
        function setWalkTargetVelocit(obj, x, y, theta, frequency)
            mex_setWalkTargetVelocity(obj.ptrNaoQi, x, y, theta, frequncy)
        end
        function walkTo(obj, x, y, theta, post)
            if (nargin < 5)   
                post = 0;
            end
            mex_walkTo(obj.ptrNaoQi, x, y, theta, post)
        end
        function active = walkIsActive(obj)
            %Returns >0 if the walk is active
            active = mex_walkIsActive(obj.ptrNaoQi);
        end
        function stiffnessInterpolation(obj, varargin)
            % arguments are in pairs of three
            % name1, value1, time1, name2, value2, time2...
            if (nargin < 4)   
                error('NaoQi:argChk', 'Wrong number of input arguments')
            end
            mex_stiffnessInterpolation(obj.ptrNaoQi,varargin{:})
        end
        function setAngles(obj, varargin)
            % arguments are in pairs of two and the fractionMaxSpeed and
            % post as last two option
            % name1, angle1, time1, name2, ... , maxSpeed, post 
            if (nargin < 4)   
                error('NaoQi:argChk', 'Wrong number of input arguments')
            end
            mex_setAngles(obj.ptrNaoQi,varargin{:})
        end
        function angles = getAngles(obj, varargin)
            % Gets the angles of the joints
            % post as last two option
            % name1, name2, 
            if (nargin < 2)   
                error('NaoQi:argChk', 'Wrong number of input arguments')
            end
            angles = mex_getAngles(obj.ptrNaoQi,varargin{:});
        end
        function [T R] = getPosition(obj, name, space, useSensorValues)
            if (nargin ~= 4)   
                error('NaoQi:argChk', 'Wrong number of input arguments')
            end
            pos = mex_getPosition(obj.ptrNaoQi, name, space, boolean(useSensorValues));
            T = pos(1:3);
            R = pos(4:6);
        end
        function I = getCameraImage(obj, resolution, colorSpace, fps, camera, data_format)
            % resolution 	Resolution requested. { 0 = kQQVGA, 1 = kQVGA, 2 = kVGA }
            % colorSpace 	Colorspace requested. { 0 = kYuv, 9 = kYUV422, 10 = kYUV, 11 = kRGB, 12 = kHSY, 13 = kBGR }
            % fps           Fps (frames per second) requested. { 5, 10, 15, 30 } 
            % camera        Camera { 0 = CameraTop, 1 = CameraBottom } 
            % data_format   data_format { 1 == Matlab, 0 = C++ } 
            if (nargin < 5) || (nargin > 6)    
                error('NaoQi:argChk', 'Wrong number of input arguments')
            end
            if nargin == 5
                data_format = 1
            end
            I = mex_getCameraImage(obj.ptrNaoQi, resolution, colorSpace, fps, camera, data_format);
        end
        function say(obj, text, post)            
            if (nargin ~= 2) && (nargin ~= 3)
                error('NaoQi:argChk', 'Wrong number of input arguments')
            end
            if (nargin == 2)
                mex_say(obj.ptrNaoQi, text);
            else
                mex_say(obj.ptrNaoQi, text, post);
            end
        end
    end
    
end

