function OutFGVTX = normalizepoints(FGVTX)

    FGVTX_X = FGVTX(:,1);
    FGVTX_Y = FGVTX(:,2);
    FGVTX_Z = FGVTX(:,3);
    
    % N is the normal to the plane
    [N,D] = fitPlane(FGVTX_X, FGVTX_Y, FGVTX_Z);
    
    % Normalize N...
    N_norm = N/norm(N);
    
    % Get the rotation axis using cross product...
    RotAxis = cross([0,0,1], N_norm);
    RotAxis_norm = RotAxis / norm(RotAxis);
    
    % Get the angle of rotation...
    Angle = acos(dot([0,0,1], N_norm));
    % Angle = asin(norm(RotAxis));
    % Angle = atan2(norm(RotAxis_norm),dot(N_norm,[0,0,1]));
    
    % Build the Axis-Angle rotation matrix...
    R = build_angle_axis_rotation_matrix(RotAxis_norm, Angle);
    
    % Find the mean centroid of the foreground points...
    MeanCentroid = mean(FGVTX);
    
    % Translate the foreground cluster to the origin & rotate it...
    T_FGVTX = FGVTX - repmat(MeanCentroid, size(FGVTX,1), 1);
    
    % Rotate all the foreground points... old method.
%     for i = 1:size(T_FGVTX,1)
%         vc = cross(T_FGVTX(i,:),RotAxis_norm);
%         OutFGVTX(i,:) = dot(T_FGVTX(i,:),RotAxis_norm)*RotAxis_norm + cos(Angle)*cross(RotAxis_norm,vc) + sin(Angle)*vc;
%     end

    % Rotate all the foreground points... axis-angle, vectorized.
    OutFGVTX = T_FGVTX * R;
    
    % Translate the background cluster to the origin & rotate it...
    % T_BGVTX = BGVTX - repmat(MeanCentroid, size(BGVTX,1), 1);
    
    % Rotate all the background points... old method.
%     for i = 1:size(T_BGVTX,1)
%         vc = cross(T_BGVTX(i,:),RotAxis_norm);
%         OutBGVTX(i,:) = dot(T_BGVTX(i,:),RotAxis_norm)*RotAxis_norm + cos(Angle)*cross(RotAxis_norm,vc) + sin(Angle)*vc;
%     end

    % Rotate all the background points... axis-angle, vectorized.
    % OutBGVTX = T_BGVTX * R;
    
    function R = build_angle_axis_rotation_matrix(Axis, angle)
        x = Axis(1);
        y = Axis(2);
        z = Axis(3);
        R = zeros(3,3);
        
        R(1,1) = 1 + (1-cos(angle))*(x*x-1);
        R(1,2) = -z*sin(angle)+(1-cos(angle))*x*y;
        R(1,3) = y*sin(angle)+(1-cos(angle))*x*z;
        R(2,1) = z*sin(angle)+(1-cos(angle))*x*y;
        R(2,2) = 1 + (1-cos(angle))*(y*y-1);
        R(2,3) = -x*sin(angle)+(1-cos(angle))*y*z;
        R(3,1) = -y*sin(angle)+(1-cos(angle))*x*z;
        R(3,2) = x*sin(angle)+(1-cos(angle))*y*z;
        R(3,3) = 1 + (1-cos(angle))*(z*z-1);