function eigs = shape_3d_feature(P)

P = normalizepoints(P) ;
[solution eigs] = fitsurface(P, 1) ;

% ------------------------------------------------------------------ %
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

% ------------------------------------------------------------------ %
function  [n,d] = fitPlane(x,y,z)
% FITPLANE Fitting of a plane or hyperplane to a set
%	of points.
%	[N,D] = FITPLANE(X,Y,Z) Calculates a least
%	squares fit to the normal N  to a plane through
%	a set of points with coordinates X,Y,Z in the
%	form  N(1)*X+N(2)*Y+N(3)*Z = D.
%	Normally N is normalized so that D = 1 unless
%	it is close to zero (a plane goes near
%	coordinate system origin).
%	[N,D] = FITPLANE(X,Y) calculates a similar fit
%	to a line, while
%	[N,D] = FITPLANE(R) calculates a fit to a
%	hyper-plane. In this case each row of R
%	must correspond to coordinates of a certain
%	point in a set, such as
%	R = [x1 y1 z1 t1; x2 y2 z2 t2; ...].

%  Kirill K. Pankratov,  kirill@plume.mit.edu
%  02/09/95

tol = .01;  % Tolerance for canonocal form reduction

 % Handle input ............................
if nargin==1      % Arbitrary dimension (hyperplane)
  R = x;
  if diff(size(R)) > 0, R = R'; end

elseif nargin==2  % 2-dimensional (line)
  R = [x(:) y(:)];

else              % 3-dimensional (plane)
  R = [x(:) y(:) z(:)];

end

 % Auxillary
szR = size(R);
o = ones(szR(1),1);

 % Make offset (so that plane will not go through
 % the origin)
r0 = 2*min(R)-max(R)-1;
R = R-r0(o,:);

% Corrupt R slightly if it is singular - stupid but good enough
% if det(R)==0 
 if rank(R)<3
    R=R+rand(size(R))*eps*100;
 end;
 
 % Now the fit itself ......................
n = R\o;
d = 1+r0*n;  % Compensate offset

 % Try to reduce it to the canonical form
 % (Nx*X + Ny*Y + Nz*Z = 1) if possible
if abs(d) > tol
  n = n/d;
  d = 1;
end

