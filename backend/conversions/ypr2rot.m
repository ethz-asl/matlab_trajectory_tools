function R = ypr2rot(y, p, r)
%compute rotation matrix from yaw pitch roll angles

% TODO(alexmillane): Add support for yaw pitch roll trajectories. As mbosse
% below

% angles = reshape(angles,[3 1 n]);

% cphi = cos(angles(1,1,:));
% sphi = sin(angles(1,1,:));
% cthe = cos(angles(2,1,:));
% sthe = sin(angles(2,1,:));
% cpsi = cos(angles(3,1,:));
% spsi = sin(angles(3,1,:));
% 
% R = [
%    cthe.*cpsi, cthe.*spsi, -sthe;
%    sphi.*sthe.*cpsi-cphi.*spsi, sphi.*sthe.*spsi+cphi.*cpsi, sphi.*cthe;
%    cphi.*sthe.*cpsi+sphi.*spsi, cphi.*sthe.*spsi-sphi.*cpsi, cphi.*cthe;
% ];

cy = cos(y);
sy = sin(y);
cp = cos(p);
sp = sin(p);
cr = cos(r);
sr = sin(r);

R = [   cy*cp   cy*sp*sr-cr*sy  sy*sr+cy*cr*sp ;
        cp*sy   cy*cr+sy*sp*sr  cr*sy*sp-cy*sr ;
        -sp     cp*sr           cp*cr          ; ];