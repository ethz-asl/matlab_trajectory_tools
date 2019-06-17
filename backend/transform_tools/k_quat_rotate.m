function rv = k_quat_rotate(q,v)
%QUAT_ROTATE rotates a vector by a quaternion
% Quaternions are w x y z, active.

% Normal -. Mbosse
q = q';
v = v';

if size(q,1) == 1
  rv = squeeze(quat2rot(q)) * v;
else

  rv = zeros(3,size(q,2));
  rv(1,:) = (q(1,:).^2+q(2,:).^2-q(3,:).^2-q(4,:).^2).*v(1,:)+...
            (2.0.*(q(2,:).*q(3,:)-q(1,:).*q(4,:))).*v(2,:)+...
            (2.0.*(q(2,:).*q(4,:)+q(1,:).*q(3,:))).*v(3,:);
     
  rv(2,:) = (2.0.*(q(2,:).*q(3,:)+q(1,:).*q(4,:))).*v(1,:)+...
            (q(1,:).^2-q(2,:).^2+q(3,:).^2-q(4,:).^2).*v(2,:)+...
            (2.0.*(q(3,:).*q(4,:)-q(1,:).*q(2,:))).*v(3,:);
  
  rv(3,:) = (2.0.*(q(2,:).*q(4,:)-q(1,:).*q(3,:))).*v(1,:)+...
            (2.0.*(q(3,:).*q(4,:)+q(1,:).*q(2,:))).*v(2,:)+...
            (q(1,:).^2-q(2,:).^2-q(3,:).^2+q(4,:).^2).*v(3,:);

  %qv = [zeros(1,size(v,2)); v];
  %qrv = mul_quat(q, mul_quat(qv, diag([1 -1 -1 -1])*q));
  %rv = qrv(2:4,:);
end
rv = rv';
end
