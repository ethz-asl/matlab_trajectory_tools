function c = k_tf_mult(a, b)
% Transformations are [x y z q_w q_x q_y q_z].
%compose two 6DOF configurations

Na = size(a,1);
Nb = size(b,1);
if Na==1 && Nb>Na
  a = repmat(a,Nb,1);
end
if Nb==1 && Na>Nb
  b = repmat(b,Na,1);
end


%c = zeros(max([size(a); size(b)]));
c = a; %zeros(size(a));

c(:, 1:3) = a(:, 1:3) + k_quat_rotate(a(:, 4:7), b(:, 1:3));
c(:, 4:7) = k_quat_mult(a(:, 4:7), b(:, 4:7));  
end
