function c = k_tf_transform(t, v)
% Transformations are [x y z q_w q_x q_y q_z].
% Transform vector v by transform t.

Nt = size(t,1);
Nv = size(v,1);
if Nt==1 && Nv>Nt
  t = repmat(t,Nv,1);
end
if Nv==1 && Nt>Nv
  v = repmat(v,Nt,1);
end

c = k_quat_rotate(t(:, 4:7), v) + t(:, 1:3);
end
