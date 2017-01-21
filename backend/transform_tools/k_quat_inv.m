function iq = k_quat_inv(q)
% Quaternions are w x y z, active.
q = q';
iq = [q(1,:);-q(2:4,:)];
iq = reshape(iq,size(q));

iq = iq';
end