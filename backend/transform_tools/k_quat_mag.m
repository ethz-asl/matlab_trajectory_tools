function mag = k_quat_mag(q)
% Quaternions are w x y z, active.
% Magnitude of a quaternion.
q_norm = k_quat_norm(q);
mag = 2*acos(q_norm(1));
end

