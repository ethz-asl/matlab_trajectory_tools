function ia = k_tf_inv(a)
% Transformations are [x y z q_w q_x q_y q_z].
ia = a; %zeros(size(a));

ia(:, 4) = a(:, 4);
ia(:, 5:7) = -a(:, 5:7);
ia(:, 1:3) = -k_quat_rotate(ia(:, 4:7), a(:, 1:3));
end
