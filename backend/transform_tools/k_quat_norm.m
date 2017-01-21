function q = k_quat_norm(q)
% qn = renorm(q)
% 		This renormalizes each quaternion such that it has unit magnitude;

q = q';

imag = 1./sqrt(sum(q.^2));

for i=1:size(q,1)
  q(i,:) = q(i,:).*imag(:,:);
end

q = q';
end


