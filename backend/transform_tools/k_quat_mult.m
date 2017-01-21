function r = k_quat_mult(p,q)
% Quaternions are w x y z, active.
p = p';
q = q';

M = size(p,2);
N = size(q,2);
if M ~= N
  if M==1
    p = repmat(p,1,N);
  else
    q = repmat(q,1,M);
  end
end

r = [ ...
    p(1,:).*q(1,:) - sum(p(2:4,:).*q(2:4,:)); ...
    p([1 1 1],:).*q(2:4,:) + ...
    q([1 1 1],:).*p(2:4,:) + ...
    p([3 4 2],:).*q([4 2 3],:) - ...
    p([4 2 3],:).*q([3 4 2],:) ...
    ];

r = r';
end
