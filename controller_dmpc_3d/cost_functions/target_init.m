function [res] = target_init(s, params)

vel = squeeze(s(1,4:6,:));
% res =  norm(pos(:,1) - target);
n = size(s, 3);

res = 0;
for i = 1:n
    res = res + norm(vel(:,i) - params.target);
end

end
