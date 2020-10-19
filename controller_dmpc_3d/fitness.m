function [res] = fitness(s, params)

if params.turn
    res = params.wc * sum_sq_distances(s) + params.ws * separation(s) + vm(s, params); 
%           params.wt * target(s, params)
else
    res = params.wc * sum_sq_distances(s) + params.ws * separation(s) +...
          params.wt * target(s, params) + vm(s, params);
end

end
