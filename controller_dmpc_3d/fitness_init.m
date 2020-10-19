function [res] = fitness_init(s, params)

if params.turn
    res = params.wc * sum_sq_distances(s) + params.ws * separation(s) +...
          params.wt * target_init(s, params) + vm(s, params);
else
    res = params.wc * sum_sq_distances(s) + params.ws * separation(s) +...
          params.wt * target_init(s, params) + vm(s, params);
end

end

