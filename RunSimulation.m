function [traj,psi,weight] = RunSimulation(s_init, params, simNum, opt)

s = zeros(params.steps, 12, params.n);
accs = zeros(params.steps, 3, params.n);
s(1,:,:) = s_init;

w_turn(1:params.num_leaders) = 0;
alpha = 0.9;
count = 0;
% if params.turn_angle <= 120
%     slope = 0.3;
%     mu = 170;
% elseif params.turn_angle > 120 && params.turn_angle <= 140
%     slope = 0.4;
%     mu = 160;
% else
%     slope = 0.5;
%     mu = 150;
% end

if params.turn_angle < 0
    theta_turn = -params.turn_angle;
else
    theta_turn = params.turn_angle;
end

control_jump = uint32(params.ct / params.dt);
control_steps = ceil(params.steps / control_jump);

a = [zeros(1,params.n); zeros(1,params.n); ones(1,params.n)];
a_past = zeros(3,20);
a_past2 = zeros(3,20);

for i = 1:params.n
    e(i).prev_e_R = 0;
    e(i).sum_e_R = 0;
    e(i).prev_e_w = 0;
    e(i).sum_e_w = 0;
end

fval = zeros(1,params.steps);
exit_flag = zeros(1,params.steps);

u_dash_prev = zeros(4,params.n);
rot_speed = zeros(params.steps, 4, params.n);
u_quad = zeros(params.steps, 4, params.n);

% modelfile = 'Neural Controller Models/model_7neighbor.h5';
% net = importKerasNetwork(modelfile);

tic;
for k = 0:control_steps - 1
    if params.turn
        cur_step = 1+k*control_jump;
        past_vel = squeeze(s(max(cur_step-1,1),4:6,:));
        cur_vel = squeeze(s(cur_step,4:6,:));
        delta_vel = sqrt(sum((cur_vel - past_vel).^2, 1)); %For turn logic
        if k == floor(params.start_turn/control_jump)
            signal = 1;
            [edge_agents, acc_turn, axis_rot] = edge_cluster(s(1 + k*control_jump,:,:), params, signal);
        end
%         if k == floor((params.start_turn + params.t_fix + params.t_cont)/control_jump)
%             signal = 2;
%             [edge_agents, acc_turn, axis_rot] = edge_cluster(s(1 + k*control_jump,:,:), params, signal);
%         end
    end
    
    %Run MPC controller take first action
    if params.turn
        [a, fit_val, e_flag] = controller_dmpc_3d_turn(s(1 + k*control_jump,:,:), delta_vel, params, opt);
%         if k >= floor(params.start_turn/control_jump) && k <= floor((params.start_turn+params.t_fix)/control_jump) %dot(P1,P2)/(norm(P1)*norm(P2))
        if k >= floor(params.start_turn/control_jump)
            vel_init = squeeze(s(params.start_turn,4:6,:));
            avg_vel_init = mean(vel_init,2).';
            pos = squeeze(s(2*k,4:6,:));
            sq_d_p = sq_distances_pairwise(pos);
            for i = 1:numel(edge_agents)
                [~, initIndex] = sort(sq_d_p(edge_agents(i),:));
                init_sub = initIndex(1:params.knn+1);
                for j = 1:numel(init_sub)
                    vel(:,j) = squeeze(s(2*k,4:6,init_sub(j)));
                end
                avg_vel(i,:) = mean(vel,2).';
                theta(i) = atan2d(norm(cross(avg_vel_init(1,:),avg_vel(i,:))),dot(avg_vel_init(1,:),avg_vel(i,:)));
                
            end
        else
            theta(1:params.num_leaders) = 1000;
        end
        psi(k+1,:) = theta;
        
%%      Logistic function based turning
        if k >= floor(params.start_turn/control_jump) && mean(theta) <= alpha*theta_turn
%             for i = 1:params.num_leaders
%                 w_turn(i) = 1/(1 + exp(1 * ((100*(theta(i)/theta_turn)) - 75)));
%             end
%             if w_turn < 0.0001
%                 w_turn = 0;
%             end
            for idx = 1:numel(edge_agents)
                w_turn(idx) = 1/(1 + exp(1 * ((100*(theta(idx)/theta_turn)) - 75)));
                a(:,edge_agents(idx)) = trim_vec(w_turn(idx)*(a_past(:,edge_agents(idx)) + params.dt * params.jerk * acc_turn) + (1 - w_turn(idx))*a(:,edge_agents(idx)), params.amax);
                a_past(:,edge_agents(idx)) = a(:,edge_agents(idx));
            end
%             count = count + 2;
        end
        weight(k+1,:) = w_turn;
        
%%        Angle based turning
%         if k >= floor(params.start_turn/control_jump) && theta <= beta*theta_turn
% %             [edge_agents, acc_turn, axis_rot] = edge_cluster(s(1 + k*control_jump,:,:), params);
% %             if k > params.start_turn + 100
%             if theta >= alpha*theta_turn
%                 w = min(50,2 * count * 0.8);
%                 w_turn = max(0,w_turn - (1/w));
% %                 w_turn = 1/(1 + (0.001*exp(10*(theta/theta_turn))));
%                 for idx = 1:numel(edge_agents)
%                     a(:,edge_agents(idx)) = trim_vec(w_turn*(a_past(:,edge_agents(idx)) + params.dt * params.jerk * acc_turn) + (1 - w_turn)*a(:,edge_agents(idx)), params.amax);
%                     a_past(:,edge_agents(idx)) = a(:,edge_agents(idx));
%                 end
%             else 
%                 for idx = 1:numel(edge_agents)
%                     a(:,edge_agents(idx)) = trim_vec(a_past(:,edge_agents(idx)) + params.dt * params.jerk * acc_turn, params.amax);
%                     a_past(:,edge_agents(idx)) = a(:,edge_agents(idx));
%                 end
%                 count = count + 1;
%             end
%         end
%         weight(k+1,1) = w_turn;
        
%%        Fixed time turning        
%         if k >= floor((params.start_turn + params.t_fix + params.t_cont)/control_jump) && k <= floor(((params.start_turn + params.t_fix + params.t_cont)+params.t_fix)/control_jump) %dot(P1,P2)/(norm(P1)*norm(P2))
% %             [edge_agents, acc_turn, axis_rot] = edge_cluster(s(1 + k*control_jump,:,:), params);
%             if k > params.start_turn + params.t_fix + params.t_cont + 100
%                 for idx = 1:numel(edge_agents)
%                     a(:,edge_agents(idx)) = trim_vec(w_turn*(a_past2(:,edge_agents(idx)) + params.dt * params.jerk * acc_turn) + (1 - w_turn)*a(:,edge_agents(idx)), params.amax);
%                     a_past2(:,edge_agents(idx)) = a(:,edge_agents(idx));
%                 end
%                 w_turn = w_turn - (1/50);
%             else 
%                 for idx = 1:numel(edge_agents)
%                     a(:,edge_agents(idx)) = trim_vec(a_past2(:,edge_agents(idx)) + params.dt * params.jerk * acc_turn, params.amax);
%                     a_past2(:,edge_agents(idx)) = a(:,edge_agents(idx));
%                 end
%             end
%             
%         end
    else
        %  [a, fit_val, e_flag, history] = controller_cmpc_3d(s(1 + k*control_jump,:,:), params, a, opt);
        [a, fit_val, e_flag] = controller_dmpc_3d(s(1 + k*control_jump,:,:), params, opt);
    end
    
%     plot(history.fval, 'LineWidth', 1.5, 'Marker', '*', 'Color', 'r', 'MarkerEdgeColor', 'b');
%     pause(0.1);
%     [a] = controller_dnn_3d(s(1 + k*control_jump,:,:), params, net);
    fval(1+k*control_jump: (k+1)*control_jump) = fit_val;
    exit_flag(1+k*control_jump: (k+1)*control_jump) = e_flag;
    
    for c = 0:control_jump-1
        accs(1+k*control_jump+c, :,:) = reshape(a, [1,3,params.n]);
        %  fval(1+k*control_jump+c) = fit_val;
    end
    
    %% Apply first action to the quadcopters.
    %  [s(1 + k*control_jump:1 + (k+1)*control_jump,:,:)] = dynamics(s(1 + k*control_jump, :, :), a, params);
    num_steps = uint32(params.ct / params.dt);
    s_new = zeros(num_steps+1, 12, params.n);
    s_new(1,:,:) = s(1 + k*control_jump, :, :);
    
    if params.quad % Plant is a quadrotor
        for i = 1:params.n
            for m = 1:num_steps
                %acceleration to orientation and thrust
                u_dash = acc2thrust(a(:,i), params, u_dash_prev(2:4,i) );
                u_dash_prev(:,i) = u_dash;
                %tracking inner loop control
                [u_quad(k*control_jump+m, :, i), e(i)] = controller_inner_pid(s_new(m,:,i), u_dash, e(i));
                
                [s_new(m+1,:,i), rot_speed(k*control_jump+m,:,i)] = dynamics_quadcopter(s_new(m,:,i), u_quad(k*control_jump+m, :, i)', params);
            end
        end
    else % Plant is point-like
        for m2 = 1:num_steps
            [s_new(m2+1,:,:)] = dynamics_point(s_new(m2,:,:), a ,params);
        end
    end
    s(1 + k*control_jump:1 + (k+1)*control_jump,:,:) = s_new;
    
    %% print progress
    if (mod(k,4) == 0)
        elapsed_time = round(toc, 1);
        disp(['sim: ' num2str(simNum) ', step: ' num2str(control_jump * k) ', Time: ' num2str(elapsed_time) ' s']);
    end
end
%% Store result
if params.quad == 1
    x = squeeze(s(:,1,:));
    y = squeeze(s(:,2,:));
    z = squeeze(s(:,3,:));
    vx = squeeze(s(:,4,:));
    vy = squeeze(s(:,5,:));
    vz = squeeze(s(:,6,:));
    
    phi = squeeze(s(:,7,:));
    theta = squeeze(s(:,8,:));
    psi = squeeze(s(:,9,:));
    phi_dot = squeeze(s(:,10,:));
    theta_dot = squeeze(s(:,11,:));
    psi_dot = squeeze(s(:,12,:));
    
    accx = squeeze(accs(:,1,:));
    accy = squeeze(accs(:,2,:));
    accz = squeeze(accs(:,3,:));
    
    traj.x = x(1:params.steps,:);
    traj.y = y(1:params.steps,:);
    traj.z = z(1:params.steps,:);
    traj.vx = vx(1:params.steps,:);
    traj.vy = vy(1:params.steps,:);
    traj.vz = vz(1:params.steps,:);
    traj.phi = phi(1:params.steps,:);
    traj.theta = theta(1:params.steps,:);
    traj.psi = psi(1:params.steps,:);
    traj.phi_dot = phi_dot(1:params.steps,:);
    traj.theta_dot = theta_dot(1:params.steps,:);
    traj.psi_dot = psi_dot(1:params.steps,:);
    traj.accx = accx(1:params.steps,:);
    traj.accy = accy(1:params.steps,:);
    traj.accz = accz(1:params.steps,:);
    traj.params = params;
    traj.mpc_cost= fval;
    traj.exit_flag = exit_flag;
    
    traj.u = u_quad;
    traj.s = s;
    traj.accs = accs;
    traj.omegas = rot_speed;
    if params.turn
        traj.axis_rot = axis_rot;
        traj.edge_agents = edge_agents;
    end
elseif params.quad == 0
    x = squeeze(s(:,1,:));
    y = squeeze(s(:,2,:));
    z = squeeze(s(:,3,:));
    vx = squeeze(s(:,4,:));
    vy = squeeze(s(:,5,:));
    vz = squeeze(s(:,6,:));
    
    accx = squeeze(accs(:,1,:));
    accy = squeeze(accs(:,2,:));
    accz = squeeze(accs(:,3,:));
    
    traj.x = x(1:params.steps,:);
    traj.y = y(1:params.steps,:);
    traj.z = z(1:params.steps,:);
    traj.vx = vx(1:params.steps,:);
    traj.vy = vy(1:params.steps,:);
    traj.vz = vz(1:params.steps,:);
    
    traj.accx = accx(1:params.steps,:);
    traj.accy = accy(1:params.steps,:);
    traj.accz = accz(1:params.steps,:);
    traj.params = params;
    traj.mpc_cost= fval;
    traj.exit_flag = exit_flag;
    
    traj.s = s;
    traj.accs = accs;
    
    if params.turn
        traj.axis_rot = axis_rot;
        traj.edge_agents = edge_agents;
    end
end
end


