load("map_actor_1.mat")
load("map_actor_2.mat")
maps{1} = map_goal_1;
maps{2} = map_goal_2;

p{1}=[0 5;0.5 4.5;1 4;1.5 3.5;2 3];
K=100;
pred_samples = 5;
T_s_o=0.3;
T_s_r=0.3;
obs_vec.current_pose = p{1}(end,:)';
obs_vec.current_r=0.3;
L = JointStochasticSampling(p,maps,K,pred_samples);
[cord_y, cord_x] = meshgrid(-1.7:0.1:6.7, -7.1:0.1:7.1); 
figure(2)
hold on
for t=1:pred_samples
    surf(cord_x,cord_y,L{1}{t})
end

predict_bounds = extract_bounds(L,pred_samples,T_s_o,T_s_r,obs_vec.current_pose,obs_vec.current_r);

