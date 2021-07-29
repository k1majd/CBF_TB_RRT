function goal_map_index = GoalSampler(tracklet,maps,beta)
[init_x, init_y] = env_map.cord2grid(maps{1}.resolution,tracklet(1,1),tracklet(1,2),maps{1}.x_limit(1),maps{1}.y_limit(1));
[final_x, final_y]= env_map.cord2grid(maps{1}.resolution,tracklet(end,1),tracklet(end,2),maps{1}.x_limit(1),maps{1}.y_limit(1));
V_gradient=[];

%calculate gradients of V for each map
for i=1:length(maps) 
    V_gradient=[V_gradient (maps{i}.map(final_x,final_y).V)-(maps{i}.map(init_x,init_y).V)];
end
g_dist = exp(beta*V_gradient)/sum(exp(beta*V_gradient)); %goals distribution
g_cumdist = cumsum(g_dist); %cumulative distribution
goal_map_index=find(g_cumdist>rand,1); %sample over cumulative distribution
end