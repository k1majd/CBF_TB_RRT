function sampled_action = ActionSampler(person, goal_map, alpha)
[i, j] = env_map.cord2grid(goal_map.resolution,person.s(1),person.s(2),goal_map.x_limit(1),goal_map.y_limit(1)); %position of person in the map
temp = exp(alpha*(goal_map.map(i,j).Q-goal_map.map(i,j).V));
action_dist=temp/sum(sum(temp));
[row, col] = size(action_dist);
action_cum_vec=cumsum(reshape(action_dist',[row*col, 1])); %transforming the dist to vec form
select_index = find(action_cum_vec>rand,1);
row_index = ceil(select_index/col); %obrain index of selected action in dist
col_index = select_index - (row_index - 1)*col; 
sampled_action = [goal_map.a_v_grid(1,col_index) goal_map.a_theta_grid(row_index,1)];
end