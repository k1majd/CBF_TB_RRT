clc
clear all 
close all


%environment static map
env_dim_x = [-7.1 7.1]; %[x y] 10 m by 10 m >=0
env_dim_y = [-1.7 6.7]; %[x y] 10 m by 10 m >=0
resolution = 10; %pixel per meter
goal{2} = [7 -1];

map_goal_1=env_map(env_dim_x,env_dim_y,resolution,goal{2});
% save("map_actor_1.mat","map_goal_1")

goal{3} = [-7 1];

map_goal_2=env_map(env_dim_x,env_dim_y,resolution,goal{3});
%save("map_actor_2.mat","map_goal_2")



%Plotting code

% [i_max,j_max]=env_map.cord2grid(10,7.1,6.7,-7.1,-1.7);
%  
%  for i=1:i_max
%      for j=1:j_max
%          [~, col1] = size(map_goal_2.map(i,j).policy);
%          sel1=randi(col1);
%          pol=[map_goal_2.map(i,j).policy(1,sel1); map_goal_2.map(i,j).policy(2,sel1)];
%          if map_goal_2.map(i,j).C==1;pol=[0;0];end
%          map1_x_vec(i, j) = pol(1)*cos(pol(2));
%          map1_y_vec(i, j) = pol(1)*sin(pol(2));
%      end    
%  end
%  
%  [yy, xx]=meshgrid(-1.7:0.1:6.7,-7.1:0.1:7.1);
%  
%  figure(5)
%  quiver(xx,yy,map1_x_vec,map1_y_vec,1)
