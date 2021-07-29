
  clear all
  close all

%% Start ROS connection

rosshutdown
rosinit('http://192.168.0.20:11311')
pause(1);
handles = rossubscriber('/gazebo/model_states', 'BufferSize', 25); % 3:actor1, 4:actor2, 5:actor3, 6:actor4, 7:actor5, 8: hsrb
pause(1);
rob_pos = handles.LatestMessage.Pose(5); %extract robot pose
rob_orient = quat2eul([rob_pos.Orientation.W rob_pos.Orientation.X rob_pos.Orientation.Y rob_pos.Orientation.Z]); %quaternion to euler
%Create publisher node for controller
node1 = ros.Node('/test_node_1');
pause(1);
pub = ros.Publisher(node1,'/chatter','geometry_msgs/Twist');
pause(1);
msg = rosmessage('geometry_msgs/Twist');
%% Initialization

%Controller parameters
x_goal = [0; 5];
init_goal = [0; 0];
x_start=[rob_pos.Position.X;rob_pos.Position.Y;rob_orient(1)]; % Robot start pose
T_s_r=0.3; %Step size
l=0.1; %approximation coeff 
num_obs=2;
v_max=0.35;
w_max=0.3;
velocity_gain = 0.1;
ang_gain = 0.2;
robot=robot(T_s_r,x_start,init_goal,v_max,w_max,l);
rrt_obj=rrt_plan(T_s_r,num_obs,robot,v_max,w_max,l);

%Predictor parameters
T_s_o = 0.6;
K = 50; %monte carlo smaples
pred_samples = 5; %prediction steps
load("map_actor_1.mat")
load("map_actor_2.mat")
maps{1} = map_goal_1;
maps{2} = map_goal_2;
 for k=1:num_obs
     p{k}=[env_map.round_down(10,[handles.LatestMessage.Pose(k+2).Position.X handles.LatestMessage.Pose(k+2).Position.Y])];
 end
 max_p_size = 10;    %maximum size of recorded tracklet
 p_restart_size = 5; %[m]
 

u_traj=[];
x_traj={};
traj_idx=0;
real_curr_act11 = [];
real_curr_act12 = [];
real_curr_act21 = [];
real_curr_act22 = [];
real_curr_act31 = [];
real_curr_act32 = [];
real_curr_act41 = [];
real_curr_act42 = [];
real_curr_act51 = [];
real_curr_act52 = [];
obs_vec_record=[];
obs_traj_record=[];

%% Planning and control 
while (norm(robot.current_state(1:2)-x_goal)>0.6)
    if (norm(robot.current_state(1:2)-init_goal)<2.5)
        robot.x_goal = x_goal;
    end
    
    %Predictor
    for k=1:num_obs
        new_loc = env_map.round_down(10,[handles.LatestMessage.Pose(k+2).Position.X handles.LatestMessage.Pose(k+2).Position.Y]);
        temp_diff = new_loc - p{k}(end,:);
        if (norm(temp_diff) < p_restart_size)
            if length(p{k})>max_p_size
               p{k} = circshift(p{k},-1,1);
               p{k}(end,:) = new_loc; %shift the array and add new_loc to the end
            else
                p{k}=[p{k}; new_loc];
            end
        else
            p{k} = new_loc;
        end
    end
    obs_vec.current_pose = [p{1}(end,:)', p{2}(end,:)'];
    obs_vec.current_r=[0.3; 0.3];
    L = JointStochasticSampling(p,maps,K,pred_samples);
    predict_bounds = extract_bounds(L,pred_samples,T_s_o,T_s_r,obs_vec.current_pose,obs_vec.current_r);     %bounds are extracted based on the robot sample time T_s
    obs_vec.predicted_bounds = predict_bounds;
    obs_vec_prev = obs_vec;
%     if abs(p{2}(end,1)-p{2}(end-1,1))>7
%         obs_traj_record=[obs_traj_record p];
%         p{1}=p{1}(end,:);
%         p{2}=p{2}(end,:);
%         obs_vec = obs_vec_prev;
%     end
    obs_vec_record = [obs_vec_record;obs_vec];
    
    %Controller
    rob_pos = handles.LatestMessage.Pose(5); %extract robot pose
    rob_orient = quat2eul([rob_pos.Orientation.W rob_pos.Orientation.X rob_pos.Orientation.Y rob_pos.Orientation.Z]);
    robot.current_state=[rob_pos.Position.X;rob_pos.Position.Y;rob_orient(1)]; % Robot current pose
    real_curr_act11 = [real_curr_act11 ;obs_vec.current_pose(:,1)'];
    real_curr_act21 = [real_curr_act21; obs_vec.current_pose(:,2)'];
    G = rrt_obj.planning(0,robot,obs_vec, (pred_samples-1)*T_s_o);
    
    % obtain control input
    path_indx = [];
    xr = [];
    ur = [];
    prev_node = G.node_select;
    xr = [flip(G.node(prev_node).prev.xr) xr];
    ur = [flip(G.node(prev_node).prev.ur) ur];
    path_indx = [path_indx prev_node];

    while prev_node~=1
       prev_node = G.node(prev_node).prev.idx;
       path_indx = [path_indx prev_node];
       xr = [flip(G.node(prev_node).prev.xr) xr];
       ur = [flip(G.node(prev_node).prev.ur) ur];
    end
    xr=flip(xr);
    ur=flip(ur);
    path_indx = flip(path_indx);
    [~,col]=size(ur);
    
    
    if isempty(ur)
        ur=[0;0];
    end
    u_traj = [u_traj ur(:,1)]; %save input trajectory
    traj_idx=traj_idx+1;
    x_traj{traj_idx}=xr; %save selected trajectories at each iteration of tree expansion

    real_curr_act12 = [real_curr_act12 ;env_map.round_down(10,[handles.LatestMessage.Pose(3).Position.X handles.LatestMessage.Pose(3).Position.Y])];
    real_curr_act22 = [real_curr_act22 ;env_map.round_down(10,[handles.LatestMessage.Pose(4).Position.X handles.LatestMessage.Pose(4).Position.Y])];
    
    % publish the control command on the control node (node1)
    msg.Linear.X = ur(1,1);
    msg.Angular.Z = ur(2,1);
    send(pub,msg) % Send to node 1
end
% publish the control command on the control node (node1)
    msg.Linear.X = 0;
    msg.Angular.Z = 0;
    send(pub,msg) % Send to node 1
%% plot graph

% Define Static obstacles
stat_obs1=[-7.2 -1.79;7.2 -1.79];
stat_obs2=[-7.2 1.55; -1.68 1.55];
stat_obs3=[1.66 1.55;7.2 1.55];
stat_obs4=[-1.68 1.55;-1.68 6.8];
stat_obs5=[1.66 1.55;1.66 6.8];

figure(2)
axis([-7.1 7.1 -1.8 6.7])
xlabel('$$x~(m)$$','Interpreter','latex','fontsize',20)
ylabel('$y~(m)$','Interpreter','latex','fontsize',20)
hold on
for i=1:length(rrt_obj.graph_h)
    G = rrt_obj.graph_h(i);
    for j=1:length(G.edge)
        plot(G.edge(j).xr(1,:),G.edge(j).xr(2,:),'b')
    end
end
plot(stat_obs1(:,1),stat_obs1(:,2),'k','LineWidth',2.5)
plot(stat_obs2(:,1),stat_obs2(:,2),'k','LineWidth',2.5)
plot(stat_obs3(:,1),stat_obs3(:,2),'k','LineWidth',2.5)
plot(stat_obs4(:,1),stat_obs4(:,2),'k','LineWidth',2.5)
plot(stat_obs5(:,1),stat_obs5(:,2),'k','LineWidth',2.5)
plot(x_goal(1),x_goal(2),'k*','LineWidth',2.5)
plot(x_start(1),x_start(2),'k*','LineWidth',2.5)


figure(3)
axis([-7.1 7.1 -1.8 6.7])
xlabel('$$x~(m)$$','Interpreter','latex','fontsize',20)
ylabel('$y~(m)$','Interpreter','latex','fontsize',20)
hold on
for i=1:length(rrt_obj.graph_h)
    G = rrt_obj.graph_h(i);
    plot(G.node(1).pose(1),G.node(1).pose(2),'bo','LineWidth',2)
    plot(real_curr_act22(i,1),real_curr_act22(i,2),'ro','LineWidth',2)
    plot(real_curr_act12(i,1),real_curr_act12(i,2),'go','LineWidth',2)
end
plot(stat_obs1(:,1),stat_obs1(:,2),'k','LineWidth',2.5)
plot(stat_obs2(:,1),stat_obs2(:,2),'k','LineWidth',2.5)
plot(stat_obs3(:,1),stat_obs3(:,2),'k','LineWidth',2.5)
plot(stat_obs4(:,1),stat_obs4(:,2),'k','LineWidth',2.5)
plot(stat_obs5(:,1),stat_obs5(:,2),'k','LineWidth',2.5)
plot(x_goal(1),x_goal(2),'k*','LineWidth',2.5)
plot(x_start(1),x_start(2),'k*','LineWidth',2.5)



simulate_plot4 = figure(4);
axis([-7.1 7.1 -1.8 6.7])
set(gcf,'Position',[500 1000 900 700])
xlabel('$$x~(m)$$','Interpreter','latex','fontsize',20)
ylabel('$y~(m)$','Interpreter','latex','fontsize',20)
hold on;
plot(stat_obs1(:,1),stat_obs1(:,2),'k','LineWidth',2.5)
plot(stat_obs2(:,1),stat_obs2(:,2),'k','LineWidth',2.5)
plot(stat_obs3(:,1),stat_obs3(:,2),'k','LineWidth',2.5)
plot(stat_obs4(:,1),stat_obs4(:,2),'k','LineWidth',2.5)
plot(stat_obs5(:,1),stat_obs5(:,2),'k','LineWidth',2.5)
plot(x_goal(1),x_goal(2),'k*','LineWidth',2.5)
plot(x_start(1),x_start(2),'k*','LineWidth',2.5)

th = 0:pi/50:2*pi;
for i=1:length(rrt_obj.graph_h)
    G = rrt_obj.graph_h(i);
    %plot robot
    rob = plot(G.node(1).pose(1),G.node(1).pose(2),'o','color','b','LineWidth', 2);
    xunit = rrt_obj.robot_r * cos(th) + G.node(1).pose(1);
    yunit = rrt_obj.robot_r * sin(th) + G.node(1).pose(2);
    rob_r_plot=plot(xunit, yunit,'b','LineWidth',1);
    
    %plot obstacles
    obs1_prev = plot(real_curr_act11(i,1),real_curr_act11(i,2),'go','LineWidth',2);
    obs2_prev = plot(real_curr_act21(i,1),real_curr_act21(i,2),'go','LineWidth',2); %loc before control computation
    obs1_now = plot(real_curr_act12(i,1),real_curr_act12(i,2),'ro','LineWidth',2);
    obs2_now = plot(real_curr_act22(i,1),real_curr_act22(i,2),'ro','LineWidth',2); %loc after control compute (before apply the control)
    
    %plot graph
    
    graph_plot = {};
    graph_plot{1} = plot(G.edge(1).xr(1,:),G.edge(1).xr(2,:));
    if G.edge_indx>1
        for j=2:length(G.edge)
            graph_plot{j} = plot(G.edge(j).xr(1,:),G.edge(j).xr(2,:));
        end
    end
    select_traj = [];
    if length(x_traj{i})>=1
        select_traj = plot(x_traj{i}(1,:),x_traj{i}(2,:),'b','LineWidth',2);
    end
    %plot obstacle predicted bounds
    pred_plot={};
    num_plot_pred = 0;
    %OBS bound current
    num_plot_pred = num_plot_pred +1;
    xunit = obs_vec_record(i).current_r(1) * cos(th) + obs_vec_record(i).current_pose(1,1);
    yunit = obs_vec_record(i).current_r(1) * sin(th) + obs_vec_record(i).current_pose(2,1);
    pred_plot{num_plot_pred}=plot(xunit, yunit,'g','LineWidth',1);
    
    num_plot_pred = num_plot_pred +1;
    xunit = obs_vec_record(i).current_r(2) * cos(th) + obs_vec_record(i).current_pose(1,2);
    yunit = obs_vec_record(i).current_r(2) * sin(th) + obs_vec_record(i).current_pose(2,2);
    pred_plot{num_plot_pred} = plot(xunit, yunit,'g','LineWidth',1);
    
    %OBS bound future
    for k=1:2
        for t=1:pred_samples
            if isempty(obs_vec_record(i).predicted_bounds{k}{t}.r)==0
                num_plot_pred = num_plot_pred +1;
                xunit = obs_vec_record(i).predicted_bounds{k}{t}.r * cos(th) + obs_vec_record(i).predicted_bounds{k}{t}.center(1);
                yunit = obs_vec_record(i).predicted_bounds{k}{t}.r * sin(th) + obs_vec_record(i).predicted_bounds{k}{t}.center(2);
                pred_plot{num_plot_pred} = plot(xunit, yunit,'g','LineWidth',1); 
            end
        end
    end

    drawnow
    pause(0.1);
    set(rob,'Visible','off')
    set(obs1_prev,'Visible','off')
    set(obs2_prev,'Visible','off')
    set(obs1_now,'Visible','off')
    set(obs2_now,'Visible','off')
    set(rob_r_plot,'Visible','off')
    if ~isempty(select_traj)
       set(select_traj,'Visible','off')
    end
    
    for io = 1:length(graph_plot)
          set(graph_plot{io},'Visible','off')
    end
    for iq = 1:num_plot_pred
          set(pred_plot{iq},'Visible','off')
    end
end

close(simulate_plot4)

figure(5)
axis([init_goal(1)-2 init_goal(1)+2 init_goal(2)-2 init_goal(2)+2])
set(gcf,'Position',[500 1000 900 700])
xlabel('$$x~(m)$$','Interpreter','latex','fontsize',20)
ylabel('$y~(m)$','Interpreter','latex','fontsize',20)
hold on;
plot(stat_obs1(:,1),stat_obs1(:,2),'k','LineWidth',2.5)
plot(stat_obs2(:,1),stat_obs2(:,2),'k','LineWidth',2.5)
plot(stat_obs3(:,1),stat_obs3(:,2),'k','LineWidth',2.5)
plot(stat_obs4(:,1),stat_obs4(:,2),'k','LineWidth',2.5)
plot(stat_obs5(:,1),stat_obs5(:,2),'k','LineWidth',2.5)
plot(x_goal(1),x_goal(2),'k*','LineWidth',2.5)
plot(x_start(1),x_start(2),'k*','LineWidth',2.5)

th = 0:pi/50:2*pi;
for i=1:length(rrt_obj.graph_h)
    G = rrt_obj.graph_h(i);
    axis([G.node(1).pose(1)-2 G.node(1).pose(1)+2 G.node(1).pose(2)-2 G.node(1).pose(2)+2])
    %plot robot
    rob = plot(G.node(1).pose(1),G.node(1).pose(2),'o','color','b','LineWidth', 2);
    xunit = rrt_obj.robot_r * cos(th) + G.node(1).pose(1);
    yunit = rrt_obj.robot_r * sin(th) + G.node(1).pose(2);
    rob_r_plot=plot(xunit, yunit,'b','LineWidth',1);
    
    %plot obstacles
    obs1_prev = plot(real_curr_act11(i,1),real_curr_act11(i,2),'go','LineWidth',2);
    obs2_prev = plot(real_curr_act21(i,1),real_curr_act21(i,2),'go','LineWidth',2); %loc before control computation
    obs1_now = plot(real_curr_act12(i,1),real_curr_act12(i,2),'ro','LineWidth',2);
    obs2_now = plot(real_curr_act22(i,1),real_curr_act22(i,2),'ro','LineWidth',2); %loc after control compute (before apply the control)
    
    %plot graph
    graph_plot = {};
    graph_plot{1} = plot(G.edge(1).xr(1,:),G.edge(1).xr(2,:));
    if G.edge_indx>1
        for j=2:length(G.edge)
            graph_plot{j} = plot(G.edge(j).xr(1,:),G.edge(j).xr(2,:));
        end
    end
    select_traj = [];
    if length(x_traj{i})>=1
        select_traj = plot(x_traj{i}(1,:),x_traj{i}(2,:),'b','LineWidth',2);
    end
    %plot obstacle predicted bounds
    pred_plot={};
    num_plot_pred = 0;
    %OBS bound current
    num_plot_pred = num_plot_pred +1;
    xunit = obs_vec_record(i).current_r(1) * cos(th) + obs_vec_record(i).current_pose(1,1);
    yunit = obs_vec_record(i).current_r(1) * sin(th) + obs_vec_record(i).current_pose(2,1);
    pred_plot{num_plot_pred}=plot(xunit, yunit,'g','LineWidth',1);
    
    num_plot_pred = num_plot_pred +1;
    xunit = obs_vec_record(i).current_r(2) * cos(th) + obs_vec_record(i).current_pose(1,2);
    yunit = obs_vec_record(i).current_r(2) * sin(th) + obs_vec_record(i).current_pose(2,2);
    pred_plot{num_plot_pred} = plot(xunit, yunit,'g','LineWidth',1);
    
    %OBS bound future
    for k=1:2
        for t=1:pred_samples
            if isempty(obs_vec_record(i).predicted_bounds{k}{t}.r)==0
                num_plot_pred = num_plot_pred +1;
                xunit = obs_vec_record(i).predicted_bounds{k}{t}.r * cos(th) + obs_vec_record(i).predicted_bounds{k}{t}.center(1);
                yunit = obs_vec_record(i).predicted_bounds{k}{t}.r * sin(th) + obs_vec_record(i).predicted_bounds{k}{t}.center(2);
                pred_plot{num_plot_pred} = plot(xunit, yunit,'g','LineWidth',1); 
            end
        end
    end

    drawnow
    pause(0.1);
    set(rob,'Visible','off')
    set(obs1_prev,'Visible','off')
    set(obs2_prev,'Visible','off')
    set(obs1_now,'Visible','off')
    set(obs2_now,'Visible','off')
    set(rob_r_plot,'Visible','off')
    if ~isempty(select_traj)
       set(select_traj,'Visible','off')
    end
    for io = 1:length(graph_plot)
          set(graph_plot{io},'Visible','off')
    end
    for iq = 1:num_plot_pred
          set(pred_plot{iq},'Visible','off')
    end
end

figure(8)
h_vex=[];
for k=1:length(rrt_obj.graph_h)
    node_sel = rrt_obj.graph_h(k).node_select;
    h_vex=[h_vex rrt_obj.graph_h(k).node(node_sel).prev.hr];
end

