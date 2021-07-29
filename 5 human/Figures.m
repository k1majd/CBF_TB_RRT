%% plot graph
%close all
%clear all
% Define Static obstacles
stat_obs1=[-7.2 -1.79;7.2 -1.79];
stat_obs2=[-7.2 1.55; -1.68 1.55];
stat_obs3=[1.66 1.55;7.2 1.55];
stat_obs4=[-1.68 1.55;-1.68 6.8];
stat_obs5=[1.66 1.55;1.66 6.8];

%load('controller_data.mat');
%load('matlab.mat');
%load('5_human_data.mat');


% figure(2)
% axis([-7.1 7.1 -1.8 6.7])
% xlabel('$$x~(m)$$','Interpreter','latex','fontsize',20)
% ylabel('$y~(m)$','Interpreter','latex','fontsize',20)
% hold on
% for i=1:length(rrt_obj.graph_h)
%     G = rrt_obj.graph_h(i);
%     for j=1:length(G.edge)
%         plot(G.edge(j).xr(1,:),G.edge(j).xr(2,:),'b')
%     end
% end
% plot(stat_obs1(:,1),stat_obs1(:,2),'k','LineWidth',2.5)
% plot(stat_obs2(:,1),stat_obs2(:,2),'k','LineWidth',2.5)
% plot(stat_obs3(:,1),stat_obs3(:,2),'k','LineWidth',2.5)
% plot(stat_obs4(:,1),stat_obs4(:,2),'k','LineWidth',2.5)
% plot(stat_obs5(:,1),stat_obs5(:,2),'k','LineWidth',2.5)
% plot(x_goal(1),x_goal(2),'k*','LineWidth',2.5)
% plot(x_start(1),x_start(2),'k*','LineWidth',2.5)
% 
% 
% figure(3)
% axis([-7.1 7.1 -1.8 6.7])
% xlabel('$$x~(m)$$','Interpreter','latex','fontsize',20)
% ylabel('$y~(m)$','Interpreter','latex','fontsize',20)
% hold on
% for i=1:length(rrt_obj.graph_h)
%     G = rrt_obj.graph_h(i);
%     plot(G.node(1).pose(1),G.node(1).pose(2),'bo','LineWidth',2)
%     plot(real_curr_act22(i,1),real_curr_act22(i,2),'ro','LineWidth',2)
%     plot(real_curr_act12(i,1),real_curr_act12(i,2),'go','LineWidth',2)
%     plot(real_curr_act32(i,1),real_curr_act32(i,2),'ko','LineWidth',2)
%     plot(real_curr_act42(i,1),real_curr_act42(i,2),'co','LineWidth',2)
%     plot(real_curr_act52(i,1),real_curr_act52(i,2),'yo','LineWidth',2)
% end
% plot(stat_obs1(:,1),stat_obs1(:,2),'k','LineWidth',2.5)
% plot(stat_obs2(:,1),stat_obs2(:,2),'k','LineWidth',2.5)
% plot(stat_obs3(:,1),stat_obs3(:,2),'k','LineWidth',2.5)
% plot(stat_obs4(:,1),stat_obs4(:,2),'k','LineWidth',2.5)
% plot(stat_obs5(:,1),stat_obs5(:,2),'k','LineWidth',2.5)
% plot(x_goal(1),x_goal(2),'k*','LineWidth',2.5)
% plot(x_start(1),x_start(2),'k*','LineWidth',2.5)



simulate_plot4 = figure(4);
axis([-7.1 7.1 -1.8 6.7])

set(gcf,'Position',[500 500 900 700])
set(gca,'FontSize',20)
xlabel('x [m]','fontsize',30)
ylabel('y [m]','fontsize',30)
hold on;
rectangle('Position',[-7.2 1.55 5.52 5.52],'FaceColor','k')
rectangle('Position',[1.66 1.55 5.54 5.25],'FaceColor','k')
plot(stat_obs1(:,1),stat_obs1(:,2),'k','LineWidth',2.5)
plot(stat_obs2(:,1),stat_obs2(:,2),'k','LineWidth',2.5)
plot(stat_obs3(:,1),stat_obs3(:,2),'k','LineWidth',2.5)
plot(stat_obs4(:,1),stat_obs4(:,2),'k','LineWidth',2.5)
plot(stat_obs5(:,1),stat_obs5(:,2),'k','LineWidth',2.5)
plot(x_goal(1),x_goal(2),'k*','LineWidth',4)
plot(x_start(1),x_start(2),'k*','LineWidth',2.5)

lightBlue = [153,204,255] / 255;
darkGreen = [0,153,0] / 255;
%title('Time: 8 [s]')

th = 0:pi/50:2*pi;
fig4_steps = length(rrt_obj.graph_h);
for i=1:fig4_steps
    G = rrt_obj.graph_h(i);
    axis([-7.1 7.1 -1.8 6.7])
    %plot robot
    rob = plot(G.node(1).pose(1),G.node(1).pose(2),'o','color','b','LineWidth', 2);
    xunit = rrt_obj.robot_r * cos(th) + G.node(1).pose(1);
    yunit = rrt_obj.robot_r * sin(th) + G.node(1).pose(2);
    rob_r_plot=plot(xunit, yunit,'b','LineWidth',2);
    
    %plot obstacles
    obs1_prev = plot(real_curr_act11(i,1),real_curr_act11(i,2),'Color', darkGreen,'Marker','o','LineWidth',2);
    obs2_prev = plot(real_curr_act21(i,1),real_curr_act21(i,2),'Color', darkGreen,'Marker','o','LineWidth',2); %loc before control computation
    obs3_prev = plot(real_curr_act31(i,1),real_curr_act31(i,2),'Color', darkGreen,'Marker','o','LineWidth',2);
    obs4_prev = plot(real_curr_act41(i,1),real_curr_act41(i,2),'Color', darkGreen,'Marker','o','LineWidth',2);
    obs5_prev = plot(real_curr_act51(i,1),real_curr_act51(i,2),'Color', darkGreen,'Marker','o','LineWidth',2);
    %obs1_now = plot(real_curr_act12(i,1),real_curr_act12(i,2),'ro','LineWidth',2);
    %obs2_now = plot(real_curr_act22(i,1),real_curr_act22(i,2),'ro','LineWidth',2); %loc after control compute (before apply the control)
    %obs3_now = plot(real_curr_act32(i,1),real_curr_act32(i,2),'ro','LineWidth',2);
    %obs4_now = plot(real_curr_act42(i,1),real_curr_act42(i,2),'ro','LineWidth',2);
    %obs5_now = plot(real_curr_act52(i,1),real_curr_act52(i,2),'ro','LineWidth',2);
    
    %plot graph
    graph_plot = {};
    graph_plot{1} = plot(G.edge(1).xr(1,:),G.edge(1).xr(2,:),'Color',lightBlue,'LineWidth',1.5);
    if i~=fig4_steps
        if G.edge_indx>1
            for j=2:length(G.edge)
                graph_plot{j} = plot(G.edge(j).xr(1,:),G.edge(j).xr(2,:),'Color',lightBlue,'LineWidth',1.5);
            end
        end
    else
        if G.edge_indx>1
            for j=2:length(G.edge)
                graph_plot{j} = plot(G.edge(j).xr(1,:),G.edge(j).xr(2,:),'b','LineStyle','--','LineWidth',2);
            end
        end
    end
    select_traj = [];
    if length(x_traj{i})>=1
        select_traj = plot(x_traj{i}(1,:),x_traj{i}(2,:),'b','LineWidth',3);
    end
    %plot obstacle predicted bounds
    pred_plot={};
    num_plot_pred = 0;
    %OBS bound current
    num_plot_pred = num_plot_pred +1;
    xunit = obs_vec_record(i).current_r(1) * cos(th) + obs_vec_record(i).current_pose(1,1);
    yunit = obs_vec_record(i).current_r(1) * sin(th) + obs_vec_record(i).current_pose(2,1);
    pred_plot{num_plot_pred}=plot(xunit, yunit,'Color', darkGreen,'LineWidth',2);
    
    num_plot_pred = num_plot_pred +1;
    xunit = obs_vec_record(i).current_r(2) * cos(th) + obs_vec_record(i).current_pose(1,2);
    yunit = obs_vec_record(i).current_r(2) * sin(th) + obs_vec_record(i).current_pose(2,2);
    pred_plot{num_plot_pred} = plot(xunit, yunit,'Color', darkGreen,'LineWidth',2);
    
    num_plot_pred = num_plot_pred +1;
    xunit = obs_vec_record(i).current_r(3) * cos(th) + obs_vec_record(i).current_pose(1,3);
    yunit = obs_vec_record(i).current_r(3) * sin(th) + obs_vec_record(i).current_pose(2,3);
    pred_plot{num_plot_pred} = plot(xunit, yunit,'Color', darkGreen,'LineWidth',2);
    
    num_plot_pred = num_plot_pred +1;
    xunit = obs_vec_record(i).current_r(4) * cos(th) + obs_vec_record(i).current_pose(1,4);
    yunit = obs_vec_record(i).current_r(4) * sin(th) + obs_vec_record(i).current_pose(2,4);
    pred_plot{num_plot_pred} = plot(xunit, yunit,'Color', darkGreen,'LineWidth',2);
    
    num_plot_pred = num_plot_pred +1;
    xunit = obs_vec_record(i).current_r(5) * cos(th) + obs_vec_record(i).current_pose(1,5);
    yunit = obs_vec_record(i).current_r(5) * sin(th) + obs_vec_record(i).current_pose(2,5);
    pred_plot{num_plot_pred} = plot(xunit, yunit,'Color', darkGreen,'LineWidth',2);
    
    %OBS bound future
    for k=1:5
        for t=1:pred_samples
            if isempty(obs_vec_record(i).predicted_bounds{k}{t}.r)==0
                num_plot_pred = num_plot_pred +1;
                xunit = obs_vec_record(i).predicted_bounds{k}{t}.r * cos(th) + obs_vec_record(i).predicted_bounds{k}{t}.center(1);
                yunit = obs_vec_record(i).predicted_bounds{k}{t}.r * sin(th) + obs_vec_record(i).predicted_bounds{k}{t}.center(2);
                pred_plot{num_plot_pred} = plot(xunit, yunit,'Color', darkGreen,'LineWidth',2); 
            end
        end
    end

    drawnow
    pause(0.1);
    if i~=fig4_steps
        %set(rob,'Visible','off')
        set(obs1_prev,'Visible','off')
        set(obs2_prev,'Visible','off')
        set(obs3_prev,'Visible','off')
        set(obs4_prev,'Visible','off')
        set(obs5_prev,'Visible','off')
        %set(obs1_now,'Visible','off')
        %set(obs2_now,'Visible','off')
        %set(obs3_now,'Visible','off')
        %set(obs4_now,'Visible','off')
        %set(obs5_now,'Visible','off')
        set(rob_r_plot,'Visible','off')
        if ~isempty(select_traj)
           set(select_traj,'Visible','off')
        end

        for io = 1:length(graph_plot)
              %set(graph_plot{io},'Visible','off')
        end
        for iq = 1:num_plot_pred
              set(pred_plot{iq},'Visible','off')
        end
    end
end

%close(simulate_plot4)

%
% th = 0:pi/50:2*pi;
% for i=1:length(rrt_obj.graph_h)
%     G = rrt_obj.graph_h(i);
%     axis([G.node(1).pose(1)-2 G.node(1).pose(1)+2 G.node(1).pose(2)-2 G.node(1).pose(2)+2])
%     %plot robot
%     rob = plot(G.node(1).pose(1),G.node(1).pose(2),'o','color','b','LineWidth', 2);
%     xunit = rrt_obj.robot_r * cos(th) + G.node(1).pose(1);
%     yunit = rrt_obj.robot_r * sin(th) + G.node(1).pose(2);
%     rob_r_plot=plot(xunit, yunit,'b','LineWidth',1);
%     
%     %plot obstacles
%     obs1_prev = plot(real_curr_act11(i,1),real_curr_act11(i,2),'go','LineWidth',2);
%     obs2_prev = plot(real_curr_act21(i,1),real_curr_act21(i,2),'go','LineWidth',2); %loc before control computation
%     obs3_prev = plot(real_curr_act31(i,1),real_curr_act31(i,2),'go','LineWidth',2);
%     obs4_prev = plot(real_curr_act41(i,1),real_curr_act41(i,2),'go','LineWidth',2);
%     obs5_prev = plot(real_curr_act51(i,1),real_curr_act51(i,2),'go','LineWidth',2);
%     obs1_now = plot(real_curr_act12(i,1),real_curr_act12(i,2),'ro','LineWidth',2);
%     obs2_now = plot(real_curr_act22(i,1),real_curr_act22(i,2),'ro','LineWidth',2); %loc after control compute (before apply the control)
%     obs3_now = plot(real_curr_act32(i,1),real_curr_act32(i,2),'ro','LineWidth',2);
%     obs4_now = plot(real_curr_act42(i,1),real_curr_act42(i,2),'ro','LineWidth',2);
%     obs5_now = plot(real_curr_act52(i,1),real_curr_act52(i,2),'ro','LineWidth',2);
%     
%     %plot graph
%     graph_plot = {};
%     graph_plot{1} = plot(G.edge(1).xr(1,:),G.edge(1).xr(2,:));
%     if G.edge_indx>1
%         for j=2:length(G.edge)
%             graph_plot{j} = plot(G.edge(j).xr(1,:),G.edge(j).xr(2,:));
%         end
%     end
%     select_traj = [];
%     if length(x_traj{i})>=1
%         select_traj = plot(x_traj{i}(1,:),x_traj{i}(2,:),'b','LineWidth',2);
%     end
%     %plot obstacle predicted bounds
%     pred_plot={};
%     num_plot_pred = 0;
%     %OBS bound current
%     num_plot_pred = num_plot_pred +1;
%     xunit = obs_vec_record(i).current_r(1) * cos(th) + obs_vec_record(i).current_pose(1,1);
%     yunit = obs_vec_record(i).current_r(1) * sin(th) + obs_vec_record(i).current_pose(2,1);
%     pred_plot{num_plot_pred}=plot(xunit, yunit,'g','LineWidth',1);
%     
%     num_plot_pred = num_plot_pred +1;
%     xunit = obs_vec_record(i).current_r(2) * cos(th) + obs_vec_record(i).current_pose(1,2);
%     yunit = obs_vec_record(i).current_r(2) * sin(th) + obs_vec_record(i).current_pose(2,2);
%     pred_plot{num_plot_pred} = plot(xunit, yunit,'g','LineWidth',1);
%     
%     num_plot_pred = num_plot_pred +1;
%     xunit = obs_vec_record(i).current_r(3) * cos(th) + obs_vec_record(i).current_pose(1,3);
%     yunit = obs_vec_record(i).current_r(3) * sin(th) + obs_vec_record(i).current_pose(2,3);
%     pred_plot{num_plot_pred} = plot(xunit, yunit,'g','LineWidth',1);
%     
%     num_plot_pred = num_plot_pred +1;
%     xunit = obs_vec_record(i).current_r(4) * cos(th) + obs_vec_record(i).current_pose(1,4);
%     yunit = obs_vec_record(i).current_r(4) * sin(th) + obs_vec_record(i).current_pose(2,4);
%     pred_plot{num_plot_pred} = plot(xunit, yunit,'g','LineWidth',1);
%     
%     num_plot_pred = num_plot_pred +1;
%     xunit = obs_vec_record(i).current_r(5) * cos(th) + obs_vec_record(i).current_pose(1,5);
%     yunit = obs_vec_record(i).current_r(5) * sin(th) + obs_vec_record(i).current_pose(2,5);
%     pred_plot{num_plot_pred} = plot(xunit, yunit,'g','LineWidth',1);
%     
%     %OBS bound future
%     for k=1:5
%         for t=1:pred_samples
%             if isempty(obs_vec_record(i).predicted_bounds{k}{t}.r)==0
%                 num_plot_pred = num_plot_pred +1;
%                 xunit = obs_vec_record(i).predicted_bounds{k}{t}.r * cos(th) + obs_vec_record(i).predicted_bounds{k}{t}.center(1);
%                 yunit = obs_vec_record(i).predicted_bounds{k}{t}.r * sin(th) + obs_vec_record(i).predicted_bounds{k}{t}.center(2);
%                 pred_plot{num_plot_pred} = plot(xunit, yunit,'g','LineWidth',1); 
%             end
%         end
%     end
% 
%     drawnow
%     pause(0.1);
%     set(rob,'Visible','off')
%     set(obs1_prev,'Visible','off')
%     set(obs2_prev,'Visible','off')
%     set(obs3_prev,'Visible','off')
%     set(obs4_prev,'Visible','off')
%     set(obs5_prev,'Visible','off')
%     set(obs1_now,'Visible','off')
%     set(obs2_now,'Visible','off')
%     set(obs3_now,'Visible','off')
%     set(obs4_now,'Visible','off')
%     set(obs5_now,'Visible','off')
%     set(rob_r_plot,'Visible','off')
%     if ~isempty(select_traj)
%        set(select_traj,'Visible','off')
%     end
%     
%     for io = 1:length(graph_plot)
%           set(graph_plot{io},'Visible','off')
%     end
%     for iq = 1:num_plot_pred
%           set(pred_plot{iq},'Visible','off')
%     end
% end
h_vex=[];
for k=1:length(rrt_obj.graph_h)
    node_sel = rrt_obj.graph_h(k).node_select;
    h_vex=[h_vex rrt_obj.graph_h(k).node(node_sel).hr];
end

t=0:0.5:length(u_traj)*0.5-0.5;

figure(8)
subplot(2,1,1)
plot(t,u_traj(1,:),'b','LineWidth',2)
hold
plot(t,u_traj(2,:),'Color',darkGreen,'LineWidth',2)
set(gca,'FontSize',18)
ylabel('Control Inputs','Interpreter','latex','fontsize',20)
legend('Velocity $(m/s)$','Angular Velocity $(rad/s)$','Interpreter','latex','fontsize',20)
axis([0 length(u_traj)*0.5-0.5 -0.33 1])


subplot(2,1,2)
plot(t(1:length(h_vex)),h_vex,'b','LineWidth',2)
set(gca,'FontSize',18)
xlabel('Time $(s)$','Interpreter','latex','fontsize',20)
ylabel('Safety Measure','Interpreter','latex','fontsize',20)
axis([0 length(u_traj)*0.5-0.5 -1 max(h_vex)+1])

%minimum safety
h_vex=[];
for k=1:length(rrt_obj.graph_h)
    node_sel = rrt_obj.graph_h(k).node_select;
    h_vex=[h_vex rrt_obj.graph_h(k).node(node_sel).hr_min];
end

t=0:0.5:length(u_traj)*0.5-0.5;

figure(9)
subplot(2,1,1)
plot(t,u_traj(1,:),'b','LineWidth',2)
hold
plot(t,u_traj(2,:),'Color',darkGreen,'LineWidth',2)
set(gca,'FontSize',18)
ylabel('Control Inputs','Interpreter','latex','fontsize',20)
legend('Velocity $(m/s)$','Angular Velocity $(rad/s)$','Interpreter','latex','fontsize',20)
axis([0 length(u_traj)*0.5-0.5 -0.33 1])


subplot(2,1,2)
plot(t(1:length(h_vex)),h_vex,'b','LineWidth',2)
set(gca,'FontSize',18)
xlabel('Time $(s)$','Interpreter','latex','fontsize',20)
ylabel('Safety Measure','Interpreter','latex','fontsize',20)
axis([0 length(u_traj)*0.5-0.5 -1 max(h_vex)+1])
