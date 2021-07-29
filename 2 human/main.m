% clc
 clear all
% close all

% Start ROS connection
rosshutdown
rosinit('http://192.168.0.20:11311')
pause(1);
handles = rossubscriber('/gazebo/model_states', 'BufferSize', 25); % 3:actor1, 4:actor2, 5: hsrb
pause(1);
rob_pos = handles.LatestMessage.Pose(5); %extract robot pose
rob_orient = quat2eul([rob_pos.Orientation.W rob_pos.Orientation.X rob_pos.Orientation.Y rob_pos.Orientation.Z]); %quaternion to euler
current_state=[rob_pos.Position.X;rob_pos.Position.Y;rob_orient(1)]; % Robot start pose
pub = rospublisher('/hsrb/command_velocity'); %publisher to HSR actuator
pause(1);
tw = rosmessage('geometry_msgs/Twist');
control_rate=0.15;
%Create subscriber to read the controller commands
node2 = ros.Node('/test_node_2');
control_node = ros.Subscriber(node2,'/chatter','geometry_msgs/Twist');
pause(1);

x_goal = [0; 5];
u_traj=[];
while (norm(current_state(1:2)-x_goal)>0.5)
    
    rob_pos = handles.LatestMessage.Pose(5); %extract robot pose
    rob_orient = quat2eul([rob_pos.Orientation.W rob_pos.Orientation.X rob_pos.Orientation.Y rob_pos.Orientation.Z]);
    current_state=[rob_pos.Position.X;rob_pos.Position.Y;rob_orient(1)]; % Robot current pose
    
    %Read the control command from controller
    u=[control_node.LatestMessage.Linear.X;control_node.LatestMessage.Angular.Z];
    u_traj = [u_traj u];
    
    %Publish the control on HSR actuator
    tw.Linear.X = u(1,1);
    tw.Angular.Z = u(2,1);
    send(pub,tw)
    pause(0.15)
%     if (norm(current_state(1:2)-x_goal)>0.5)
%         send(pub,tw)
%         pause(0.1)
%     end
    
end

%Plot control commands
t=0:control_rate:length(u_traj)*control_rate-control_rate;

figure(1)
subplot(2,1,1)
plot(t,u_traj(1,:),'b','LineWidth',2)
xlabel('$$t~(s)$$','Interpreter','latex','fontsize',20)
ylabel('$v~(\frac{m}{s})$','Interpreter','latex','fontsize',20)

subplot(2,1,2)
plot(t,u_traj(2,:),'b','LineWidth',2)
xlabel('$$t~(s)$$','Interpreter','latex','fontsize',20)
ylabel('$w~(\frac{rad}{s})$','Interpreter','latex','fontsize',20)

save('u_traj_actuator.mat','u_traj','t')