classdef robot < handle
    %robot object (Bicycle model)
    properties
        
        T_s                   %Sample time
        x_start               %initial position
        x_goal                %goal point
        state_traj = [];      % robot state trajectory
        input_traj = [];      % robot input trajectory
        current_state         %state of the robot in real time
        v_max                 %maximum velocity
        w_max                 %maximum steering rate
        
        %motion and approximated model
        motion  %robot motion model
        f       %robot approximated models
        g
        f_r     
        

        %rrt planner properties
        planner                 %prm object
        %.....planner properties
    end
    methods
        
        %constructor method
        function obj=robot(T_s,x_start,x_goal,v_max,w_max,l)
           obj.T_s = T_s;
           obj.x_start = x_start;
           obj.x_goal = x_goal;
           obj.current_state = x_start;
           obj.state_traj = x_start;
           obj.v_max = v_max;
           obj.w_max = w_max;
           
           %motion model: Bicycle model
           
           %real motion model
           f_rob = @(x) [0;0;0];
           g_rob = @(x) [cos(x(3)) 0;sin(x(3)) 0;0 1];
           obj.motion = @(x,u) obj.T_s*(f_rob(x)+g_rob(x)*[u(1); u(2)])+x;
           
           %approximated model
           obj.f = @(x)  [0 ;0; 0]; 
           obj.g = @(x) [cos(x(3)) -l*sin(x(3)); sin(x(3)) l*cos(x(3)); 0 1];
           obj.f_r = @(t,x,u) f(x)+g(x)* [u(1); u(2)];
           %obj.F_r = f_r(0, x_r_s, u_s);
        end
         
        %Drive
        function drive(obj,u)
            obj.current_state = obj.motion(obj.current_state,u);
            obj.state_traj = [obj.state_traj obj.current_state];
            obj.input_traj = [obj.input_traj u];
        end
    end
end







