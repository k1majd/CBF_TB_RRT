classdef env_map < handle
    properties
        map
        goal
        x_limit
        y_limit
        resolution
        v_max = 1.3;              %max speed of human
        v_min = 0;              %min speed of human
        theta_max = 7*pi/4;       %max heading of human
        v_resolution = 0.1;       %speed resolution
        theta_resolution = pi/4;  %heading resolution
        a_v_grid                  %speed table
        a_theta_grid              %heading table
        dt = 1;  %sample time
    end
    methods
        %% Method 1
        function obj=env_map(env_dim_x,env_dim_y,resolution,goal)
           obj.goal=goal;
           obj.x_limit = env_dim_x; 
           obj.y_limit = env_dim_y; 
           obj.resolution = resolution;
           [obj.a_v_grid, obj.a_theta_grid] = meshgrid(obj.v_min:obj.v_resolution:obj.v_max,0:obj.theta_resolution:obj.theta_max); 
           %initialize map
           [cord_y, cord_x] = meshgrid(env_dim_y(1):1/resolution:env_dim_y(2), env_dim_x(1):1/resolution:env_dim_x(2)); 
           [~,c_v]=size(obj.a_v_grid);
           [r_theta,~]=size(obj.a_theta_grid);
           [i_last,j_last] = obj.cord2grid(obj.resolution, env_dim_x(2), env_dim_y(2), env_dim_x(1), env_dim_y(1));
           %obj.map=zeros(i_last, j_last);
           for j=1:j_last 
               for i=1:i_last
                   obj.map(i,j).cord=[cord_x(i,j) cord_y(i,j)];  
                   obj.map(i,j).V=-10;
                   obj.map(i,j).Q=zeros(r_theta,c_v);                   
                   if (cord_x(i,j)<=-1.6129)
                       if (cord_y(i,j)>=1.4845)
                           obj.map(i,j).C=1;                          %assume the area is accupied (occupancy)
                       else
                           obj.map(i,j).C=0.2;   
                       end
                   elseif (cord_x(i,j)>=1.5871)
                       if (cord_y(i,j)>=1.4845)
                           obj.map(i,j).C=1;                          %assume the area is accupied (occupancy)
                       else
                           obj.map(i,j).C=0.2;   
                       end
                   else
                       obj.map(i,j).C=0.2;                          %assume the area is not accupied (occupancy)
                   end
                   obj.map(i,j).policy=[];
               end
           end
           value_iteration(obj);
        end
        
        %% Method 2
        function val = in_bound(obj, s)
            val = true;
            if (s(1)>obj.x_limit(2)) || (s(1)<obj.x_limit(1)); val = false; end
            if (s(2)>obj.y_limit(2)) || (s(2)<obj.y_limit(1)); val = false; end
        end
        
        %% Method 3
        function reward=reward_cal(obj,s,a)
           w1=1;
           w2=1;
           s_next=obj.next_state(obj.resolution, s, a, obj.dt);
           [i, j]=obj.cord2grid(obj.resolution, s_next(1), s_next(2), obj.x_limit(1),obj.y_limit(1));
           if s_next==obj.goal; reward=0; else; reward=-w1*obj.map(i,j).C-w2*norm(s_next-s,1); end
        end
        
        %% Method 4
        function value_iteration(obj)
            w_a=0.5;
            [row, col]=size(obj.map);
            [row_a, col_a]=size(obj.map(1,1).Q);
            
            V=[obj.map.V];
            V_old=V+1;
            pp=1;
            e=norm(V-V_old);
            e_old=norm(V-V_old)+1;
            while (norm(V-V_old)~=0)
                V_old=[obj.map.V];
                e_old=norm(V-V_old);
                %iterate on states (coordinates)
                for i=1:row
                    for j=1:col
                        s=obj.map(i,j).cord; %current location
                        policy_opt = [];
                        Q_prev = -inf;
                        %iterate on actions
                        for ii=1:row_a
                           for jj=1:col_a
                               a=[obj.a_v_grid(1,jj) obj.a_theta_grid(ii,1)];
                               s_next=obj.next_state(obj.resolution, s, a, obj.dt);
                               [i_next, j_next]=obj.cord2grid(obj.resolution, s_next(1), s_next(2),obj.x_limit(1),obj.y_limit(1));
                               if in_bound(obj, s_next)
                                   reward=reward_cal(obj, s, a);
                                   obj.map(i,j).Q(ii,jj) = w_a*reward+obj.map(i_next, j_next).V;
                               else
                                   obj.map(i,j).Q(ii,jj) = -1000;
                               end
                               %save opt policy
                               if obj.map(i,j).Q(ii,jj) > Q_prev 
                                   policy_opt = a';
                                   Q_prev = obj.map(i,j).Q(ii,jj);
                               elseif obj.map(i,j).Q(ii,jj) == Q_prev
                                   policy_opt = [policy_opt a'];
                               end 
                               %if obj.map(i,j).Q(ii,jj) == Q_prev; policy_opt = [policy_opt a']; end
                           end
                        end
                        obj.map(i,j).V = max(max(obj.map(i,j).Q));
                        obj.map(i,j).policy = policy_opt; 
                    end
                end
                V=[obj.map.V];
                norm(V)
                e=norm(V-V_old)
                norm(e-e_old);
            end
        end
    end
    methods(Static)
       %% Method 5
       function [cor_new]=round_up(resolution,cor)  %round down a coordinate to the closest resolution point
            cor_new = ceil(cor) + ceil( (cor-ceil(cor))/(1/resolution)) * (1/resolution);      
       end 
       
       %% Method 6
       function [cor_new]=round_down(resolution,cor)  %round down a coordinate to the closest resolution point
            cor_new = floor(cor) + floor( (cor-floor(cor))/(1/resolution)+(1/100000*resolution)) * (1/resolution);           
       end
       
       %% Method 7
       function [i, j]=cord2grid(resolution, x, y, x_l, y_l)
           i=int32((env_map.round_down(resolution,x)-env_map.round_down(resolution,x_l))*resolution+1); 
           j=int32((env_map.round_down(resolution,y)-env_map.round_down(resolution,y_l))*resolution+1); 
       end
       
       %% Method 8
       function [x, y]=grid2cord(resolution, i, j)
           x = (i-1)/resolution;  
           y = (j-1)/resolution;  
       end
       
       %% Method 9
       function s_next=next_state(resolution, s, a, dt)
            s_next(1)=env_map.round_down(resolution, dt*a(1)*cos(a(2))+s(1));
            s_next(2)=env_map.round_down(resolution, dt*a(1)*sin(a(2))+s(2));
       end
    end
end