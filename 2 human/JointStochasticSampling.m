function L = JointStochasticSampling(p,maps,K,T)
beta=2;
alpha=10000000;
N = length(p); % # of humans
I = [0.9 0.9]; %Inertial term for action
r_person=0.3; %radius around each person =0.5 m
[i_max,j_max] = env_map.cord2grid(maps{1}.resolution, maps{1}.x_limit(2), maps{1}.y_limit(2), maps{1}.x_limit(1), maps{1}.y_limit(1));
states_dim = [i_max,j_max];
%states_dim = [maps{1}.x_limit*maps{1}.resolution+1 maps{1}.y_limit*maps{1}.resolution+1];
%avg_frame = 10;
theta = zeros(N,1);
v = zeros(N,1);
T_avg = zeros(N,2);
box_filter = true;
for i = 1:N %calculate speeds and headings
    [v(i),theta(i), T_avg(i,:)] = moving_average(p{i});
%     %new piece averaging over samples
%     theta(i)=0;
%     v(i)=0;
%     if length(p{i})>=avg_frame+1
%         for q=1:avg_frame
%             head = atan2((p{i}(end-q+1,2)-p{i}(end-q,2)),(p{i}(end-q+1,1)-p{i}(end-q,1)));
%             if head<-pi/2
%                 head=head+2*pi;
%             end
%             theta(i)=theta(i)+head;
%             v(i)=v(i)+sqrt((p{i}(end-q,2)-p{i}(end-q+1,2))^2+(p{i}(end-q,1)-p{i}(end-q+1,1))^2);
%         end
%         v(i)=v(i)/avg_frame;
%         theta(i)=theta(i)/avg_frame;
%         
%     else
%         for q=1:length(p{i})-1
%             head = atan2((p{i}(end-q+1,2)-p{i}(end-q,2)),(p{i}(end-q+1,1)-p{i}(end-q,1)));
%             if head<-pi/2
%                 head=head+2*pi;
%             end
%             theta(i)=theta(i)+head;
%             v(i)=v(i)+sqrt((p{i}(end-q,2)-p{i}(end-q+1,2))^2+(p{i}(end-q,1)-p{i}(end-q+1,1))^2);
%         end
%         v(i)=v(i)/(length(p{i})-1);
%         theta(i)=theta(i)/(length(p{i})-1);
%     end
%     if theta(i)<0
%         theta(i)=theta(i)+2*pi;
%     end
end
% initialize maps 
for i=1:N
    for t=1:T
        L{i}{t}=zeros(states_dim(1),states_dim(2));
    end
end

% sample K joint path for each person
actions=zeros(K,1); %stores action samples for each person at each sample
for k=1:K
    %set initial states of each person
    for i=1:N
        person{i}.v = v(i);
        person{i}.theta = theta(i);
        person{i}.s = p{i}(end,:);
        %person{i}.s = T_avg(i,:);
        person{i}.g_map_index = GoalSampler(p{i},maps,beta);
        person{i}.out_bound = 0;
    end    
    for t=1:T
       for i=1:N
          if person{i}.out_bound == 0
              %sample random action of person i according to its goal
              a{i}{t} = ActionSampler(person{i}, maps{person{i}.g_map_index}, alpha);

              %apply inertia for smoothing action
              a{i}{t}(1) = (1-I(1))*a{i}{t}(1) + I(1)*person{i}.v;
              a{i}{t}(2) = (1-I(2))*a{i}{t}(2) + I(2)*person{i}.theta;
%               if i<=2
%                   a{i}{t} = (1-I).*a{i}{t} + I.*[person{i}.v  person{i}.theta];
%               else
%                   person{i}.v=0; 
%                   a{i}{t} = (1-[1 1]).*a{i}{t} + [1 1].*[person{i}.v  person{i}.theta];
%               end
              %Calculate state transition of person i executing action a ----->
              %--->fix this (add a part to normalize states out of the bound)
              s_new = env_map.next_state(maps{person{i}.g_map_index}.resolution, person{i}.s, a{i}{t}, maps{person{i}.g_map_index}.dt);

              %Social force on person i given current agents’ positions s:
              %F_s = SocialForce(person, i, s_new); %F_s: new direction, s_new: new position before applying social force

              %Modify transition of person i given the current social force:
              %s_next{i} = person{i}.s + F_s; %apply the social force
              s_next{i}=s_new; %SINCE IN OUR SIMULATION THE SOCIAL FORCE IS NOT BEING CONSIDERED, HERE WE DO NOT CONSIDER EITHER
              %check if the given pose is in bound
              if (s_next{i}(1)>=maps{person{i}.g_map_index}.x_limit(2)) || ...
                      (s_next{i}(1)<=maps{person{i}.g_map_index}.x_limit(1)) ||...
                      (s_next{i}(2)>=maps{person{i}.g_map_index}.y_limit(2)) ||...
                      (s_next{i}(2)<=maps{person{i}.g_map_index}.y_limit(1))
                  person{i}.out_bound = 1;
              else
                  [i_x, j_x]=env_map.cord2grid(maps{person{i}.g_map_index}.resolution, s_next{i}(1), s_next{i}(2), maps{1}.x_limit(1), maps{1}.y_limit(1));
                  if maps{person{i}.g_map_index}.map(i_x,j_x).C == 1
                      person{i}.out_bound = 1;
                  end
              end
              
              % Inflate and add the next position si_new of person i to the occupancy map:
              if person{i}.out_bound == 0
                  [r_x,r_y]=meshgrid(1-j_x:states_dim(2)-j_x,1-i_x:states_dim(1)-i_x); %inflate the point
                  nhood = r_x.^2 + r_y.^2 <= (r_person*maps{person{i}.g_map_index}.resolution)^2;
                  L{i}{t} = L{i}{t} + nhood;
              end
          end
       end
       
       %Update current positions, orientations and velocities of all human agents:
       for i=1:N
           if person{i}.out_bound == 0
               v_new = norm(person{i}.s-s_next{i});
               theta_new = atan2(s_next{i}(2)-person{i}.s(2),s_next{i}(1)-person{i}.s(1));
               
               if theta_new<0
                  theta_new=theta_new+2*pi;
               end
%                person{i}.v = v_new;
%                person{i}.theta = theta_new;
               person{i}.v = a{i}{t}(1);
               person{i}.theta = a{i}{t}(2);
               person{i}.s = s_next{i};
           end
        end  
    end
end

%normalize the map
for i=1:N
    for t=1:T
        if box_filter
            L{i}{t} = imboxfilt(L{i}{t},3);
            L{i}{t} = imboxfilt(L{i}{t},3);
            L{i}{t} = imboxfilt(L{i}{t},3);
        end
        L{i}{t} = L{i}{t}/K;
    end
end
end

function [v,theta,loc] = moving_average(tracklet)
T_avg = movmean(tracklet, [2,2], 1); %smoothing path
[row, ~] = size(tracklet);
if row>1
    diff_T = diff(T_avg);
    diff_T_avg = movmean(diff_T,[5,5]); %smoothin first diff
    v = hypot(diff_T_avg(:,1),diff_T_avg(:,2));
    theta = atan2(diff_T_avg(:,2),diff_T_avg(:,1));
    if theta<0
        theta = theta+2*pi;
    end
    v=v(end);
    theta=theta(end);
else    %just having single point
    v = 0;   
    theta = 4.71;
end
v= round(v,1); 
loc = T_avg(end,:);
end