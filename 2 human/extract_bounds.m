function predict_bounds = extract_bounds(L,pred_samples,T_s_o,T_s_r,pose_init,r_init)
    N=length(L);
    prob = 0.5;
    step_size = round(T_s_o/T_s_r+0.00000001); % interpolation steps
    I = (1:step_size)/step_size;
    inetval_x=-7.1:0.1:7.1;
    inetval_y=-1.7:0.1:6.7;
    %Obtain more probable regions from L
    predict_bounds = cell(1,N);
    for t=1:pred_samples
        for i=1:N
            try
                Q = contourc(inetval_y,inetval_x,L{i}{t},[prob*max(max(L{i}{t})) prob*max(max(L{i}{t}))]); %exrtract contours above prob level
            catch ME
                [prob*max(max(L{i}{t})) prob*max(max(L{i}{t}))]
            end
            [~,col_Q] = size(Q);
            %extract center of the region
            center = [sum(Q(1,2:end))/(col_Q-1); sum(Q(2,2:end))/(col_Q-1)];
            prev_center = center;
            %extract radius of the region
            sol=Q-center;
            size_vec=vecnorm(sol);
            r=max(max(size_vec(2:end)));
            prev_r=r;
            %modify the prediction of r is pretty large
            if r>1
%                 center = prev_center+T_s_r;
%                 r = prev_r;
                  Q = contourc(inetval_y,inetval_x,L{i}{t},[0.95*max(max(L{i}{t})) 0.95*max(max(L{i}{t}))]); %exrtract contours above prob level
                  [~,col_Q] = size(Q);
                  %extract center of the region
                  center = [sum(Q(1,2:end))/(col_Q-1); sum(Q(2,2:end))/(col_Q-1)];
                  prev_center = center;
                  %extract radius of the region
                  sol=Q-center;
                  size_vec=vecnorm(sol);
                  r=max(max(size_vec(2:end)));
                  if prev_r>1
                      r=0.5;
                  end
                  prev_r=r;                  
            end
            %save bounds
            if max(max(L{i}{t})) == 0
                for k=1:step_size
                   predict_bounds{i}{round(step_size*(t-1)+k+0.0001)}.r = [];
                   predict_bounds{i}{round(step_size*(t-1)+k+0.0001)}.center = [];
                   predict_bounds{i}{round(step_size*(t-1)+k+0.0001)}.in_scene = false; 
                end
            else
                for k=1:step_size
                   if t>1 
                       predict_bounds{i}{round(step_size*(t-1)+k+0.0001)}.r = I(k)*r+(1-I(k))*predict_bounds{i}{round(step_size*(t-1)+0.0001)}.r;
                       predict_bounds{i}{round(step_size*(t-1)+k+0.0001)}.center = I(k)*flip(center)+(1-I(k))*predict_bounds{i}{round(step_size*(t-1)+0.0001)}.center;
                       predict_bounds{i}{round(step_size*(t-1)+k+0.0001)}.in_scene = true; 
                   else
                       predict_bounds{i}{round(step_size*(t-1)+k+0.0001)}.r = I(k)*r+(1-I(k))*r_init(i);
                       predict_bounds{i}{round(step_size*(t-1)+k+0.0001)}.center = I(k)*flip(center)+(1-I(k))*pose_init(:,i);
                       predict_bounds{i}{round(step_size*(t-1)+k+0.0001)}.in_scene = true; 
                   end
                end
            end
        end
    end
end