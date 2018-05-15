%Final Project

function []= run_FinalProject(N_boid,N_hoick,tot_time)

%%%%%%%%%%Initialize boid
% x(i,j) gives the x coordinate of the ith particle at time j
x_boid=zeros(N_boid,tot_time+1);  %define initial x coordiantes of all particles
x_boid(:,1)=L*rand(N_boid,1);

y_boid=zeros(N_boid,tot_time+1);
y_boid(:,1)=L*rand(N_boid,1);

v_boid = zeros(N_boid,tot_time+1);   %velocity vector for all boids
v_y_boid=zeros(N_boid,tot_time+1);
v_x_boid=zeros(N_boid,tot_time+1);

%%%%%%%%%%Initialize hoick
x_hoick=zeros(N_hoick,tot_time+1);  %define initial x coordiantes of all particles
x_hoick(:,1)=L*rand(N_hoick,1);

y_hoick=zeros(N_hoick,tot_time+1);
y_hoick(:,1)=L*rand(N_hoick,1);

v_hoick = zeros(N_hoick,tot_time+1);      %velocity vector for all hoicks
v_x_hoick=zeros(N_hoick,tot_time+1);
v_y_hoick=zeros(N_hoick,tot_time+1);

for t=1:tot_time
        
%     if boid
%         x = x_boid;
%         y=y_boid;
%         T=T_boid;
%         v=v_boid;
%         v_x = v_x_boid;
%         v_y = v_y_boid;
%         e = e_boid;
%         theta = theta_hoick;
%         
%     else
%         if food
%             r_ij_prey
%             omega = omega_food;
%             
%             
%         else %hoick
%             x = x_pf;
%             y= y_hoick;
%             T= T_pf;
%             v= v_pf;
%             v_x = v_prey_x;
%             v_y = v_prey_y;
%             e = e_hoick;
%             theta = theta_hoick;
%             
%             omega=omega_predator;            
%         end
%     end
    
    
    
    %%%%%%%%%%%%%%%Predator
    if any(isnan(x_hoick(i,t+1)))    %Skips this iteration if the value is NaN (successfull or dead hoick)
        continue
    end
    
    %Distance to the boids
    r_ij_prey(:,1)=((x_hoick(t)-x_boid(:,t)).^2 + (y_hoick(t)-y_boid(:,t)).^2).^0.5;
    r_hat_x_prey(:,1) = (x_hoick(t)-x_boid(:,t))./r_ij_prey(:,1);               %Unit direction
    r_hat_y_prey(:,1)= (y_hoick(t)-y_boid(:,t))./r_ij_prey(:,1);                  %Unit direction
    
    [r_ij_predator_sort,index1] = sort(r_ij_prey,1);               %Sorting the direction vector r_ij_predator
    
    %%%%%%%%%Find v
    %%%%%%%%%Find v_b
    %Looking at all the boids inside the repulsion radius
    inside_R_r = r_ij_prey(r_ij_prey<R_r);

    
    if any(inside_R_r)      %If there are any boids inside the repulsion radius
        for j=1:length(inside_R_r)
            if v_x_hoick(i)*r_hat_x_prey(index1(j)) +v_y_hoick(i)*r_hat_y_prey(index1(j))> v_hoick(i)*cos(theta_hoick(i)/2)
                x_boid(index1(1),t+1:end)=NaN;
                y_boid(index1(1),t+1:end)=NaN;
                %T_boid(index1(1),t+1:end)=NaN;
                disp('eaten');
                break
            end
        end        
    end
    
    %%%%%%Find v_prey
    r_ij_prey = ((x_hoick(hoick_nr,t)-x(:,t)).^2 + (y_hoick(hoick_nr)-y(:,t)).^2)^0.5;
    [r_ij_vec, index_prey]= sort(r_ij_prey);
    
    v_prey_x = 0;
    v_prey_y = 0;
    
    for k=1:(r_ij_vec<R_a)
        if  v_x_hoick(i)*r_hat_x_prey(index_pf(k)) +v_y_hoick(i)*r_hat_y_prey(index_pf(k))> v_hoick(i)*cos(theta(i)/2)
            v_prey_x = x_boid(index_pf(k),t);
            v_prey_y = y_boid(index_pf(k),t);
            break;
        end
    end
    
    %%%%%Find v_noise
    v_noise_x = randn(1,1);
    v_noise_y = randn(1,1);
    
    v_noise_x = noise_x/(v_noise_x^2 + v_noise_y^2)^0.5;
    v_noise_y = v_noise_y/(v_noise_x^2 + v_noise_y^2)^0.5;
    
    %Add together all the components for the velocity vector
    v_x_hoick(hoick_nr,t+1) = omega_predator*v_prey_x + e*v_noise_x;
    v_y_hoick(hoick_nr,t+1) = omega_predator*v_prey_y + e*v_noise_y;
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%Boid
    for i=1:N_boid
        if any(isnan(x(i,t+1)))    %Skips this iteration if the value is NaN (dead Boid)
            continue
        end
        
        %Distance to all other particles
        r_ij(:,1)=((x_boid(i,t)-x_boid(:,t)).^2 + (y_boid(i,t)-y_boid(:,t)).^2).^0.5;
        r_hat_x(:,1) = (x_boid(i,t)-x_boid(:,t))./r_ij(:,1);                    %Unit direction
        r_hat_y(:,1)= (y_boid(i,t)-y_boid(:,t))./r_ij(:,1);                %Unit direction
        
        [r_ij_sort,index] = sort(r_ij,1);               %Sorting the direction vector r_ij
        
        
        %%%%%%%%%Find v
        %%%%%%%%%Find v_b
        %Looking at all the boids inside the repulsion radius
        inside_R_r = r_ij(r_ij<R_r);
        
        v_bx=0;
        v_by=0;
        
        if any(inside_R_r)      %If there are any boids inside the repulsion radius
            %Kan ta bort att den d� det �r inte verkligt att de kan
            %st� i varandra s� l�nge de inte ser varandra
            
            for j=1:length(inside_R_r)
                
                if v_x_boid(i)*r_hat_x(index(j)) +v_y_boid(i)*r_hat_y(index(j))> v_boid(i)*cos(theta(i)/2)
                    v_bx = v_bx + sum(r_hat_x(index(j)));
                    v_by = v_by + sum(r_hat_y(index(j)));
                end
                
                v_bx = -v_bx/sum(r_ij(index(j)));
                v_by = -v_by/sum(r_ij(index(j)));
            end
            
            
        else  %No boids in the repulsion area
            
            %%%%%%%%%%Find v_o
            index_vo = find(r_ij>R_r & r_ij<R_o);               %Index for the boids in orientation radius
            v_ox=0;
            v_oy=0;
            if any(index_vo)
                v_ox= -v_x(index_vo)./sum(v_x(index_vo));
                v_oy = -v_y(index_vo)./sum(v_y(index_vo));
            end
            
            %%%%%%%%%Find v_a
            index_va = find(r_ij>R_o & r_ij<R_a);
            v_ax=0;
            v_ay=0;
            
            if any(index_va)
                v_ax = r_hat_x(index_va)./sum(r_ij);
                v_ay = r_hat_y(index_va)./sum(r_ij);
                
            end
            
            %Define v_b
            v_b = ((v_ax + v_ox).^2 + (v_ay + v_ay).^2).^0.5;
            v_bx = (v_ax + v_ox)/v_b;
            v_by = (v_ay + v_ay)/v_b;
            
        end
        
        %%%%%%Find v_pf_boid
        r_ij_pf_boid = ((x-x_pf_boid).^2 + (y-y_pf_boid).^2)^0.5;
        [r_ij_vec, index_pf]= sort(r_ij_pf_boid);
        
        v_pf_x_boid = 0;
        v_pf_y_boid = 0;
        
        for k=1:(r_ij_vec<R_a)
            if  v_x(i)*r_hat_x_pf(index_pf(k)) +v_y(i)*r_hat_y_pf(index_pf(k))> v(i)*cos(theta(i)/2)
                if food
                    v_pf_x_boid = x_pf;
                    v_pf_y_boid = y_pf;
                    break;
                else
                    v_pf_x_boid = -x_pf;
                    v_pf_y_boid = -y_pf;
                    break;
                end
            end
        end
        
        %%%%%Find v_noise
        v_noise_x = randn(1,1);
        v_noise_y = randn(1,1);
        
        v_noise_x = noise_x/(v_noise_x^2 + v_noise_y^2)^0.5;
        v_noise_y = v_noise_y/(v_noise_x^2 + v_noise_y^2)^0.5;
        
        %Add together all the components for the velocity vector
        v_x(i,t+1) = v_bx + omega*v_pf_x_boid + e*v_noise_x;
        v_y(i,t+1) = v_by + omega*v_pf_y_boid + e*v_noise_y;
        
    end
end











