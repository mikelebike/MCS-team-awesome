%Final Project
close all;
clear all;

%[] kolla omega predator om den behövs?
%[] vad är food





%function []= run_FinalProject(N_boid,N_hoick,tot_time)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Parameters

L=400;      %system size
N_boid = 80;     %Nr of boids
N_hoick = 8; %Nr of hoicks
dt = 0.1;   %Time-step
R_r = 1;    %repulsion radius

R_o = 10;      %Orientation radius
R_a =  20;     %attraction radius

v_boid = 0;
theta_boid  = pi;       %turning angle for boids
theta_hoick =pi;        %turning angle for hoicks
phi =0;

A_s =0;      %Possible sighting area
A_m =0;       %Possible movement area

omega_food =0;          %food preference
omega_hoick =0;         %anti-predator preference
omega_boid=0;           %preference to stay in the center of the group?

distribution_food=0;
distribution_predator=0;

e_boid =0;              %noise
e_hoick=0;              %Hoick noise  
warm_up =10000;         %Warm up time, 15 minutes in the paper
lifetime_hoick = 1000;  %Lifetime of a hoick
lifetime_food=0;        %Food lifetime

tot_time=100;   %Totalt time

omega_food =0; 
omega_predator =0; 
food = false;
makemovie=false;
plot_sim=true;
food=false;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Set up movie

if plot_sim
   fig=figure;
end
if makemovie
    filename = 'Flocks And Predators 1.avi';
    video = VideoWriter(filename);
    video.FrameRate = 5;
    video.Quality = 100;
    open(video);
end 


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

hoick_nr=1;
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
    %Distance to the boids
    r_ij_prey(:,1)=((x_hoick(hoick_nr,t)-x_boid(:,t)).^2 + (y_hoick(hoick_nr,t)-y_boid(:,t)).^2).^0.5;
    r_hat_x_prey(:,1) = (x_hoick(hoick_nr,t)-x_boid(:,t))./r_ij_prey(:,1);               %Unit direction
    r_hat_y_prey(:,1)= (y_hoick(hoick_nr,t)-y_boid(:,t))./r_ij_prey(:,1);                  %Unit direction
    
    [r_ij_prey_sort,index1] = sort(r_ij_prey,1);               %Sorting the direction vector r_ij_predator
    
    %%%%%%%%%Find v
    %%%%%%%%%Find v_b
    %Looking at all the boids inside the repulsion radius
    inside_R_r = r_ij_prey(r_ij_prey<R_r);

    
    if any(inside_R_r)      %If there are any boids inside the repulsion radius
        for j=1:length(inside_R_r)
            if v_x_hoick(hoick_nr,t)*r_hat_x_prey(index1(j),1) +v_y_hoick(hoick_nr,t)*r_hat_y_prey(index1(j),1)> v_hoick(hoick_nr,t)*cos(theta_hoick/2)
                x_boid(index1(1),t+1:end)=NaN;
                y_boid(index1(1),t+1:end)=NaN;
                
                x_hoick(hoick_nr,t+1:end)=NaN;
                y_hoick(hoick_nr,t+1:end)=NaN;
                
                hoick_nr = hoick_nr +1;            %Next hoick is chosen                
                disp('eaten');
                break
            end
        end        
    end
    
    %%%%%%Find v_prey
    r_ij_prey = ((x_hoick(hoick_nr,t)-x_boid(:,t)).^2 + (y_hoick(hoick_nr,t)-y_boid(:,t)).^2).^0.5;
    [r_ij_vec, index_prey]= sort(r_ij_prey);
    
    v_prey_x = 0;
    v_prey_y = 0;
    
    for k=1:length(r_ij_vec(r_ij_vec<R_a))
        if  v_x_hoick(hoick_nr,t)*r_hat_x_prey(index_prey(k)) +v_y_hoick(hoick_nr,t)*r_hat_y_prey(index_prey(k))> v_hoick(hoick_nr,t)*cos(theta_hoick/2)
            v_prey_x = x_boid(index_prey(k),t);
            v_prey_y = y_boid(index_prey(k),t);
            break;
        end
    end
    
    %%%%%Find v_noise
    v_noise_x = randn(1,1);
    v_noise_y = randn(1,1);
    
    v_noise_x = v_noise_x/(v_noise_x^2 + v_noise_y^2)^0.5;
    v_noise_y = v_noise_y/(v_noise_x^2 + v_noise_y^2)^0.5;
    
    %Add together all the components for the velocity vector
    v_x_hoick(hoick_nr,t+1) = omega_hoick*v_prey_x + e_hoick*v_noise_x;
    v_y_hoick(hoick_nr,t+1) = omega_hoick*v_prey_y + e_hoick*v_noise_y;
    
    %Plot predator           
    if plot_sim
        if abs(x_hoick(hoick_nr,t)-x_hoick(hoick_nr,t+1))<v_hoick(hoick_nr,t) && abs(y_hoick(hoick_nr,t)-y_hoick(hoick_nr,t+1))<v_hoick(hoick_nr,t)
            plot([x_hoick(hoick_nr,t), x_hoick(hoick_nr,t+1)] ,[y_hoick(hoick_nr,t),y_hoick(hoick_nr,t+1)],'r-','markersize',20) 
            axis([0 L 0 L]);
            hold on
            plot(x_hoick(hoick_nr,t+1) ,y_predator(hoick_nr,t+1),'r.','markersize',15)
            xlabel('X position')
            ylabel('Y position')
        end
    end   

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%Boid
    for i=1:N_boid
        if any(isnan(x_boid(i,t+1)))    %Skips this iteration if the value is NaN (dead Boid)
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
            %Kan ta bort att den då det är inte verkligt att de kan
            %stå i varandra så länge de inte ser varandra
            
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
                v_ox= -v_x_boid(index_vo)./sum(v_x_boid(index_vo));
                v_oy = -v_y_boid(index_vo)./sum(v_y_boid(index_vo));
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
            v_b = ((v_ax + v_ox).^2 + (v_ay + v_oy).^2).^0.5;
            v_bx = (v_ax + v_ox)/v_b;
            v_by = (v_ay + v_ay)/v_b;
            
        end
        
        %%%%%%Find v_pf_boid
        r_ij_pf_boid = ((x_boid(i,t)-x_hoick(hoick_nr,t)).^2 + (y_boid(i,t)-y_hoick(hoick_nr,t)).^2).^0.5;
        [r_ij_vec, index_pf]= sort(r_ij_pf_boid);
        
        v_pf_x_boid = 0;
        v_pf_y_boid = 0;
        
        for k=1:length(r_ij_vec(r_ij_vec<R_a))
            if food 
%                 if  v_x(i)*r_hat_x_pf(index_pf(k)) +v_y(i)*r_hat_y_pf(index_pf(k))> v(i)*cos(theta(i)/2)
%                     v_pf_x_boid = x_food;
%                     v_pf_y_boid = y_food;
%                     break;
%                 end
            else
                v_pf_x_boid = -x_hoick;
                v_pf_y_boid = -y_hoick;
                break;
            end
        end
        
        %%%%%Find v_noise
        v_noise_x = randn(1,1);
        v_noise_y = randn(1,1);
        
        v_noise_x = v_noise_x/(v_noise_x^2 + v_noise_y^2)^0.5;
        v_noise_y = v_noise_y/(v_noise_x^2 + v_noise_y^2)^0.5;
        
        %Add together all the components for the velocity vector
        v_x_boid(i,t+1) = v_bx + omega_boid*v_pf_x_boid(i,t) + e_boid*v_noise_x;
        v_y_boid(i,t+1) = v_by + omega_boid*v_pf_y_boid(i,t) + e_boid*v_noise_y;
        
        %Plot boids           
        if plot_sim                
            if abs(x_boid(i,t)-x_boid(i,t+1))<v_boid(i,t) && abs(y_boid(i,j)-y_boid(i,j+1))<v_boid(i,t)
                plot([x_boid(i,t), x_boid(i,t+1)] ,[y_boid(i,t),y_boid(i,t+1)],marker1,'markersize',7) %plots the first half of the particles in black
                axis([0 L 0 L]);
                hold on
                plot(x_boid(i,t+1) ,y_boid(i,t+1),'k','markersize',5)
                title(['Timestep: ',num2str(t)])
                xlabel('X position') 
                ylabel('Y position')
            end
        end       
        
    end
        
    %Making the video
    if makemovie && plot_sim
        hold off
        F(j) = getframe(gcf);  %Gets the current frame
        writeVideo(video,F(j)); %Puts the frame into the videomovie
    end
   
end

if makemovie && plot_sim
    close(video);  %Closes movie
end