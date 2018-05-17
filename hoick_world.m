%This code is for extension of the models, implementing hoicks in the boid
%world

close all
clear all

%LEGEND FOR STUFF
% CHECK
% TEMPORARY
% DELETE
% OPTIMIZE

%INITIALIZE PARAMETERS
L=400;                  %System size
N_boid = 40;            %Nr of boids
N_hoick = 1;         %Nr of predators
%dt = 0.1;              % CHECK Time-step, why is this used?
R_r = 1;                %repulsion radius
R_o = 4;                %Orientation radius
R_a = 15;               %Attraction radius
v_evolve = 2;           % CHECK(no evolution for boids) the evolvable speed of boid

theta_boid  = pi/4;      %turning angle for boids
theta_hoick = pi/4;      %turning angle for hoicks
phi_boid = pi;           %viewing angle
phi_hoick = pi;          %viewing angle

A_s =0;                 % CHECK do we use this? Possible sighting area
A_m =0;                 % CHECK do we use this? Possible movement area

e_boid = 0.2;           %Sensitivity to noise
warm_up = 10000;        %CHECK do we really need this? %Warm up time, 15 minutes in the paper
tot_time = 10;          %Totalt time


%DEFINE HELPFUL VECTORS
r = zeros(N_boid + N_hoick,1);       %r is the distance from current boid to all other boids
rx_hat = zeros(N_boid + N_hoick,1);  %unit vector for x component
ry_hat = zeros(N_boid + N_hoick,1);    %unit vector for y component


%GRAPHICS STUFF
fig=figure;
marker1 = 14;



%------INITIALIZE BOIDS AND PREDATORs-------%
% x(i,t) gives the x coordinate of the ith particle at time t
x=zeros(N_boid + N_hoick,tot_time+1);    %define initial x coordiantes for boids
x(:,1)=L/2+L/5*rand(N_boid + N_hoick,1); %initial positions

y=zeros(N_boid + N_hoick,tot_time+1);    %define initial y coordinates for boids
y(:,1)=L/2+L/5*rand(N_boid + N_hoick,1); %initial positions

v = zeros(N_boid + N_hoick,tot_time+1);   %velocity vector for all boids
vy = zeros(N_boid + N_hoick,tot_time+1);
vx = zeros(N_boid + N_hoick,tot_time+1);


%ITERATE OVER TIME
for t = 1:tot_time
    
    
    rx_temp = repmat(x(:,1)',numel(x(:,1)),1); %create matrix of all individuals positions in x
    ry_temp = repmat(y(:,1)',numel(y(:,1)),1); %create matrix of all individuals positions in y
    
    rx_hat = (rx_temp-x(:,1));               %find distance vector between elements x-components
    ry_hat = (ry_temp-y(:,1));               %find distance vector between elements x-components
    
    r = (rx_hat.^2+ry_hat.^2).^0.5+inf*eye(N_boid + N_hoick);         %find euclidian distance and add term to avoid division by zero.
    rx_hat = rx_hat./r;                     %normalize to create unit direction vector
    rx_hat = rx_hat./r;                     %normalize to create unit direction vector
    
    
    [r_sort_m,index] = sort(r');               % OPTIMIZE by making this in the beginning   %Sorting the direction vector r_ij
    index = index';                         %turn index right so every row represents a boid
    r_sort_m = r_sort_m';                   %do the same operations for r_sort_m as for index.
    
    r_boid = r([1:N_boid]);                   %Vector defining distances between boids, deleting distance to hoick
    [r_sort_boid,boid_index] = sort(r_boid'); %Sort the vector for finding closest boids
    boid_index = boid_index';                 %Turn index right so every row represents a boid
    r_sort_boid = r_sort_boid';               % CHECK do we even use this? Do the same operations for r_sort_boid as for index.
    
    r_hoick = r([N_boid+1:N_hoick]);             %Vector defining distances between boids and hoicks
    [r_sort_hoick,hoick_index] = sort(r_hoick');
    hoick_index = hoick_index';
    r_sort_hoick = r_sort_hoick';                % CHECK do we even use this?
    
    
    %ITERATE OVER BOIDS
    for i=1:N_boid
        if isnan(x(i,t))    %Skips this iteration if the value is NaN (dead Boid)
            continue
        end
        
       % r_sort = r_sort_boid(i,:); %CHECK do we even use this?
        index_b = boid_index(i,:);
        
        %-------FIND INTERACTION WITH OTHER BOIDS------------
        %Looking at all the boids inside the repulsion radius
        inside_R_r = r_boid(r_boid < R_r);
        
        
        %------ SEE IF ANY BOIDS IN REPULSION AREA--------
        if any(inside_R_r)
            vx_b = 0;
            vy_b = 0;
            
            for j=1:length(inside_R_r)
                %SEE IF WITHING VIEWING ANGLE
                if vx(i,t)*rx_hat(index_b(j)) +vy(i,t)*ry_hat(index_b(j))> v_evolve*cos(theta_boid/2)
                    vx_b = vx_b + sum(rx_hat(index_b(j)));
                    vy_b = vy_b + sum(ry_hat(index_b(j)));
                end
                vx_b = -vx_b/sum(r(index_b(j)));
                vy_b = -vy_b/sum(r(index_b(j)));
            end
            
            %------ ELSE CHECK BOIDS IN ORIENTATION AND ATTRACTION ZONE %-----
        else
            
            %%%%%%%%%%Find v_o - orientation
            index_vbo = find(r_boid>=R_r & r_boid<R_o);               %Index for the boids in orientation radius
            vx_bo=0;
            vy_bo=0;
            if any(index_vbo)
                for k = 1:length(index_vbo)
                    vx_bo = -vx(index_vbo(k))/length(index_vbo);
                    vy_bo = -vy(index_vbo(k))/length(index_vbo);
                end
            end
            
            %%%%%%%%%Find v_a - attraction
            index_vba = find(r_boid>=R_o & r_boid<R_a);
            vx_ba=0;
            vy_ba=0;
            
            %CHECK IF THERE ARE ANY BOIDS IN ATTRACTION AREA
            if any(index_vba)
                %ITERATE OVER ALL BOIDS IN ATTRACTION AREA
                for k = 1:length(index_vba)
                    vx_ba = vx_ba + rx_hat(index_vba(k))/length(index_vba);
                    vy_ba = vy_ba + ry_hat(index_vba(k))/length(index_vba);
                end
            end
            %Define velocity unit vector v_b
            v_b = ((vx_ba + vx_bo).^2 + (vy_ba + vy_bo).^2).^0.5+0.00000000001;
            vx_b = (vx_ba + vx_ba)/v_b;
            vy_b = (vy_ba + vy_ba)/v_b;
            
            
        end
        
        %---------FIND INTERACTION WITH PREDATORS------------
        
        index_h = hoick_index(i,:);
        
        vx_p = 0;
        vy_p = 0;
        if r(index_h) < R_a %TEMPORARY for setting up interaction with hoicks
            vx_p = (x(i)-x(index_h))/r_hoick(i);
            vy_p = (y(i)-y(index_h))/r_hoick(i);
        end
        
        %%%%%Find v_noise
        vx_noise = randn(1,1);
        vy_noise = randn(1,1);
        
        vx_noise = vx_noise/(vx_noise^2 + vy_noise^2)^0.5;
        vy_noise = vy_noise/(vx_noise^2 + vy_noise^2)^0.5;
        
        %%%%%Add together all the components for the velocity vector
        vx(i,t+1) = vx_b + e_boid*vx_noise + vx_p;% + omega_boid*v_pf_x_boid(i,t);
        vy(i,t+1) = vy_b + e_boid*vy_noise + vy_p;% + omega_boid*v_pf_y_boid(i,t);
        
        vxy_norm = (vx(i,t+1)^2 + vy(i,t+1)^2)^.5+0.000000001;
        
        x(i,t+1) = x(i,t) + v_evolve*vx(i,t+1)/vxy_norm;
        y(i,t+1) = y(i,t) + v_evolve*vy(i,t+1)/vxy_norm;
        
        %Plot boids
        %    if abs(x_boid(i,t)-x_boid(i,t+1))<v_boid(i,t) && abs(y_boid(i,j)-y_boid(i,j+1))<v_boid(i,t)
        plot([x(i,t), x(i,t+1)] ,[y(i,t),y(i,t+1)],'k-','markersize',5) %plots the first half of the particles in black
        axis([0 L 0 L]);
        hold on
        plot(x(i,t+1) ,y(i,t+1),'k.','markersize',14)
        %       end
        %     hold on
    end
    %ITERATE BOID i OVER PREDATORS
    pause(0.00001)
    hold off
end