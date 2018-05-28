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
N_hoick = 1;            %Nr of predators
R_r = 1;                %repulsion radius
R_o = 2;                %Orientation radius
R_a = 14;               %Attraction radius
v_evolve = 2;           % CHECK(no evolution for boids) the evolvable speed of boid
v_hoick = 8;            % TEMPORARY value. Speed of hoick

theta_boid  = pi/4;      %turning angle for boids
theta_hoick = pi/4;      %turning angle for hoicks
phi_boid = pi;           %viewing angle
phi_hoick = pi;          %viewing angle

A_s = 0;                 % CHECK do we use this? Possible sighting area
A_m = 0;                 % CHECK do we use this? Possible movement area

e_boid = 0.0;           %Sensitivity to noise
omega_boid = 0;         %Sensitivity to predator
warm_up = 10000;        %CHECK do we really need this? %Warm up time, 15 minutes in the paper
tot_time = 1000;       %Totalt time


%DEFINE HELPFUL VECTORS
r = zeros(N_boid + N_hoick,1);         %r is the distance from current boid to all other boids
rx_hat = zeros(N_boid + N_hoick,1);    %unit vector for x component
ry_hat = zeros(N_boid + N_hoick,1);    %unit vector for y component


%GRAPHICS STUFF
fig = figure;
marker1 = 14;

%-------------------INITIALIZE BOIDS AND HOICKS-------------------------%
% x(i,t) gives the x coordinate of the ith particle at time t where the %
% last element is the hoick and the first N_boid elements are boids     %
%-----------------------------------------------------------------------%
x = zeros(N_boid + N_hoick,tot_time+1);    %define initial x coordiantes for boids
x(:,1) = L/2*rand(N_boid + N_hoick,1);       %initial positions

y = zeros(N_boid + N_hoick,tot_time+1);    %define initial y coordinates for boids
y(:,1) = L/2*rand(N_boid + N_hoick,1);       %initial positions

v = zeros(N_boid + N_hoick,tot_time+1);   %velocity vector for all individuals
vy = zeros(N_boid + N_hoick,tot_time+1);
vx = zeros(N_boid + N_hoick,tot_time+1);


%ITERATE OVER TIME
for t = 1:tot_time
    
    rx_temp = repmat(x(:,t)',numel(x(:,t)),1); %create matrix of all individuals positions in x
    ry_temp = repmat(y(:,t)',numel(y(:,t)),1); %create matrix of all individuals positions in y
    
    rx_hat = (rx_temp-x(:,t));               %find distance vector between elements x-components
    ry_hat = (ry_temp-y(:,t));               %find distance vector between elements y-components
    
    diagonal_temp = ones(1,N_boid + N_hoick)*inf;
    r = (rx_hat.^2+ry_hat.^2).^0.5 + diag(diagonal_temp);         %find euclidian distance and add term to avoid division by zero.
    rx_hat = rx_hat./r;                     %normalize to create unit direction vector
    ry_hat = ry_hat./r;                     %normalize to create unit direction vector
    
    r_boid = r([1:N_boid],[1:N_boid]);        %Vector defining distances between boids, deleting distance to hoick
    [r_sort_boid,boid_index] = sort(r_boid'); %Sort the vector for finding closest boids
    boid_index = boid_index';                 %Turn index right so every row represents a boid
    
    r_hoick = r([N_boid + N_hoick:end],:);      %Vector defining distances between boids and hoick
    [r_sort_hoick,hoick_index] = sort(r_hoick');
    hoick_index = hoick_index';
    
    
    %----------ITERATE OVER POPULATION-----
    for i=1:N_boid + N_hoick
        
        if i <= N_boid %if individual i is a boid
            
            if isnan(x(i,t)) %Skips this iteration if the value is NaN (dead Boid)
                continue
            end
            
            index_b = boid_index(i,:); %Get indicies sorted by size from boid i to other boids
            
            %-----------------FIND INTERACTION WITH OTHER BOIDS------------
            inside_R_r = sum(r_boid(:,i) < R_r); %find how many boids inside repulsion radius
            
            %------ SEE IF ANY BOIDS IN REPULSION AREA--------
            
            if not(inside_R_r==0)
                vx_b = 0;
                vy_b = 0;
                
                lesum = 0;
                
                for j=1:inside_R_r
                    
                    %SEE IF WITHIN VIEWING ANGLE TEMPORARY deleted this for
                    %now
                    %if vx(i,t)*rx_hat(index_b(j)) +vy(i,t)*ry_hat(index_b(j))> v_evolve*cos(theta_boid/2)
                    vx_b = vx_b + rx_hat(index_b(j),i);
                    vy_b = vy_b + ry_hat(index_b(j),i);
                    lesum = lesum + r(index_b(j),i);
                    %end
                end
                vx_b = -vx_b/lesum;
                vy_b = -vy_b/lesum;
                %------ ELSE CHECK BOIDS IN ORIENTATION AND ATTRACTION ZONE %-----
            else
                
                %Find v_o - orientation
                index_vbo = find(r_boid(i,:) >= R_r & r_boid(i,:) < R_o);               %Index for the boids in orientation radius
                vx_bo = 0;
                vy_bo = 0;
                if not(isempty(index_vbo))
                    for k = 1:length(index_vbo)
                        vx_bo = -vx(index_vbo(k));
                        vy_bo = -vy(index_vbo(k));
                    end
                    
                end
                
                %Find v_a - attraction
                index_vba = find(r_boid(i,:) >= R_o & r_boid(i,:) < R_a);
                vx_ba = 0;
                vy_ba = 0;
                
                %CHECK IF THERE ARE ANY BOIDS IN ATTRACTION AREA
                if not(isempty(index_vba))
                    %ITERATE OVER ALL BOIDS IN ATTRACTION AREA
                    for k = 1:length(index_vba)
                        vx_ba = vx_ba + rx_hat(index_vba(k),i);
                        vy_ba = vy_ba + ry_hat(index_vba(k),i);
                    end
                end
                %----DEFINE VELOCITY UNIT VECTOR v_b----
                v_b = ((vx_ba + vx_bo).^2 + (vy_ba + vy_bo).^2).^0.5+0.00000000001;
                vx_b = (vx_ba + vx_bo)/v_b;
                vy_b = (vy_ba + vy_bo)/v_b;
                
                
            end
            
            %---------FIND A BOIDS INTERACTION WITH PREDATORS--------------%
            
            vx_p = 0;
            vy_p = 0;
            
            if r(N_boid + N_hoick,i) < R_a % TEMPORARY value. Calculating v_p
                vx_p = -(x(N_boid + N_hoick)-x(i))/r_hoick(i);
                vy_p = -(y(N_boid + N_hoick)-y(i))/r_hoick(i);
            end
            
            % TEMPORARY Deleted this for now just to make the movement of the boids
            %work
            %             if r(N_boid + N_hoick,i) <= R_o % TEMPORARY value. boid dies if hoick comes close
            %                 x(i,:) = NaN;
            %                 y(i,:) = NaN;
            %             end
            
            %----------FIND NOISE----------------------------------%
            vx_noise = randn(1,1);
            vy_noise = randn(1,1);
            
            vx_noise = vx_noise/(vx_noise^2 + vy_noise^2)^0.5;
            vy_noise = vy_noise/(vx_noise^2 + vy_noise^2)^0.5;
            
            %----------ADD COMPONENTS FOR VELOCITY VECTOR----------%
            vx(i,t+1) = vx_b + e_boid*vx_noise; %+ vx_p;% + omega_boid*v_pf_x_boid(i,t);
            vy(i,t+1) = vy_b + e_boid*vy_noise; %+ vy_p;% + omega_boid*v_pf_y_boid(i,t);
            
            vxy_norm = (vx(i,t+1)^2 + vy(i,t+1)^2)^.5+0.000000001;
            
            x(i,t+1) = x(i,t) + v_evolve*vx(i,t+1)/vxy_norm;
            y(i,t+1) = y(i,t) + v_evolve*vy(i,t+1)/vxy_norm;
            
            %---------PLOT BOIDS---------------------------
            x(i,t+1)=mod(x(i,t+1),L); % Jumps from the right of the box to the left or vice versa
            y(i,t+1)=mod(y(i,t+1),L); % Jumps from the top of the box to the bottom or vice versa
            
            plot([x(i,t), x(i,t+1)] ,[y(i,t),y(i,t+1)],'k-','markersize',5) %plots the first half of the particles in black
            axis([0 L 0 L]);
            hold on
            plot(x(i,t+1) ,y(i,t+1),'k.','markersize',14)
            
        else %predator
            %FIND VELOCITY FOR HOICK
            vx(i,t+1) = rx_hat(i,hoick_index(1));
            vy(i,t+1) = ry_hat(i,hoick_index(1));
            vxy_norm = (vx(i,t+1)^2 + vy(i,t+1)^2)^.5+0.000000001;
            
            %UPDATE HOICKS POSITION
            x(i,t+1) = x(i,t) + v_hoick*vx(i,t+1)/vxy_norm;
            y(i,t+1) = y(i,t) + v_hoick*vy(i,t+1)/vxy_norm;
            
            
            %-----------PLOT HOICK----------------------
            x(i,t+1)=mod(x(i,t+1),L); % Jumps from the right of the box to the left or vice versa
            y(i,t+1)=mod(y(i,t+1),L); % Jumps from the top of the box to the bottom or vice versa
            
            plot([x(i,t), x(i,t+1)] ,[y(i,t),y(i,t+1)],'r-','markersize',5) %plots the first half of the particles in black
            axis([0 L 0 L]);
            hold on
            plot(x(i,t+1) ,y(i,t+1),'r.','markersize',14)
        end
    end
    
    pause(0.00001)
    hold off
end