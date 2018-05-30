%This code is for extension of the models, implementing hoicks in the boid
%world

%LEGEND FOR STUFF
    % CHECK
    % TEMPORARY
    % DELETE
    % OPTIMIZE
    
%function [polarisation]=hoick_world(p)

%-------- CONTROL VARIABLES----------%
phase_mode = 0;
hoick_mode=0;

  
if phase_mode
    %INITIALIZE PARAMETERS
    L = p.L;
    N_boid = p.N_boid;
    N_hoick = p.N_hoick;
    
    R_r = p.R_r;
    R_o = p.R_o;
    R_a = p.R_a;
    
    A_s = p.A_s;
    A_m = p.A_m;
    
    v_evolve = p.v_evolve;
    v_hoick=p.v_hoick;    
    theta_boid = p.theta_boid;
    theta_hoick = p.theta_hoick;
    phi_boid = p.phi_boid;
    phi_hoick = p.phi_hoick;
    omega_boid = p.omega_boid;
    e_boid = p.e_boid;

    warm_up = p.warm_up;
    tot_time = p.tot_time;

    make_figure=p.make_figure;
    make_movie=p.make_movie;
    
else
    close all
    clear all

<<<<<<< HEAD
%INITIALIZE PARAMETERS
L=400;                   %System size
N_boid = 10;             %Nr of boids
N_hoick = 1;             %Nr of predators
R_r = 1;                 %repulsion radius
R_o = 15;                %Orientation radius
R_a = 30;                %Attraction radius
R_catch = R_r + 1;       % TEMPORARY value. Radius describing when predation is successful
v_evolve = 2;            % CHECK(no evolution for boids) the evolvable speed of boid
v_hoick = 3;             % TEMPORARY value. Speed of hoick
A_s = 1000*R_r^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
A_m = 25*R_r^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area

phi_boid  = A_m/(2*v_evolve^2); %turning angle for boids
phi_hoick = pi/4;               %turning angle for hoicks
theta_boid = A_s/R_a^2;         %viewing angle
theta_hoick = pi;               %viewing angle

e_boid = 0.00001;       %Sensitivity to noise
omega_boid = 1;         %Sensitivity to predator
warm_up = 10000;         %CHECK do we really need this? %Warm up time, 15 minutes in the paper
tot_time = 10000 + warm_up;       %Totalt time
=======
    %INITIALIZE PARAMETERS
    L=400;                  %System size
    N_boid = 10;            %Nr of boids
    N_hoick = 1;            %Nr of predators
    
    R_r = 1;                %repulsion radius
    R_o = 7;                %Orientation radius
    R_a = 14;               %Attraction radius

    A_s = 1000*R_r^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m = 25*R_r^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    v_evolve = 2;           % CHECK(no evolution for boids) the evolvable speed of boid
    v_hoick = 3;            % TEMPORARY value. Speed of hoick
    phi_boid  = A_m/(2*v_evolve^2); %turning angle for boids
    phi_hoick = pi/4;      %turning angle for hoicks
    theta_boid = A_s/R_a^2;      %viewing angle
    theta_hoick = pi;          %viewing angle
    omega_boid = 1;         %Sensitivity to predator
    e_boid = 0.00001;       %Sensitivity to noise

    warm_up = 0;            %Warm up time
    tot_time = 1000 + warm_up;       %Totalt time

    make_figure=1;
    make_movie=0;
end

%---Temporary variables---%
R_catch = R_r +1;       % TEMPORARY value. Radius describing when predation is successful


%DELETE
second = 0;             %measures how often we enter the second loop, i.e. turn right <- see correction of angle code
first = 0;              %measures how often we enter the second loop, i.e. turn left <- see correction of angle code
%--------------------------%
>>>>>>> 397e41b7cf74f97a8175d6dc2597d7fc385f33ab


%GRAPHICS STUFF
if make_figure
    fig=figure;
    marker1 = 14;
    
    if make_movie
        filename = 'Final Project.avi';
        video = VideoWriter(filename);
        video.FrameRate = 3;
        video.Quality = 100;
        open(video);
    end   
end

%-------------------INITIALIZE BOIDS AND HOICKS-------------------------%
% x(i,t) gives the x coordinate of the ith particle at time t where the %
% last element is the hoick and the first N_boid elements are boids     %
%-----------------------------------------------------------------------%
x = zeros(N_boid + N_hoick,tot_time+1);          %define initial x coordiantes for boids
x(:,1) = L/2+L/8*rand(N_boid + N_hoick,1)-L/16;  % TEMPORARY initial positions
x(N_boid + N_hoick, warm_up + 1) = L*rand();     %set random x-position for hoick once it's introduced to the world

y = zeros(N_boid + N_hoick,tot_time+1);          %define initial y coordinates for boids
y(:,1) = L/2+L/8*rand(N_boid + N_hoick,1)-L/16;  %TEMPORARY initial positions
y(N_boid + N_hoick, warm_up + 1) = L*rand();     %set random y-position for hoick once it's introduced to the world

v = zeros(N_boid + N_hoick,tot_time+1);   %velocity vector for all individuals
vy = zeros(N_boid + N_hoick,tot_time+1);
vx = zeros(N_boid + N_hoick,tot_time+1);

%DEFINE HELPFUL VECTORS
r = zeros(N_boid + N_hoick,1);         %r is the distance from current boid to all other boids
rx_hat = zeros(N_boid + N_hoick,1);    %unit vector for x component
ry_hat = zeros(N_boid + N_hoick,1);    %unit vector for y component

prevdirection = zeros(N_boid + N_hoick, tot_time+1);
newdirection = zeros(N_boid + N_hoick, tot_time+1);
newdirection(:,1) = 2*pi*rand(N_boid + N_hoick, 1);

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
        
        if i <= N_boid
            %----------ITERATE OVER BOIDS---------%
            
            if isnan(x(i,t)) %Skips this iteration if the value is NaN (dead Boid)
                continue
            end
            
            index_b = boid_index(i,:); %Get indicies sorted by size from boid i to other boids
            
            %-----------------FIND INTERACTION WITH OTHER BOIDS------------

            inside_R_r = sum(r_boid(:,i) < R_r); %find how many boids inside repulsion radius
            
            %---------SEE IF ANY BOIDS IN REPULSION AREA--------
            lesum = 0; %initializes lesum here just to make if-loop for interaction with predator work
            
            if not(inside_R_r==0)
                
                vx_b = 0;
                vy_b = 0;
                
                lesum = 0.000000000000000000001;
                
                for j=1:inside_R_r
                    %SEE IF WITHIN VIEWING ANGLE
                    if vx(i,t)*rx_hat(i,index_b(j)) + vy(i,t)*ry_hat(i,index_b(j)) > v_evolve*cos(theta_boid/2)
                        vx_b = vx_b + rx_hat(i,index_b(j));
                        vy_b = vy_b + ry_hat(i,index_b(j));
                        lesum = lesum + r(i,index_b(j));
                    end
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
                        %SEE IF WITHIN VIEWING ANGLE
                        if vx(i,t)*rx_hat(i,index_b(k)) + vy(i,t)*ry_hat(i,index_b(k)) > v_evolve*cos(theta_boid/2)
                            vx_bo = -vx(index_vbo(k));
                            vy_bo = -vy(index_vbo(k));
                        end
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
                        %SEE IF WITHIN VIEWING ANGLE
                        if vx(i,t)*rx_hat(i,index_b(k)) + vy(i,t)*ry_hat(i,index_b(k)) > v_evolve*cos(theta_boid/2)
                            vx_ba = vx_ba + rx_hat(i,index_vba(k));
                            vy_ba = vy_ba + ry_hat(i,index_vba(k));
                        end
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
            
            if(hoick_mode)
            %if lesum == 0.000000000000000000001 % CHECK, get wierd behaviour if this is implemented. if repulsion was determined, do not care for predator
                if r(N_boid + N_hoick,i) < R_a + 50 % TEMPORARY value. Calculating v_p
                %vx_p = -(x(N_boid + N_hoick)-x(i))/r_hoick(i);
                %vy_p = -(y(N_boid + N_hoick)-y(i))/r_hoick(i);
                vx_p = rx_hat(i,N_boid + N_hoick);
                vy_p = ry_hat(i,N_boid + N_hoick);
                end
                
                if r(N_boid + N_hoick,i) <= R_catch % TEMPORARY value. boid dies if hoick comes close
                    x(i,[t:end]) = NaN;
                    y(i,[t:end]) = NaN;
                end
            %end
            end
            
            %----------FIND NOISE----------------------------------%
            vx_noise = 2*rand-1;
            vy_noise = 2*rand-1;
            
            vx_noise = vx_noise/(vx_noise^2 + vy_noise^2)^0.5;
            vy_noise = vy_noise/(vx_noise^2 + vy_noise^2)^0.5;
            
            
            %----------ADD COMPONENTS FOR VELOCITY VECTOR----------%
            vx(i,t+1) = vx_b + e_boid*vx_noise + omega_boid*vx_p;% + omega_boid*v_pf_x_boid(i,t);
            vy(i,t+1) = vy_b + e_boid*vy_noise + omega_boid*vy_p;% + omega_boid*v_pf_y_boid(i,t);
            vxy_norm = (vx(i,t+1)^2 + vy(i,t+1)^2)^.5+0.000000001;
            
            %----------CORRECT FOR TURNING ANGLE-----------------%
            newdirection(i,t+1) = atan2(vy(i,t+1),vx(i,t+1)); %calculate "wanted" the angle of direction of the boid
            prevdirection(i,t) = newdirection(i,t);%atan2(vy(i,t),vx(i,t));
            
            delta_angle = angdiff(prevdirection(i,t),newdirection(i,t+1));
            if (abs(delta_angle)>phi_boid/2)
                if (delta_angle>0)
                    newdirection(i,t+1) = wrapTo2Pi(prevdirection(i,t) + phi_boid/2);
                    first = first +1;
                else
                    newdirection(i,t+1) = wrapTo2Pi(prevdirection(i,t) - phi_boid/2);
                    second = second + 1;
                end
            end
            
            %DELETE this just prints the sum of directions, to check for bias
            %sum(sum(wrapToPi(newdirection)));  
            
            if(t>warm_up)
            mean(mean(wrapToPi(newdirection(:,1:t+1))))
            end
            x(i,t+1) = x(i,t) + v_evolve*cos(newdirection(i,t+1));
            y(i,t+1) = y(i,t) + v_evolve*sin(newdirection(i,t+1));
            
            %---------GRAPHICS--------%
            %---------PLOT BOIDS---------------------------
            if make_movie && t>warm_up
                x(i,t+1)=mod(x(i,t+1),L); % Jumps from the right of the box to the left or vice versa
                y(i,t+1)=mod(y(i,t+1),L); % Jumps from the top of the box to the bottom or vice versa

                plot([x(i,t), x(i,t+1)] ,[y(i,t),y(i,t+1)],'k-','markersize',5) %plots the first half of the particles in black
                axis([0 L 0 L]);
                hold on
                plot(x(i,t+1) ,y(i,t+1),'k.','markersize',14)
                %title(['Timestep: ',num2str(t)])
                %xlabel('X position')
                %ylabel('Y position')
            end                   
            
            
        elseif hoick_mode && i > N_boid && t > warm_up %introduce hoick to the world after warm up is finished
            %-----------ITERATE OVER HOICKS------------%
            %FIND VELOCITY FOR HOICK
            vx(i,t+1) = rx_hat(i,hoick_index(1));
            vy(i,t+1) = ry_hat(i,hoick_index(1));
            vxy_norm = (vx(i,t+1)^2 + vy(i,t+1)^2)^.5+0.000000001;
            
            %UPDATE HOICKS POSITION
            x(i,t+1) = x(i,t) + v_hoick*vx(i,t+1)/vxy_norm;
            y(i,t+1) = y(i,t) + v_hoick*vy(i,t+1)/vxy_norm;
            
            %-----------PLOT HOICK----------------------
            if make_figure
                x(i,t+1)=mod(x(i,t+1),L); % Jumps from the right of the box to the left or vice versa
                y(i,t+1)=mod(y(i,t+1),L); % Jumps from the top of the box to the bottom or vice versa

                plot([x(i,t), x(i,t+1)] ,[y(i,t),y(i,t+1)],'r-','markersize',5) %plots the first half of the particles in black
                axis([0 L 0 L]);
                hold on
                plot(x(i,t+1) ,y(i,t+1),'r.','markersize',14)     
            end
        end
    end
    

    %----------Calculate polarisation----------%       
    vx_sum=sum(vx(1:N_boid,t));
    vy_sum=sum(vy(1:N_boid,t));

    polarisation(t) = (1/N_boid).*sqrt(vx_sum.^2 + vy_sum.^2);      %Polarisation
    
    
    
    %Making the video
    if make_figure && t>warm_up
        pause(0.00001)
        hold off

        if make_movie
        F(j) = getframe(gcf);  %Gets the current frame
        writeVideo(video,F(j)); %Puts the frame into the videomovie
        end
    end
    
    pause(0.00001)
    hold off   
end

if make_movie
    close(video);  %Closes movie
end 
    
%end