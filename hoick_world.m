%This code is for extension of the models, implementing hoicks in the boid
%world

%LEGEND FOR STUFF
% CHECK
% TEMPORARY
% DELETE
% OPTIMIZE

function [polarisation]=hoick_world(p)

close all;

%-------- CONTROL VARIABLES----------%
phase_mode = 1;
hoick_mode = 0;
hoick_type_mode=1;
make_movie = 0;
type=1;

hoick_advantage =1.25;    %Predator advantage

if(not(phase_mode))
    make_figure = 1;
else
    make_figure = 0;
end

if phase_mode
    %TAKE INPUT PARAMETERS IF IN PHASE MODE
    L = p.L;
    N_boid = p.N_boid;
    N_hoick = p.N_hoick;
    
    R_r_boid = p.R_r_boid;
    R_o_boid = p.R_o_boid;
    R_a_boid = p.R_a_boid;
    
    R_r_hoick = p.R_r_hoick;
    R_o_hoick = p.R_o_hoick;
    R_a_hoick = p.R_a_hoick;
    
    A_s_boid = p.A_s_boid;
    A_m_boid = p.A_m_boid;
    A_s_hoick = p.A_s_hoick;
    A_m_hoick = p.A_m_hoick;
    
    v_boid = p.v_boid;
    v_hoick=p.v_hoick;
    
    theta_boid = p.theta_boid;
    theta_hoick = p.theta_hoick;
    phi_boid = p.phi_boid;
    phi_hoick = p.phi_hoick;
    omega_boid = p.omega_boid;
    omega_hoick=p.omega_hoick;
    e_boid = p.e_boid;
    e_hoick = p.e_hoick;
    
    warm_up = p.warm_up;
    tot_time = p.tot_time;
    
    make_figure = p.make_figure;
    make_movie = p.make_movie;
    
else
    
    %INITIALIZE PARAMETERS IF NOT IN PHASE MODE
    L=400;                  %System size
    N_boid = 80;            %Nr of boids
    N_hoick = 1;            %Nr of predators
    
    R_r_boid = 1;                %Repulsion radius
    R_o_boid = 10;               %Orientation radius
    R_a_boid = 13;               %Attraction radius
    
    
    R_r_hoick = 1;                %repulsion radius
    R_o_hoick = 4;                %Orientation radius
    R_a_hoick = 14;               %Attraction radius
    
    
%     A_s_boid = 1000*R_r_boid^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
%     A_m_boid = 25*R_r_boid^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
%     A_s_hoick=1000*R_r_hoick^2;
%     A_m_hoick=25*R_r_hoick^2;
%          
    A_s_boid = 2*pi;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m_boid = 2*pi*(13)^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    A_s_hoick= 2*pi*(1*hoick_advantage)^2;                %Depends on how much better the predator is than the prey
    A_m_hoick= 2*pi*(13*hoick_advantage)^2;
    
    v_boid = 2.5;           % CHECK(no evolution for boids) the evolvable speed of boid
    
    v_hoick = v_boid*1.25;            % TEMPORARY value. Speed of hoick
    phi_boid  = A_s_boid/(2*(v_boid)^2);%pi;%A_m_boid/(4*v_boid^2); %turning angle for boids
    phi_hoick = A_m_hoick/(2*(v_hoick)^2);      %turning angle for hoicks
    theta_boid = A_s_boid/((R_a_boid)^2);      %viewing angle
    theta_hoick = A_s_hoick/((R_a_hoick))^2;          %viewing angle
    
    omega_boid = 5;         %Boid sensitivity to predator
    
    omega_hoick=10;          %Hoick sensitivity to prey
    omega_group=1;
    omega_independence=12;
    
    e_boid = 0.2;       %Sensitivity to noise
    e_hoick = 0.00001;
    
    warm_up = 3000;            %Warm up time
    tot_time = 300 + warm_up;       %Totalt time
    
    
end

%--------Setting type and type variables-----%
%types: 1=group,2=independant individuals, 3=Rivals
if hoick_type_mode
    type_variables = Hoick_types(type,v_hoick);
    
    R_r_hoick = type_variables.R_r_hoick;               %Repulsion radius
    R_o_hoick = type_variables.R_o_hoick;               %Orientation radius
    R_a_hoick = type_variables.R_a_hoick;               %Attraction radius
    
    A_s_hoick = type_variables.A_s_hoick;               % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m_hoick = type_variables.A_m_hoick;               % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    phi_hoick = type_variables.phi_hoick;               %turning angle for hoicks
    theta_hoick = type_variables.theta_hoick;           %viewing angle
    
    %NEWLY ADDED BY MIKE
    %omega_independence=type_variables.omega_independence;
    R_avoid=type_variables.R_avoid;
end
%----------------------------%



% %-----_DELETE_---------%
% R_r_hoick = 5;               %Repulsion radius
% R_o_hoick = 40;               %Orientation radius
% R_a_hoick = 100;               %Attraction radius
%
% A_s_hoick = type_variables.A_s_hoick;               % TEMPORARY value (same value as used for fig 1). Possible sighting area
% A_m_hoick = type_variables.A_m_hoick;               % TEMPORARY value (same value as used for fig 1). Possible movement area
%
phi_hoick = pi/2;               %turning angle for hoicks
theta_hoick = pi/2;           %viewing angle
%
% %-----------------------_%


%------ NEW VARIABLES -------%
R_catch = 3;       %TEMPORARY value. Radius describing when predation is successful
R_flee = 25;           %TEMPORARY Radius for boids fleeing hoicks
predator_delay_time = 10;          %delay for when predator will arrive.
hoick_kills = zeros(N_hoick,tot_time-warm_up);   %Measures how many kills the hoicks make

%DELETE
second = 0;             %measures how often we enter the second loop, i.e. turn right <- see correction of angle code
first = 0;              %measures how often we enter the second loop, i.e. turn left <- see correction of angle code
%--------------------------%




%GRAPHICS STUFF
if make_figure
    fig = figure;
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
x(:,1) = L/2+L/6*rand(N_boid + N_hoick,1)-L/12; %L*rand(N_boid + N_hoick,1); %TEMPORARY initial positions
x(N_boid+1:end, warm_up+1+predator_delay_time) = L/4 + L/8*rand(N_hoick,1)-L/16;     %set random x-position for hoicks once it's introduced to the world

y = zeros(N_boid + N_hoick,tot_time+1);          %define initial y coordinates for boids
y(:,1) = L/2+L/6*rand(N_boid + N_hoick,1)-L/12; %L*rand(N_boid + N_hoick,1);   %TEMPORARY initial positions
y(N_boid+1:end, warm_up+1+predator_delay_time) = L/4 + L/8*rand(N_hoick,1)-L/16;     %set random y-position for hoicks once it's introduced to the world


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
newdirection(N_boid+1:end,warm_up+1+predator_delay_time) = 2*pi*rand(N_hoick, 1);
%ITERATE OVER TIME
for t = 1:tot_time
    
    rx_temp = repmat(x(:,t)',numel(x(:,t)),1); %create matrix of all individuals positions in x
    ry_temp = repmat(y(:,t)',numel(y(:,t)),1); %create matrix of all individuals positions in y
    
    rx_hat = (rx_temp-x(:,t));               %find distance vector between elements x-components
    ry_hat = (ry_temp-y(:,t));               %find distance vector between elements y-components
    
    diagonal_temp = ones(1,N_boid + N_hoick)*inf;
    r = (rx_hat.^2+ry_hat.^2).^0.5 + diag(diagonal_temp);         %find euclidian distance to others and add term to avoid division by zero.
    rx_hat = rx_hat./r;                     %normalize to create unit direction vector
    ry_hat = ry_hat./r;                     %normalize to create unit direction vector
    
    r_boid = r([1:N_boid],[1:N_boid]);        %Vector defining distances between boids, deleting distance to hoick
    [r_sort_boid,boid_index] = sort(r_boid'); %Sort the vector for finding closest boids
    boid_index = boid_index';                 %Turn index right so every row represents a boid
    
    r_hoick = r([N_boid+1:end],[N_boid+1:end]);       %Vector defining distances between hoicks
    [r_sort_hoick,hoick_index] = sort(r_hoick');    %
    hoick_index = hoick_index';                     %Every row represents a boid or hoick
    
    
    
    
    %----------ITERATE OVER POPULATION-----
    for i=1:N_boid + N_hoick
        
        %---------------BOIDS-----------------------%
        %-------------------------------------------%
        %-------------------------------------------%
        if i <= N_boid
            
            if isnan(x(i,t)) %Skips this iteration if the value is NaN (dead Boid)
                continue
            end
            
            index_b = boid_index(i,:); %Get indicies sorted by size from boid i to other boids
            
            
            
            %--------------------------------------------------------------%
            %-----------------FIND INTERACTION WITH OTHER BOIDS------------
            %--------------------------------------------------------------%
            
            
            inside_R_r = sum(r_boid(:,i) < R_r_boid); %find how many boids inside repulsion radius
            
            %---------SEE IF ANY BOIDS IN REPULSION AREA--------
            v_b_sum = 0; %initializes v_b_sum here just to make if-loop for interaction with predator work
            
            if not(inside_R_r==0)
                vx_b = 0;
                vy_b = 0;
                
                for j=1:inside_R_r
                    %SEE IF WITHIN VIEWING ANGLE
                    if vx(i,t)*rx_hat(i,index_b(j)) + vy(i,t)*ry_hat(i,index_b(j)) < v_boid*cos(theta_boid/2)
                        
                        vx_b = vx_b + rx_hat(i,index_b(j));
                        vy_b = vy_b + ry_hat(i,index_b(j));
                    end
                end
                vr_norm = (vx_b^2+vy_b^2)^0.5+0.0000000001;
                vx_b = -vx_b/(vr_norm);
                vy_b = -vy_b/(vr_norm);
                
                
                %------ ELSE CHECK BOIDS IN ORIENTATION AND ATTRACTION ZONE %-----
            else
                
                %Find v_o - orientation
                index_vbo = find(r_boid(i,:) >= R_r_boid & r_boid(i,:) < R_o_boid);               %Index for the boids in orientation radius
                vx_bo = 0;
                vy_bo = 0;
                if not(isempty(index_vbo))
                    for k = 1:length(index_vbo)
                        %SEE IF WITHIN VIEWING ANGLE
                        if vx(i,t)*rx_hat(i,index_b(k)) + vy(i,t)*ry_hat(i,index_b(k)) < v_boid*cos(theta_boid/2)
                            vx_bo = vx_bo-vx(index_vbo(k));
                            vy_bo = vx_bo-vy(index_vbo(k));
                        end
                    end
                end
                
                vo_norm = (vx_bo^2+vy_bo^2)^0.5+0.0000000001;
                vx_bo = vx_bo/(vo_norm);
                vy_bo = vy_bo/(vo_norm);
                
                %Find v_a - attraction
                index_vba = find(r_boid(i,:) >= R_o_boid & r_boid(i,:) < R_a_boid);
                
                vx_ba = 0;
                vy_ba = 0;
                
                %CHECK IF THERE ARE ANY BOIDS IN ATTRACTION AREA
                
                if not(isempty(index_vba))
                    %ITERATE OVER ALL BOIDS IN ATTRACTION AREA
                    for k = 1:length(index_vba)
                        %SEE IF WITHIN VIEWING ANGLE
                        if vx(i,t)*rx_hat(i,index_b(k)) + vy(i,t)*ry_hat(i,index_b(k)) < v_boid*cos(theta_boid/2)
                            
                            vx_ba = vx_ba + rx_hat(i,index_vba(k));
                            vy_ba = vy_ba + ry_hat(i,index_vba(k));
                        end
                    end
                    
                    va_norm = (vx_ba^2+vy_ba^2)^0.5+0.0000000001;
                    vx_ba = vx_ba/(va_norm);
                    vy_ba = vy_ba/(va_norm);
                    
                end
                %----DEFINE VELOCITY UNIT VECTOR v_b----
                v_b = ((vx_ba + vx_bo).^2 + (vy_ba + vy_bo).^2).^0.5+0.00000000001;
                vx_b = (vx_ba + vx_bo)/v_b;
                vy_b = (vy_ba + vy_bo)/v_b;
                
                
            end
            
            %---------FIND BOIDS INTERACTIONS WITH PREDATORS--------------%
            %-------------------------------------------------------------%
            vx_p = 0;
            vy_p = 0;
            
            if(hoick_mode)
                for j = 1:N_hoick
                    if r(N_boid + j,i) < R_flee
                        vx_p = vx_p -rx_hat(i,N_boid + j);
                        vy_p = vx_p -ry_hat(i,N_boid + j);
                    end
                end
            end
            
            %------- NORMALISE -------%
            
            predator_norm = (vx_p^2+vy_p^2)^0.5+0.0000000000000001;
            vx_p = vx_p/predator_norm;
            vy_p = vy_p/predator_norm;
            
            
            %----------FIND NOISE----------------------------------%
            vx_noise = 2*randn;
            vy_noise = 2*randn;
            
            %-------NORMALISE---------%
            noise_norm = (vx_noise^2 + vy_noise^2)^0.5+0.00000000000001;
            vx_noise = vx_noise/noise_norm;
            vy_noise = vy_noise/noise_norm;
            
            
            %----------ADD COMPONENTS FOR VELOCITY VECTOR----------%
            vx(i,t+1) = vx_b + e_boid*vx_noise + omega_boid*vx_p;
            vy(i,t+1) = vy_b + e_boid*vy_noise + omega_boid*vy_p;
            vxy_norm = (vx(i,t+1)^2 + vy(i,t+1)^2)^.5+0.000000001;
            
            
            %----------CORRECT FOR TURNING ANGLE-----------------%
            newdirection(i,t+1) = atan2(vy(i,t+1),vx(i,t+1)); %calculate "wanted" the angle of direction of the boid
            prevdirection(i,t) = newdirection(i,t);
            
            delta_angle = angdiff(prevdirection(i,t),newdirection(i,t+1));
            if (abs(delta_angle) > phi_boid/2)
                if (delta_angle > 0)
                    newdirection(i,t+1) = wrapTo2Pi(prevdirection(i,t) + phi_boid/2);
                    first = first +1;
                else
                    newdirection(i,t+1) = wrapTo2Pi(prevdirection(i,t) - phi_boid/2);
                    second = second + 1;
                end
            end
            
            
            x(i,t+1) = x(i,t) + v_boid*cos(newdirection(i,t+1));
            y(i,t+1) = y(i,t) + v_boid*sin(newdirection(i,t+1));
            
            x(i,t+1)=mod(x(i,t+1),L); % Jumps from the right of the box to the left or vice versa
            y(i,t+1)=mod(y(i,t+1),L); % Jumps from the top of the box to the bottom or vice versa
            
            
            %---------GRAPHICS--------%
            %---------PLOT BOIDS---------------------------
            if make_figure && t>warm_up
                if ((x(i,t+1)-x(i,t))^2+(y(i,t+1)-y(i,t))^2<=2*v_boid^2)
                    plot([x(i,t), x(i,t+1)] ,[y(i,t),y(i,t+1)],'k-','markersize',5) %plots the first half of the particles in black
                    axis([0 L 0 L]);
                    hold on
                    plot(x(i,t+1) ,y(i,t+1),'k.','markersize',8)
                    %title(['Timestep: ',num2str(t)])
                    %xlabel('X position')
                    %ylabel('Y position')
                end
            end
            
            
            
            
            
            
            
            
            
            %*********************************************%
            
            %---------------------------------------------%
            %-------------------HOICK---------------------%
            %---------------------------------------------%
            
        elseif hoick_mode && i > N_boid && t > warm_up+predator_delay_time %introduce hoick to the world after warm up is finished
            %-----------ITERATE OVER HOICKS------------%
            if isnan(x(i,t)) %Skips this iteration if the value is NaN (dead Hoick)
                continue
            end
            
            %----------------FIND INTERACTION WITH BOIDS---------%
            %----------------------------------------------------%
            
            %----------------TEST IF HOICK CAN EAT THE CLOSEST BOID ------%
            [prey_distance, prey_index] = sort(r(i,1:N_boid));
            
            vx_p = 0;
            vy_p = 0;
            if not(isempty(prey_distance))
                if prey_distance(1) <= R_catch % Boid is killed if within in R_catch
                    x(prey_index(1),t:end) = NaN;
                    y(prey_index(1),t:end) = NaN;
                    [prey_distance, prey_index] = sort(r(i,1:N_boid));
                    hoick_kills(i-N_boid,t-warm_up) = hoick_kills(i-N_boid,t-warm_up) +1;
                end
                
                %-----------------FIND VELOCITY VECTOR FOR CLOSEST BOID------------%
                vx_p = rx_hat(i,prey_index(1));
                vy_p = ry_hat(i,prey_index(1));
                %Normalise to unit vectors
                vxy_norm = (vx_p^2 + vy_p^2)^.5+0.000000001;
                vx_p = vx_p/vxy_norm;
                vy_p = vy_p/vxy_norm;
                
            end
            
            %-----------------FIND INTERACTION WITH OTHER HOICKS------------
            %---------------------------------------------------------------%
            
            
            vx_br = 0;
            vy_br = 0;
            vx_b = 0;
            vy_b = 0;
            vx_bav = 0;
            vy_bav = 0;
            index_b = hoick_index(i-N_boid,:); %Get indicies sorted by size from hoick to other hoicks
            
            if(N_hoick>1)
                
                
                %---------CHECK IF ANY HOICKS ARE IN REPULSION ZONE--------%
                [hoick_distance, le_hoick_index] = sort(r(i,N_boid+1:end));
                inside_R_r_hoick=sum(hoick_distance<R_r_hoick);%Find how many hoicks are inside repulsion radius
                    
                    
                    
                %inside_R_r_hoick = sum(r_hoick(:,i-N_boid) < R_r_hoick);   %Find how many hoicks are inside repulsion radius
                
                %IF ANY HOICKS ARE TOO CLOSE
                if (not(inside_R_r_hoick==0))
                    
                    %for j=1:inside_R_r_hoick
                    j=1;
                    while(hoick_distance(j)<=R_r_hoick&&j<=N_hoick)
                        
                        %SEE IF WITHIN VIEWING ANGLE
                        %if vx(i,t)*rx_hat(i,index_b(j)+N_boid) + vy(i,t)*ry_hat(i,index_b(j)+N_boid) > v_hoick*cos(theta_hoick/2)
                        vx_br = vx_br + rx_hat(i,N_boid+le_hoick_index(j));
                        vy_br = vy_br + ry_hat(i,N_boid+le_hoick_index(j));
                        %end
                        j=j+1;
                    end
                    vbr_sum = (vx_br^2+vy_br^2)^0.5+0.000001;
                    vx_br = -vx_br/vbr_sum;
                    vy_br = -vy_br/vbr_sum;
                    
                    
                else
                    %------ ELSE CHECK HOICKS IN AVOIDANCE, ORIENTATION AND ATTRACTION ZONE %-----
                    
                    
                    %------------AVOIDANCE---------%
                    [hoick_distance, le_hoick_index] = sort(r(i,N_boid+1:end));
                    inside_R_r_hoick=sum(hoick_distance<R_avoid);%Find how many hoicks are inside avoidance radius

                    
                    %inside_R_r_hoick = sum(r_hoick(:,i-N_boid) < R_avoid);                      
                    inside_R_r_hoick_view=0;
                    
                    %Find how many hoicks inside avoidance radius current
                    %hoick can see
                    %for j=1:inside_R_r_hoick
                    j=1;
                    hoick_view_index=[];
                    while(hoick_distance(j)<=R_avoid&&j<=N_hoick)
                                %SEE IF WITHIN VIEWING ANGLE
                                if vx(i,t)*rx_hat(i,N_boid+le_hoick_index(j)) + vy(i,t)*ry_hat(i,N_boid+le_hoick_index(j)) > v_hoick*cos(theta_hoick)
                                    inside_R_r_hoick_view=inside_R_r_hoick_view+1;
                                    hoick_view_index=[hoick_view_index,N_boid+le_hoick_index(j)];
                                end
                                j=j+1;
                    end
                    
                    if (not(inside_R_r_hoick_view==0)&&prey_distance(1) > 3*R_catch)
                        for j = 1:inside_R_r_hoick_view
                            vx_bav = vx_bav + rx_hat(i,hoick_view_index(j));
                            vy_bav = vy_bav + ry_hat(i,hoick_view_index(j));
                        end
                        vbav_sum = (vx_bav^2+vy_bav^2)^0.5+0.000001;
                        vx_bav = -vx_bav/vbav_sum;
                        vy_bav = -vy_bav/vbav_sum;
                    else
                        %------------ORIENTATION---------%
                        index_vbo = find(r_hoick(:,i-N_boid) >= R_r_hoick & r_hoick(:,i-N_boid) < R_o_hoick);               %Index for the hoicks in orientation radius
                        vx_bo = 0;
                        vy_bo = 0;
                        if not(isempty(index_vbo))
                            for k = 1:length(index_vbo)
                                %SEE IF WITHIN VIEWING ANGLE
                                if vx(i,t)*rx_hat(i,index_vbo(k)+N_boid) + vy(i,t)*ry_hat(i,index_vbo(k)+N_boid) > v_hoick*cos(theta_hoick/2)
                                    vx_bo = vx_bo -vx(index_vbo(k)+N_boid);
                                    vy_bo = vx_bo -vy(index_vbo(k)+N_boid);
                                end
                            end
                        end
                        
                        vbo_norm = (vx_bo^2+vy_bo^2)^0.5+0.0000000001;
                        vx_bo=vx_bo/vbo_norm;
                        vy_bo=vy_bo/vbo_norm;
                        
                        %--------------ATTRACTION------------------%
                        index_vba = find(r_hoick(i-N_boid,:) >= R_o_hoick & r_hoick(i-N_boid,:) < R_a_hoick);
                        vx_ba = 0;
                        vy_ba = 0;
                        
                        %CHECK IF THERE ARE ANY HOICKS IN ATTRACTION AREA
                        
                        if not(isempty(index_vba))
                            %ITERATE OVER ALL BOIDS IN ATTRACTION AREA
                            for k = 1:length(index_vba)
                                %SEE IF WITHIN VIEWING ANGLE
                                if vx(i,t)*rx_hat(i,index_b(k)+N_boid) + vy(i,t)*ry_hat(i,index_b(k)+N_boid) > v_hoick*cos(theta_hoick/2)
                                    vx_ba = vx_ba + rx_hat(i,index_vba(k)+N_boid);
                                    vy_ba = vy_ba + ry_hat(i,index_vba(k)+N_boid);
                                end
                            end
                            
                            vba_norm = (vx_ba^2+vy_ba^2)^0.5+0.0000000001;
                            vx_ba=vx_ba/vba_norm;
                            vy_ba=vy_ba/vba_norm;
                        end
                    end
                    
                end
                
                %----DEFINE VELOCITY UNIT VECTOR v_b----
                v_b = ((vx_ba + vx_bo).^2 + (vy_ba + vy_bo).^2).^0.5+0.00000000001;
                vx_b = (vx_ba + vx_bo)/v_b;
                vy_b = (vy_ba + vy_bo)/v_b;
            end
            
            
            %----------FIND NOISE----------------------------------%
            vx_noise = 2*rand-1;
            vy_noise = 2*rand-1;
            
            vx_noise = vx_noise/(vx_noise^2 + vy_noise^2)^0.5;
            vy_noise = vy_noise/(vx_noise^2 + vy_noise^2)^0.5;
            
            
            %----------ADD COMPONENTS FOR VELOCITY VECTOR----------%
            vx(i,t+1) = omega_group*(vx_b + vx_br) + omega_independence*vx_bav + e_hoick*vx_noise + omega_hoick*vx_p;
            vy(i,t+1) = omega_group*(vy_b + vy_br) + omega_independence*vy_bav + e_hoick*vy_noise + omega_hoick*vy_p;
            
            
            %DELETE
            gruppx= omega_group*(vx_b + vx_br);
            gruppy= omega_group*(vy_b + vy_br);
            
            
            %----------CORRECT FOR TURNING ANGLE-----------------%
            newdirection(i,t+1) = atan2(vy(i,t+1),vx(i,t+1)); %calculate "wanted" the angle of direction of the hoick
            prevdirection(i,t) = newdirection(i,t);%atan2(vy(i,t),vx(i,t));
            
            delta_angle = angdiff(prevdirection(i,t),newdirection(i,t+1));
            if (abs(delta_angle)>phi_hoick/2)
                if (delta_angle>0)
                    newdirection(i,t+1) = wrapTo2Pi(prevdirection(i,t) + phi_hoick/2);
                    first = first +1;
                else
                    newdirection(i,t+1) = wrapTo2Pi(prevdirection(i,t) - phi_hoick/2);
                    second = second + 1;
                end
            end
            
            
            %UPDATE HOICKS POSITION
            x(i,t+1) = x(i,t) + v_hoick*cos(newdirection(i,t+1));
            y(i,t+1) = y(i,t) + v_hoick*sin(newdirection(i,t+1));
            
            x(i,t+1)=mod(x(i,t+1),L); % Jumps from the right of the box to the left or vice versa
            y(i,t+1)=mod(y(i,t+1),L); % Jumps from the top of the box to the bottom or vice versa
            
            %---------GRAPHICS--------%
            %-----------PLOT HOICK----------------------
            if make_figure
                if ((x(i,t+1)-x(i,t))^2+(y(i,t+1)-y(i,t))^2<=2*v_hoick^2)
                    plot([x(i,t), x(i,t+1)] ,[y(i,t),y(i,t+1)],'r-','markersize',5) %plots the first half of the particles in black
                    axis([0 L 0 L]);
                    hold on
                    plot(x(i,t+1) ,y(i,t+1),'r.','markersize',14)
                end
            end
        end
    end
    
    
    %----------Calculate polarisation----------%
    
    vx_sum=sum(cos(newdirection(1:N_boid,t+1)));
    vy_sum=sum(sin(newdirection(1:N_boid,t+1)));
    
    polarisation(t) = (1/N_boid).*sqrt(vx_sum.^2 + vy_sum.^2);      %Polarisation
    if(t > warm_up)
        polarisation(t);
    end
    
    %Making the video
    if make_figure && t > warm_up
        %pause(0.00001)
        hold off
        
        if make_movie
            F(j) = getframe(gcf);  %Gets the current frame
            writeVideo(video,F(j)); %Puts the frame into the videomovie
            pause(0.0000001)
            hold off
        end
    end 

end

if make_movie
    close(video);  %Closes movie
end

end