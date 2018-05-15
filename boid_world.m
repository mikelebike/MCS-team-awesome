%This code is just for the boids, #boidswillbeboids. This is for
%simplifying the process of building the code corpus, one step at a time.


%INITIALIZE PARAMETERS
L=400;                  %system size
N_boid = 80;            %Nr of boids
dt = 0.1;               % CHECK Time-step, why is this used?
R_r = 1;                %repulsion radius
R_o = 10;               %Orientation radius
R_a =  20;              %attraction radius
v_boid = 0;             %CHECK if this is necessary. v_void is speed of boids

theta_boid  = pi;       %turning angle for boids
theta_hoick = pi;       %turning angle for hoicks
phi_boid = 0;
phi_hoick = 0;

A_s =0;                 %Possible sighting area
A_m =0;                 %Possible movement area


e_boid = 0;             %noise
warm_up = 10000;        % CHECK do we really need this? %Warm up time, 15 minutes in the paper
tot_time=100;           %Totalt time


%DEFINE HELPFUL VECTORS
r = zeros(N_boid,1);       %r is the distance from current boid to all other boids
rx_hat = zeros(N_boid,1);  %unit vector for x component
ry_hat = zeros(N_boid,1);    %unit vector for y component


%GRAPHICS STUFF
fig=figure;



%INITIALIZE BOIDS
% x(i,j) gives the x coordinate of the ith particle at time j
x_boid=zeros(N_boid,tot_time+1);    %define initial x coordiantes for boids
x_boid(:,1)=L*rand(N_boid,1);       %initial positions

y_boid=zeros(N_boid,tot_time+1);    %define initial y coordinates for boids
y_boid(:,1)=L*rand(N_boid,1);       %initial positions

v_boid = zeros(N_boid,tot_time+1);   %velocity vector for all boids
vy_boid = zeros(N_boid,tot_time+1);
vx_boid = zeros(N_boid,tot_time+1);


%ITERATE OVER TIME
for t = 1:tot_time
    
    
    
    %ITERATE OVER BOIDS
    for i=1:N_boid
        
        if isnan(x_boid(i,t+1))    %Skips this iteration if the value is NaN (dead Boid)
            continue
        end
        
        
        
        
        %Distance to all other particles
        r(:,1)=((x_boid(i,t)-x_boid(:,t)).^2 + (y_boid(i,t)-y_boid(:,t)).^2).^0.5;
        rx_hat(:,1) = (x_boid(i,t)-x_boid(:,t))./(r(:,1)+0.0000000001);                    %Unit direction
        ry_hat(:,1)= (y_boid(i,t)-y_boid(:,t))./(r(:,1)+0.0000000001);                     %Unit direction
        
        [r_sort,index] = sort(r,1);               %Sorting the direction vector r_ij
        r_sort=r_sort(2:end);
        index=index(2:end);
        
        %Looking at all the boids inside the repulsion radius
        inside_R_r = r(r<R_r);
        
        if any(inside_R_r)      %If there are any boids inside the repulsion radius
            vx_b = 0;
            vy_b = 0;
            
            for j=1:length(inside_R_r)
                
                if vx_boid(i)*rx_hat(index(j)) +vy_boid(i)*ry_hat(index(j))> v_boid(i)*cos(theta(i)/2)
                    vx_b = vx_b + sum(rx_hat(index(j)));
                    vy_b = vy_b + sum(ry_hat(index(j)));
                end
                
                vx_b = -vx_b/sum(r(index(j)));
                vy_b = -vy_b/sum(r(index(j)));
            end
            
        else  %No boids in the repulsion area
            
            %%%%%%%%%%Find v_o
            index_vbo = find(r>R_r & r<R_o);               %Index for the boids in orientation radius
            vx_bo=0;
            vy_bo=0;
            if any(index_vbo)
                vx_bo= -vx_boid(index_vbo)./sum(vx_boid(index_vbo));
                vy_bo = -vy_boid(index_vbo)./sum(vy_boid(index_vbo));
            end
            
            %%%%%%%%%Find v_a
            index_vba = find(r_ij>R_o & r_ij<R_a);
            vx_ba=0;
            vy_ba=0;
            
            if any(index_vba)
                vx_ba = rx_hat(index_vba)./sum(r);
                vy_ba = ry_hat(index_vba)./sum(r);
                
            end
            
            %Define velocity unit vector v_b
            v_b = ((vx_ba + vx_bo).^2 + (vy_ba + vy_bo).^2).^0.5;
            vx_b = (vx_ba + vx_bo)/v_b;
            vy_b = (vy_ba + vy_ba)/v_b;
            
            
        end
        
        %%%%%Find v_noise
        vx_noise = randn(1,1);
        vy_noise = randn(1,1);
        
        vx_noise = vx_noise/(vx_noise^2 + vy_noise^2)^0.5;
        vy_noise = vy_noise/(vx_noise^2 + vy_noise^2)^0.5;
        
        %%%%%Add together all the components for the velocity vector
        vx_boid(i,t+1) = vx_b + e_boid*vx_noise;% + omega_boid*v_pf_x_boid(i,t);
        vy_boid(i,t+1) = vy_b + e_boid*vy_noise;% + omega_boid*v_pf_y_boid(i,t);
        
        
        disp("min mamma")
        %Plot boids
    %    if abs(x_boid(i,t)-x_boid(i,t+1))<v_boid(i,t) && abs(y_boid(i,j)-y_boid(i,j+1))<v_boid(i,t)
            plot([x_boid(i,t), x_boid(i,t+1)] ,[y_boid(i,t),y_boid(i,t+1)],marker1,'markersize',7) %plots the first half of the particles in black
            axis([0 L 0 L]);
            hold on
            plot(x_boid(i,t+1) ,y_boid(i,t+1),'k','markersize',5)
            title(['Timestep: ',num2str(t)])
            xlabel('X position')
            ylabel('Y position')
 %       end
        hold on
        disp("din mamma")
        waitforbuttonpress
    end
end