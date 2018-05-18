%This code is just for the boids, #boidswillbeboids. This is for
%simplifying the process of building the code corpus, one step at a time.
function [polarisation]=boid_world(p)

%LEGEND FOR STUFF
% CHECK
% TEMPORARY
% DELETE
% OPTIMIZE


%INITIALIZE PARAMETERS
L = p.L;
N_boid = p.N_boid;
dt = p.dt;
R_r = p.R_r;
R_o = p.R_o;
R_a = p.R_a;
v_evolve = p.v_evolve;
theta_boid = p.theta_boid;
theta_hoick = p.theta_hoick;
phi_boid = p.phi_boid;
phi_hoick = p.phi_hoick;
A_s = p.A_s;
A_m = p.A_m;
e_boid = p.e_boid;
tot_time = p.tot_time;

make_figure=p.make_figure;
make_movie=p.make_movie;
warm_up=p.warm_up;



%DEFINE HELPFUL VECTORS
r = zeros(N_boid,1);       %r is the distance from current boid to all other boids
rx_hat = zeros(N_boid,1);  %unit vector for x component
ry_hat = zeros(N_boid,1);    %unit vector for y component
polarisation = zeros(1,tot_time);

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


%------INITIALIZE BOIDS-------%
% x(i,j) gives the x coordinate of the ith particle at time j
x_boid=zeros(N_boid,tot_time+1);    %define initial x coordiantes for boids
x_boid(:,1)=L/2+L/5*rand(N_boid,1);       %initial positions

y_boid=zeros(N_boid,tot_time+1);    %define initial y coordinates for boids
y_boid(:,1)=L/2+L/5*rand(N_boid,1);       %initial positions

v_boid = zeros(N_boid,tot_time+1);   %velocity vector for all boids
vy_boid = zeros(N_boid,tot_time+1);
vx_boid = zeros(N_boid,tot_time+1);


%ITERATE OVER TIME
for t = 1:tot_time
        
    rx_temp = repmat(x_boid(:,1)',numel(x_boid(:,1)),1); %create matrix of all boids positions in x
    ry_temp = repmat(y_boid(:,1)',numel(y_boid(:,1)),1); %create matrix of all boids positions in y
    
    rx_hat = (rx_temp-x_boid(:,1));               %find distance vector between elements x-components
    ry_hat = (ry_temp-y_boid(:,1));               %find distance vector between elements x-components
    
    diagonal_temp = ones(1,N_boid)*inf;
    r = (rx_hat.^2+ry_hat.^2).^0.5+diag(diagonal_temp);         %find euclidian distance and add term to avoid division by zero.
    
    rx_hat = rx_hat./r;                     %normalize to create unit direction vector
    ry_hat = ry_hat./r;                     %normalize to create unit direction vector
    
    
    [r_sort_m,boid_index] = sort(r(:,:)');               % OPTIMIZE by making this in the beginning   %Sorting the direction vector r_ij
    
    boid_index = boid_index';                         %turn index right so every row represents a boid
    
   

    %--------------ITERATE OVER BOIDS-------------------
    for i=1:N_boid
        
        if isnan(x_boid(i,t))    %Skips this iteration if the value is NaN (dead Boid)
            continue
        end

        index_b=boid_index(i,:);
        
        %Looking at all the boids inside the repulsion radius
        inside_R_r = r(r<R_r);
        
        
        %------ SEE IF ANY BOIDS IN REPULSION AREA--------
        if ~isempty(inside_R_r)      %If there are any boids inside the repulsion radius
            vx_b = 0;
            vy_b = 0;
            
            for j=1:length(inside_R_r)
                %SEE IF VISIBLE
                %if vx_boid(i,t)*rx_hat(index_b(j)) +vy_boid(i,t)*ry_hat(index_b(j))> v_evolve*cos(theta_boid/2) 
                %Without viewing angle
                if vx_boid(i,t)*rx_hat(index_b(j)) +vy_boid(i,t)*ry_hat(index_b(j))> v_evolve*cos(theta_boid/2) 

                    vx_b = vx_b + sum(rx_hat(index_b(j)));
                    vy_b = vy_b + sum(ry_hat(index_b(j)));
                end
                
                vx_b = -vx_b/sum(r(index_b(j)));
                vy_b = -vy_b/sum(r(index_b(j)));
            end
            
        %------ ELSE CHECK BOIDS IN ORIENTATION AND ATTRACTION ZONE %-----
        else
            
            %%%%%%%%%%Find v_o - orientation
            index_vbo = find(r(i,:)>=R_r & r(i,:)<R_o);               %Index for the boids in orientation radius
            vx_bo=0;
            vy_bo=0;
            if ~isempty(index_vbo)
                for k = 1:length(index_vbo)
                vx_bo= -vx_boid(index_vbo(k))/length(index_vbo);
                vy_bo = -vy_boid(index_vbo(k))/length(index_vbo);
                end
            end
           
            %%%%%%%%%Find v_a - attraction
            index_vba = find(r(i,:)>=R_o & r(i,:)<R_a);
            vx_ba=0;
            vy_ba=0;
            
            %CHECK IF THERE ARE ANY BOIDS IN ATTRACTION AREA
            if ~isempty(index_vba)
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
        
        %%%%%Find v_noise
        vx_noise = randn(1,1);
        vy_noise = randn(1,1);
        
        vx_noise = vx_noise/(vx_noise^2 + vy_noise^2)^0.5;
        vy_noise = vy_noise/(vx_noise^2 + vy_noise^2)^0.5;
        
        %------Add together all the components for the velocity vector
        vx_boid(i,t+1) = vx_b + e_boid*vx_noise;% + omega_boid*v_pf_x_boid(i,t);
        vy_boid(i,t+1) = vy_b + e_boid*vy_noise;% + omega_boid*v_pf_y_boid(i,t);
        
        vxy_norm = (vx_boid(i,t+1)^2 + vy_boid(i,t+1)^2)^.5+0.000000001;
                
        x_boid(i,t+1) = x_boid(i,t) + v_evolve*vx_boid(i,t+1)/vxy_norm;
        y_boid(i,t+1) = y_boid(i,t) + v_evolve*vy_boid(i,t+1)/vxy_norm;
        

        %GRAPHICS
        %Plot boids
    %    if abs(x_boid(i,t)-x_boid(i,t+1))<v_boid(i,t) && abs(y_boid(i,j)-y_boid(i,j+1))<v_boid(i,t)
         if make_figure && t>warm_up
            plot([x_boid(i,t), x_boid(i,t+1)] ,[y_boid(i,t),y_boid(i,t+1)],'k-','markersize',5) %plots the first half of the particles in black
            axis([0 L 0 L]);
            hold on
            plot(x_boid(i,t+1) ,y_boid(i,t+1),'k.','markersize',14)
  %          title(['Timestep: ',num2str(t)])
 %           xlabel('X position')
  %          ylabel('Y position')
 %       end
   %     hold on
         end
    
%     %GRAPHICS
%     if makemovie
%         pause(0.00001)
%         hold off
%     end
    end
    
        %Making the video
        if make_figure && t>warm_up
            pause(0.00001)
            hold off

            if make_movie
            F(j) = getframe(gcf);  %Gets the current frame
            writeVideo(video,F(j)); %Puts the frame into the videomovie
            end
        end
        
        %----------Calculate polarisation----------%       
        vx_sum=sum(vx_boid(:,t));
        vy_sum=sum(vy_boid(:,t));
    
        polarisation(t) = (1/N_boid).*sqrt(vx_sum.^2 + vy_sum.^2);      %Polarisation

end


if make_movie
    close(video);  %Closes movie
end

end