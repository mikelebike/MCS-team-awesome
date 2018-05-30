%Final Project
%Simulation 

clear all;
close all;


%-----------------------------------------------------------------------%
%--------------------------PARAMETERS-----------------------------------%


%INITIALIZE PARAMETERS
L=400;                  %system size
N_boid = 30;            %Nr of boids
dt = 0.1;               %CHECK Time-step, why is this used?
R_r = 1;                %repulsion radius

R_o = 10;                %Orientation radius
R_a = 13;               %attraction radius

v_evolve = 2;           %the evolvable speed of boid


theta_boid  = pi/4;     %turning angle for boids
theta_hoick = pi/4;     %turning angle for hoicks
phi_boid = pi;        %viewing angle
phi_hoick = pi;       %viewing angle

A_s =2*phi_boid*v_evolve^2;            %Possible sighting area
A_m =theta_boid*R_a^2;                 %Possible movement area


%Values for figure 1 from paper
theta_boid = 1000/(R_a^2);
phi_boid = 25/v_evolve^2;

e_boid = 0.2;          %sensitivity to noise
warm_up =999;          %warm up 
tot_time=1000;         %Totalt time


%SIMULATION PARAMETERS
one_sim=1;
multi_sim=0;
phase=0;

make_figure=0;
make_movie=0;

p = struct('L',L,'N_boid',N_boid,'dt',dt,'R_r',R_r,'R_o',R_o,'R_a',R_a,...
    'v_evolve',v_evolve,'theta_boid',theta_boid,'theta_hoick',theta_hoick,...
    'phi_boid',phi_boid,'phi_hoick',phi_hoick,'A_s',A_s,'A_m',A_m,'e_boid',e_boid,...
    'tot_time',tot_time,'make_figure',make_figure,'make_movie',make_movie,...
    'warm_up',warm_up);

%-----------------------------------------------------------------------%
%--------------------------SIMULATIONS----------------------------------%

%-------------------SINGLE SIMULATION WITH PLOT-------------------------%

if one_sim
    p.make_figure=1;
    p.make_movie=0;
    
    p.R_o =8;
    p.R_a=15;
    %Values for figure 1 from paper
    p.theta_boid = 1000/(R_a^2);
    p.phi_boid = 25/v_evolve^2;
    
    polarisation = boid_world(p);
    
    time_vec = 1:tot_time;
    plot(time_vec,polarisation);
    ylim([0 1])
    xlabel('Timestep');
    ylabel('Polarisation');
end


%-------------------MULTIPLE SIMULATIONS VARYING PARAMETERS------------------------%
if multi_sim
    %%%%%%%%%%%%Plots several simulations
    p.make_figure=0;
    p.make_movie=0;
    simulations=10;
    time_vec = 1:tot_time;
    
    changing_values = 13:4:30;
    p.phi_boid = 25/v_evolve^2;

    figure(2)
    hold on;
    for i=1:length(changing_values)
        
        p.R_a=changing_values(i);
        p.theta_boid = 1000/(R_a^2);
    
        polarisation(i,:)=boid_world(p);
        p_sim =plot(time_vec,polarisation(i,:));
        hold on;
        Legend{i}=num2str(changing_values(i));
        disp(num2str(i));
    end

    %Plotting
    %CHECK average är ej användbart, titta inte på den svarta linjen
    ga_avg = sum(polarisation,1)/simulations;
    p_avg = plot(time_vec,ga_avg,'k','LineWidth',1.5);
    %ylim([0 1])
    xlabel('Timestep');
    ylabel('Polarisation');
    %legend([p_sim p_avg],{'Simulation','Average'});
    legend(Legend);
end


%--------------------PHASE TRANSITION 2D HEAT MAP-------------------------%
if phase         
    p.make_figure=0;
    p.make_movie=0;
    
    simulations=3;
    R_o_values =1:1:30;
    R_a_values=13:1:30;
    
    p.phi_boid = 25/v_evolve^2;
    p.theta_boid = 1000/(R_a^2);
    p.phi_boid = 25/v_evolve^2;

    [X_allTime, frequency_info]=helpSim_FinalProject(p,simulations,R_o_values,R_a_values);

    %Heatmap
    figure(3) 
    %clims = [0 0.15];
    clims=[0 1];
    im = imagesc(R_o_values, R_a_values, frequency_info,clims); 
    colorbar;
    set(gca,'YDir','normal');
    ylabel('R_a');
    xlabel('R_o');
    
end


