%Final Project
%Simulation 

clear all;
close all;


%-----------------------------------------------------------------------%
%--------------------------PARAMETERS-----------------------------------%


%INITIALIZE PARAMETERS
L=400;                  %system size
N_boid = 30;            %Nr of boids
N_hoick = 1;            %Nr of hoicks
dt = 0.1;               %CHECK Time-step, why is this used?

R_r_boid = 1;                %repulsion radius
R_o_boid = 10;                %Orientation radius
R_a_boid = 13;               %attraction radius

R_r_hoick = 1;                %repulsion radius
R_o_hoick = 10;                %Orientation radius
R_a_hoick = 13;               %attraction radius

v_boid = 2;           %the evolvable speed of boid
v_hoick = 4;            %Hoick speed

theta_boid  = pi/4;     %turning angle for boids
theta_hoick = pi/4;     %turning angle for hoicks
phi_boid = pi;        %viewing angle
phi_hoick = pi;       %viewing angle

A_s_boid =2*phi_boid*v_boid^2;            %Possible sighting area
A_m_boid =theta_boid*R_a_boid^2;           %Possible movement area

A_s_hoick =2*phi_hoick*v_hoick^2;            %Possible sighting area
A_m_hoick =theta_hoick*R_a_hoick^2;           %Possible movement area

omega_boid = 1;             %Sensitivity to predator
omega_hoick = 1;

%Values for figure 1 from paper
theta_boid = 1000/(R_a_boid^2);
phi_boid = 25/v_boid^2;

e_boid = 0.2;          %sensitivity to noise
e_hoick = 0.0001;
warm_up =999;          %warm up 
tot_time=1000;         %Totalt time


%-----SIMULATION PARAMETERS----%
one_sim=1;
multi_sim=0;
phase=0;

make_figure=0;
make_movie=0;

p = struct('L',L,'N_boid',N_boid,'dt',dt,'R_r_boid',R_r_boid,'R_o_boid',R_o_boid,'R_a_boid',R_a_boid,...
    'v_boid',v_boid,'theta_boid',theta_boid,'theta_hoick',theta_hoick,...
    'phi_boid',phi_boid,'phi_hoick',phi_hoick,'A_s_boid',A_s_boid,'A_m_boid',A_m_boid,'e_boid',e_boid,...
    'tot_time',tot_time,'make_figure',make_figure,'make_movie',make_movie,...
    'warm_up',warm_up,'N_hoick',N_hoick,'v_hoick',v_hoick,'omega_boid',omega_boid,...
    'R_r_hoick',R_r_hoick,'R_o_hoick',R_o_hoick,'R_a_hoick',R_a_hoick,...
    'A_s_hoick',A_s_hoick,'A_m_hoick',A_m_hoick,'e_hoick',e_hoick);

%-----------------------------------------------------------------------%
%--------------------------SIMULATIONS----------------------------------%

%-------------------SINGLE SIMULATION WITH PLOT-------------------------%

if one_sim
    p.make_figure=1;
    p.make_movie=0;
    
    p.R_o =8;
    p.R_a=15;
    %Values for figure 1 from paper
    p.theta_boid = 1000/(R_a_boid^2);
    p.phi_boid = 25/v_boid^2;
    
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
 
   
    %Kom ihåg att specificiera N_hoicks!
    N_hoick=0;
    
    changing_values = 13:4:30;
    p.phi_boid = 25/v_boid^2;

    figure(2)
    hold on;
    for i=1:length(changing_values)
        
        p.R_a=changing_values(i);
        p.theta_boid = 1000/(R_a_boid^2);
    
        polarisation(i,:)=hoick_world(p);
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
    
    p.phi_boid = 25/v_boid^2;
    p.theta_boid = 1000/(R_a_boid^2);
    p.phi_boid = 25/v_boid^2;
    
    %Kom ihåg att ändra på N_hoicks
    N_hoick =0; 

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


