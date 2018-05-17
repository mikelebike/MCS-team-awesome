%Final Project
%Simulation 
clear all;
close all;

%INITIALIZE PARAMETERS
L=400;                  %system size
N_boid = 40;            %Nr of boids
dt = 0.1;               %CHECK Time-step, why is this used?
R_r = 1;                %repulsion radius

R_o = 15;               %Orientation radius
R_a = 15;              %attraction radius

v_evolve = 2;           %the evolvable speed of boid

theta_boid  = pi/4;     %turning angle for boids
theta_hoick = pi/4;     %turning angle for hoicks
phi_boid = pi;          %viewing angle
phi_hoick = pi;         %viewing angle

A_s =0;                 %Possible sighting area
A_m =0;                 %Possible movement area

e_boid = 0.2;           %sensitivity to noise
warm_up = 10000;        % CHECK do we really need this? %Warm up time, 15 minutes in the paper
tot_time=100;           %Totalt time


%SIMULATION PARAMETERS
one_sim=true;
phase=false;
make_figure=false;
make_movie=false;


p = struct('L',L,'N_boid',N_boid,'dt',dt,'R_r',R_r,'R_o',R_o,'R_a',R_a,...
    'v_evolve',v_evolve,'theta_boid',theta_boid,'theta_hoick',theta_hoick,...
    'phi_boid',phi_boid,'phi_hoick',phi_hoick,'A_s',A_s,'A_m',A_m,'e_boid',e_boid,...
    'tot_time',tot_time,'make_figure',make_figure,'make_movie',make_movie);

%parameters=[vec,L,N_boid,dt,R_r,R_o,R_a,v_evolve,theta_boid,theta_hoick,phi_boid,phi_hoick,A_s,A_m,e_boid,warm_up,tot_time];

%boid_world(parameters);

if one_sim
    p.make_figure=true;
    p.make_movie=false;
    boid_world(p);  
end


%PHASE TRANSITION 2D HEAT MAP
if phase         
    p.make_figure=false;
    p.make_movie=false;
    
    simulations=1;
    R_o_values =0:(2*pi)/20:2*pi;
    R_a_values=1:N-1;
    
    [X_allTime, frequency_info]=helpSim_FinalProject(p,makemovie,simulations,R_o_values,R_a_values);

    %Histogram
    %figure(2)
    %bar(specified_bar);

    %Heatmap
    figure(3) 
    %clims = [0 0.15];
    clims=[0 1];
    im = imagesc(error_values, neighbour_values, frequency_info,clims); 
    colorbar;
    set(gca,'YDir','normal');
    ylabel('Number of neighbours');
    xlabel('Error term');
    
end


