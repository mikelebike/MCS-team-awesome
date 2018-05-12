%Final Project

%Parameters

L=400;      %system size
N_boid = 80;     %Nr of boids
N_pf = 8; %Nr of hoicks
dt = 0.1;   %Time-step
R_r = 1;    %repulsion radius

R_o = 0;      %Orientation radius
R_a =  0;     %attraction radius

v_boid = 0;
theta  = 0;
phi =0;

A_s =0;      %Possible sighting area
A_m =0;       %Possible movement area

omega_food =0;
omega_predator =0;

distribution_food=0;
distribution_predator=0;

e_boid =0;              %noise
e_hoick=0;              %Hoick noise  
warm_up =10000;         %Warm up time, 15 minutes in the paper
lifetime_hoick = 1000;  %Lifetime of a hoick
lifetime_food=0;        %Food lifetime

tot_time=10000;   %Totalt time

omega_food =0; 
omega_predator =0; 
food = true;


if food
    omega=omega_food;
    N_pf = N_food;
    e_pf=e_food;
    lifetime = lifetime_food;
    distribution = distribution_food;
    
else %Predator
    omega = omega_predator;
    N_pf=N_hoick; 
    e_pf=e_hoick;
    lifetime = lifetime_hoick;
    distribution = distribution_preadtor;
end

