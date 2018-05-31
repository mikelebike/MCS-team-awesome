

L=400;                  %System size
N_boid = 20;            %Nr of boids
N_hoick = 8;            %Nr of predators


%----GROUP----%
    
    R_r = 0;                %Repulsion radius
    R_o = 8;                %Orientation radius
    R_a = 15;               %Attraction radius
    
    A_s = 1000*R_r^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m = 25*R_r^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    v_hoick = 3;            % TEMPORARY value. Speed of hoick
    phi_hoick = A_m/(2*v_hoick^2);      %turning angle for hoicks
    theta_hoick = A_s/R_a^2;         %viewing angle
    e_boid = 0.00001;       %Sensitivity to noise


%----SEVERAL INDEPENDANT INDIVIDUALS----%

    
    R_r = 1;               %Repulsion radius
    R_o = 0;                %Orientation radius
    R_a = 0;                %Attraction radius
    
    A_s = 1000*R_r^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m = 25*R_r^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    v_hoick = 3;            % TEMPORARY value. Speed of hoick
    phi_hoick = A_m/(2*v_hoick^2);      %turning angle for hoicks
    theta_hoick = A_s/R_a^2;            %viewing angle
    e_boid = 0.00001;                   %Sensitivity to noise


%----RIVALS----%

    
    R_r = 10;               %Repulsion radius
    R_o = 10;               %Orientation radius (minus orientation)
    R_a = 0;                %Attraction radius
    
    A_s = 1000*R_r^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m = 25*R_r^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    v_hoick = 3;            % TEMPORARY value. Speed of hoick
    phi_hoick = A_m/(2*v_hoick^2);      %turning angle for hoicks
    theta_hoick = A_s/R_a^2;            %viewing angle
    e_boid = 0.00001;                   %Sensitivity to noise



