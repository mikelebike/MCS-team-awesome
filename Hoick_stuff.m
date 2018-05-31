

%----GROUP----%
    
    R_r_hoick = 0;                %Repulsion radius
    R_o_hoick = 8;                %Orientation radius
    R_a_hoick = 15;               %Attraction radius
    
    A_s_hoick = 1000*R_r_hoick^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m_hoick = 25*R_r_hoick^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    phi_hoick = A_m_hoick/(2*v_hoick^2);      %turning angle for hoicks
    theta_hoick = A_s_hoick/R_a_hoick^2;         %viewing angle
    e_hoick = 0.00001;       %Sensitivity to noise


%----SEVERAL INDEPENDANT INDIVIDUALS----%
  
    R_r_hoick = 1;               %Repulsion radius
    R_o_hoick = 0;                %Orientation radius
    R_a_hoick = 0;                %Attraction radius
    
    A_s_hoick = 1000*R_r_hoick^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m_hoick = 25*R_r_hoick^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    phi_hoick = A_m_hoick/(2*v_hoick^2);      %turning angle for hoicks
    theta_hoick = A_s_hoick/R_a_hoick^2;            %viewing angle
    e_hoick = 0.00001;                   %Sensitivity to noise


%----RIVALS----%

    
    R_r_hoick = 10;               %Repulsion radius
    R_o_hoick = 10;               %Orientation radius (minus orientation)
    R_a_hoick = 0;                %Attraction radius
    
    A_s_hoick = 1000*R_r_hoick^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m_hoick = 25*R_r_hoick^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    phi_hoick = A_m_hoick/(2*v_hoick^2);      %turning angle for hoicks
    theta_hoick = A_s_hoick/R_a_hoick^2;            %viewing angle
    e_hoick = 0.00001;                   %Sensitivity to noise



