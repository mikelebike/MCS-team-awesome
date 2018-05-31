
function type_variables=Hoick_types(type,v_hoick) 


%----GROUP----%
if type==1  
    R_r_hoick = 1;                %Repulsion radius
    R_o_hoick = 8;                %Orientation radius
    R_a_hoick = 15;               %Attraction radius
    
    A_s_hoick = 1000*R_r_hoick^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m_hoick = 25*R_r_hoick^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    phi_hoick = A_m_hoick/(2*v_hoick^2);      %turning angle for hoicks
    theta_hoick = A_s_hoick/R_a_hoick^2;         %viewing angle

    

%----SEVERAL INDEPENDANT INDIVIDUALS----%
elseif type==2   
    R_r_hoick = 30;               %Repulsion radius
    R_o_hoick = 0;                %Orientation radius
    R_a_hoick = 0;                %Attraction radius
    
    A_s_hoick = 1000*R_r_hoick^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m_hoick = 25*R_r_hoick^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    phi_hoick = A_m_hoick/(2*v_hoick^2);      %turning angle for hoicks
    theta_hoick = A_s_hoick/R_a_hoick^2;            %viewing angle


%----RIVALS----%
else %type==3
    
    R_r_hoick = 60;               %Repulsion radius
    R_o_hoick = 0;               %Orientation radius (minus orientation)
    R_a_hoick = 0;                %Attraction radius
    
    A_s_hoick = 1000*R_r_hoick^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
    A_m_hoick = 25*R_r_hoick^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area
    
    phi_hoick = A_m_hoick/(2*v_hoick^2);      %turning angle for hoicks
    theta_hoick = A_s_hoick/R_a_hoick^2;            %viewing angle

end

type_variables = struct('R_r_hoick',R_r_hoick,'R_o_hoick',R_o_hoick,'R_a_hoick',R_a_hoick,...
    'A_s_hoick',A_s_hoick,'A_m_hoick',A_m_hoick,'phi_hoick',phi_hoick,...
    'theta_hoick',theta_hoick);

end

