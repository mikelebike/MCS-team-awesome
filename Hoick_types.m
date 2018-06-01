
function type_variables=Hoick_types(type,v_hoick) 


%----GROUP----%
if type==1  
    R_r_hoick = 3;                %Repulsion radius
    R_o_hoick = 10;                %Orientation radius
    R_a_hoick = 100;               %Attraction radius
    R_avoid = 6;
    omega_independence=3;

%----SEVERAL INDEPENDANT INDIVIDUALS----%
elseif type==2   
    R_r_hoick = 3;               %Repulsion radius
    R_o_hoick = 0;                %Orientation radius
    R_a_hoick = 0;                %Attraction radius
    R_avoid = 0;
    omega_independence=0;
    
%----RIVALS----%
else %type==3
    
    R_r_hoick = 30;               %Repulsion radius
    R_o_hoick = 0;               %Orientation radius (minus orientation)
    R_a_hoick = 0;                %Attraction radius
    R_avoid = 0;
    omega_independence=5;

end

A_s_hoick = 1000*R_r_hoick^2;        % TEMPORARY value (same value as used for fig 1). Possible sighting area
A_m_hoick = 25*R_r_hoick^2;          % TEMPORARY value (same value as used for fig 1). Possible movement area

phi_hoick = A_m_hoick/(2*v_hoick^2);      %turning angle for hoicks
theta_hoick = A_s_hoick/R_a_hoick^2;            %viewing angle

type_variables = struct('R_r_hoick',R_r_hoick,'R_o_hoick',R_o_hoick,'R_a_hoick',R_a_hoick,...
    'A_s_hoick',A_s_hoick,'A_m_hoick',A_m_hoick,'phi_hoick',phi_hoick,...
    'theta_hoick',theta_hoick,'R_avoid',R_avoid,'omega_independence',omega_independence);

end

