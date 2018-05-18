function [X_allTime,frequency_info]=helpSim_FinalProject(p,simulations,R_o_values,R_a_values)


    %Initializing necessary 
    time = p.tot_time;
    X_allTime = zeros(simulations,time);                        %X is the value we are interested in
    endX = zeros(simulations,length(R_a_values));
    histS=zeros(length(R_o_values),length(R_a_values));
    
    %R_o is the paramter value on the x-axis
    
    count1=0;
    %Parameter changes
    for v1=R_o_values
    count1= count1 +1;
    p.R_o = v1;   

        count2=0;
        for v2=R_a_values
            count2=count2+1;
            p.R_a =v2;
            
            %Limiation according to figure 1
            p.theta_boid = 1000/(p.R_a^2);
 
            
            
            %Many Simulations
            for s=1:simulations   
                X_allTime(s,:) = boid_world(p);
                endX(s, count2) = X_allTime(s,end);                          %Value at the end
            end
            
            avgX(count1,count2)=sum(endX(:,count2))/simulations;     %Give the average last value
        end
            
        %histS(count1,:)=hist(avgX(count1,:),R_a_values); 
        disp(count1);
    end
    
    %specified_bar = histS(round(length(R_o_values)/2),:);     %For the middle parameter_value
    %specified_bar = histS(histS(end,:)/simulations);           %For last values of     
    %frequency_info = histS'/simulations;  
    
    frequency_info = avgX; 
    %For all parameter values