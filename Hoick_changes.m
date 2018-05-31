        %Changing Hoick snutten    
        %---------------------------------------------%
        %-------------------HOICK---------------------%
        %---------------------------------------------%
        elseif hoick_mode && i > N_boid && t > warm_up %introduce hoick to the world after warm up is finished
            
            %-----------ITERATE OVER HOICKS------------%      
            if isnan(x(i,t)) %Skips this iteration if the value is NaN (dead Hoick)
                continue
            end
            
            
            %-----------------FIND INTERACTION WITH OTHER HOICKS------------
            %---------------------------------------------------------------%
            %---------------------------------------------------------------%
            
            index_b = hoick_index(i-N_boid,:); %Get indicies sorted by size from hoick to other hoicks          

            %---------CHECK IF ANY HOICKS ARE IN REPULSION AREA--------%
            inside_R_r_hoick = sum(r_hoick(:,i-N_boid) < R_r_hoick);   %Find how many hoicks are inside repulsion radius
               
            if not(inside_R_r_hoick==0)
                
                vx_b = 0;
                vy_b = 0;
                v_b_sum = 0;        %Initializes v_b_sum here just to make if-loop for interaction with predator work
                
                for j=1:inside_R_r_hoick
                    %SEE IF WITHIN VIEWING ANGLE
                    if vx(i,t)*rx_hat(i,index_b(j)+N_boid) + vy(i,t)*ry_hat(i,index_b(j)+N_boid) > v_hoick*cos(theta_hoick/2)
                        vx_b = vx_b + rx_hat(i,index_b(j)+N_boid);
                        vy_b = vy_b + ry_hat(i,index_b(j)+N_boid);
                        v_b_sum = v_b_sum + r(i,index_b(j)+N_boid);
                    end
                end
                vx_b = -vx_b/(v_b_sum+0.000001);
                vy_b = -vy_b/(v_b_sum+0.000001);
                
                
            %------ ELSE CHECK HOICKS IN ORIENTATION AND ATTRACTION ZONE %-----
            else
                
                %------------ORIENTATION---------%
                index_vbo = find(r_hoick(:,i-N_boid) >= R_r_hoick & r(:,i_N_boid) < R_o_hoick);               %Index for the hoicks in orientation radius
                vx_bo = 0;
                vy_bo = 0;
                if not(isempty(index_vbo))
                    for k = 1:length(index_vbo)
                        %SEE IF WITHIN VIEWING ANGLE
                        if vx(i,t)*rx_hat(i,index_b(k)+N_boid) + vy(i,t)*ry_hat(i,index_b(k)+N_boid) > v_hoick*cos(theta_hoick/2)
                            vx_bo = -vx(index_vbo(k)+N_boid);
                            vy_bo = -vy(index_vbo(k)+N_boid);
                        end
                    end
                end
                
               
                
                %--------------ATTRACTION------------------%
                index_vba = find(r_hoick(i-N_boid,:) >= R_o_hoick & r_hoick(i-N_boid,:) < R_a_hoick);
                vx_ba = 0;
                vy_ba = 0;
                
                %CHECK IF THERE ARE ANY HOICKS IN ATTRACTION AREA
            
                if not(isempty(index_vba))
                    %ITERATE OVER ALL BOIDS IN ATTRACTION AREA
                    for k = 1:length(index_vba)
                        %SEE IF WITHIN VIEWING ANGLE
                        if vx(i,t)*rx_hat(i,index_b(k)+N_boid) + vy(i,t)*ry_hat(i,index_b(k)+N_boid) > v_hoick*cos(theta_hoick/2)
                            vx_ba = vx_ba + rx_hat(i,index_vba(k)+N_boid);
                            vy_ba = vy_ba + ry_hat(i,index_vba(k)+N_boid);
                        end
                    end
                end
                
                %----DEFINE VELOCITY UNIT VECTOR v_b----
                v_b = ((vx_ba + vx_bo).^2 + (vy_ba + vy_bo).^2).^0.5+0.00000000001;
                vx_b = (vx_ba + vx_bo)/v_b;
                vy_b = (vy_ba + vy_bo)/v_b;                
                
            end 
            
            
            %----------FIND NOISE----------------------------------%
            vx_noise = 2*rand-1;
            vy_noise = 2*rand-1;
            
            vx_noise = vx_noise/(vx_noise^2 + vy_noise^2)^0.5;
            vy_noise = vy_noise/(vx_noise^2 + vy_noise^2)^0.5;
            
            
            %----------ADD COMPONENTS FOR VELOCITY VECTOR----------%
            vx(i,t+1) = vx_b + e_hoick*vx_noise + omega_hoick*vx_p;% + omega_boid*v_pf_x_boid(i,t);
            vy(i,t+1) = vy_b + e_hoick*vy_noise + omega_hoick*vy_p;% + omega_boid*v_pf_y_boid(i,t);
            %vxy_norm = (vx(i,t+1)^2 + vy(i,t+1)^2)^.5+0.000000001;
            
            
            %----------CORRECT FOR TURNING ANGLE-----------------%
            newdirection(i,t+1) = atan2(vy(i,t+1),vx(i,t+1)); %calculate "wanted" the angle of direction of the hoick
            prevdirection(i,t) = newdirection(i,t);%atan2(vy(i,t),vx(i,t));
            
            delta_angle = angdiff(prevdirection(i,t),newdirection(i,t+1));
            if (abs(delta_angle)>phi_hoick/2)
                if (delta_angle>0)
                    newdirection(i,t+1) = wrapTo2Pi(prevdirection(i,t) + phi_hoick/2);
                    first = first +1;
                else
                    newdirection(i,t+1) = wrapTo2Pi(prevdirection(i,t) - phi_hoick/2);
                    second = second + 1;
                end
            end

            %UPDATE HOICKS POSITION
            x(i,t+1) = x(i,t) + v_hoick*cos(newdirection(i,t+1));
            y(i,t+1) = y(i,t) + v_hoick*sin(newdirection(i,t+1));
                        
            %---------GRAPHICS--------%
            %-----------PLOT HOICK----------------------
            if make_figure
                x(i,t+1)=mod(x(i,t+1),L); % Jumps from the right of the box to the left or vice versa
                y(i,t+1)=mod(y(i,t+1),L); % Jumps from the top of the box to the bottom or vice versa
                
                plot([x(i,t), x(i,t+1)] ,[y(i,t),y(i,t+1)],'r-','markersize',5) %plots the first half of the particles in black
                axis([0 L 0 L]);
                hold on
                plot(x(i,t+1) ,y(i,t+1),'r.','markersize',14)
            end
        end
    end
    
    
    %----------Calculate polarisation----------%
    vx_sum=sum(vx(1:N_boid,t));
    vy_sum=sum(vy(1:N_boid,t));
    
    polarisation(t) = (1/N_boid).*sqrt(vx_sum.^2 + vy_sum.^2);      %Polarisation
    
    
    