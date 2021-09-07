function [output state] = climb_capability (model,geom,state)

    [outputbase,debug] = queryModel(model,geom,state);

    [state_alphastab,outputalphastab] = trimWithAlphaStab(model,geom,state);


    if isnan(state_alphastab.alpha)
        error_flag = 1; 
        state = state;
        output = outputbase;
    else
        [state_rudder,output_rudder] = trimWithRudder(model,geom,state_alphastab);
        if isnan(state_rudder.alpha)
            error_flag = 2; 
            state = state_alphastab;
            output = output_alphastab;
        else
            [state_rudder,output_temp] = trimWithAlphaStabGamma(model,geom,state_rudder);     
            
            [state_rudderaileron,output_rudderaileron] = trimWithRudderAileron(model,geom,state_rudder);
            if isnan(state_rudderaileron.alpha)
                error_flag = 3;
                state = state_rudder; 
                output = output_rudder;                                   
            else
                [state_betarudderaileron,output] = trimWithBetaRudderAileron(model,geom,state_rudderaileron);
                if isnan(state_betarudderaileron.alpha)
                    error_flag = 4;  
                    state = state_rudderaileron; 
                    output = outputbase;                                       
                else
                    [state_gamma,output_gamma] = trimWithAlphaStabGamma(model,geom,state_betarudderaileron);
                    if isnan(state_gamma.alpha)
                        error_flag = 5;  
                        state = state_rudderaileron; 
                        output = output_rudderaileron;                                           
                    else
                        state = state_gamma;
                        output = output_gamma;
                    end
                end
            end
        end
    end 

end