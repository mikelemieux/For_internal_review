clear
clc

%%% sample takeoff input file
if ~exist('model')
    load model_xb1.mat  %load the model / aircraft configuration
    geom = defineGeometry(model.version);  %load the aircraft geometry
end

% loopable inputs
landing_gear = [1]; %1 extended | 0 Retracted
dT = [0];
throttle_setting = [2.0];
engines = [2];
kcas = [300];
alts  = [2801];
weight = [16500:100:17500];

% other inputs
input_cg_value = 0; % 0 = use default CG... 1 = input a CG
CG = [583 0 100];  %only used if input_cg_value = 1

ground_effect = [0]; %0 if the AC is in free air, 1 if in GE
max_mach = 0.9;
geom.propScalar = 0.9;

% declarations
iteration = 1;

for ige = 1:numel(ground_effect)
    if ground_effect(ige) == 1
        heightAGL = 8;
    else
        heightAGL = 1400;
    end
    
    for ilg = 1:numel(landing_gear)
        for idT = 1:numel(dT)
            for ithrottle = 1:numel(throttle_setting)
                for iFn = 1:numel(engines)
                     for ikcas = 1:numel(kcas)
                        for ia=1:numel(alts)
                            for iwt=1:numel(weight)
                                error_flag = 0;
                                mach = kcas2mach(kcas(ikcas),alts(ia));
                                if mach > max_mach
                                    mach = max_mach;
                                end

                                throttle = -2*ones(1,geom.n_engines);
                                throttle(1:engines(iFn)) = throttle_setting(ithrottle); % Set operating engine ABs ON, Center engine is critical for thrust

                                if input_cg_value == 0
                                    state = defineState(geom,'altitude',alts(ia),'throttle', throttle,'heightAGL',heightAGL,...
                                        'mach', mach,'dT',dT(idT),'weight',weight(iwt),'gearDown',landing_gear(ilg));
                                else            
                                    state = defineState(geom,'altitude',alts(ia),'throttle', throttle,'CG', CG,'heightAGL',heightAGL,...
                                        'mach', mach,'dT',dT(idT),'weight',weight(iwt),'gearDown',landing_gear(ilg));
                                end

                                [output state] = climb_capability (model,geom,state);
                                
                                output_values (iteration,1) = output;
                                state_values (iteration,1) = state;
                                Results.weight (iteration,1) = state.weight;
                                Results.altitude(iteration,1)= alts(ia);
                                Results.AGL(iteration,1) = heightAGL;
                                Results.theta(iteration,1) = state.pitch;                                
                                Results.alphaTrim(iteration,1) = state.alpha;
                                Results.stabTrim(iteration,1)  = state.dhstab;
                                Results.gammaTrim(iteration,1) = state.gamma;
                                Results.betatrim(iteration,1) = state.beta;
                                Results.rolltrim(iteration,1) = state.roll;
                                Results.kcas(iteration,1) = kcas(ikcas);
                                Results.ktas(iteration,1) = state.ktas;
                                Results.mach(iteration,1)  = state.mach;
                                Results.fpm(iteration,1)       = state.mach .* state.a .* sind(state.gamma) / 12 * 60;
                                Results.max_throttle(iteration,1) = max(throttle);
                                Results.min_throttle(iteration,1) = min(throttle);                            
                                Results.num_engines (iteration,1) = engines(iFn);
                                Results.dT (iteration,1) = dT(idT);
                                Results.Temp (iteration,1) = state.T;
                                Results.ff(iteration,1) = sum(output.prop.FF);
                                Results.landing_gear (iteration,1) = state.gearDown;
                                Results.CG (iteration,1) = state.CG(1);
                                Results.rudder(iteration,1) = state.drudder;
                                Results.l_aileron(iteration,1) = state.daileron_l;
                                Results.r_aileron(iteration,1) = state.daileron_r;
                                Results.error(iteration,1) = error_flag;
                                Results.cl_earth(iteration,1) = output.aero.earth(2);
                                Results.cl_wind(iteration,1) = output.aero.wind(2);
                                Results.cd_earth(iteration,1) = output.aero.earth(1);
                                Results.cd_wind(iteration,1) = output.aero.wind(1);
                                Results.q(iteration,1) = state.q;
                                Results.fn_factor(iteration,1) = geom.propScalar;
                                
                                %    Results.model(iteration,1) = model.version;
                                results_table = struct2table(Results);                            
                                % 

                                iteration = iteration + 1;
                            end
                        end
                    end
                end
            end
        end
    end
end


 

