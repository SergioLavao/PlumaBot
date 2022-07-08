%line = [0.6 -0.3 0.1; 0.6 0.2 0.1];
line = [0 0 0; 1 0 0];
traj =  Traj_Planner(1,line,1,0.2,0.03); % Type, p. control, Vmax,%Aceleracion, Ts
%circle = [0.5 0.1 0.2; 0.7 0 0.2; 0.5 -0.1 0.2; 0.3 0 0.2; 0.5 0.1 0.2];
%traj = traj_planner(2, circle, [1 2 3 4 5],[0 0 0 ; 0 0 0],0.1); % Type, p. control, Tiempos , vO Y VF, Ts
