% AMPLIACION DE ROBOTICA
% PRACTICA 4: Navegacion local con campos potenciales
% Evitar obstaculos

clc
clearvars
close all
%% Carga del mapa de ocupacion

map_img=imread('mapa1_150.png');
map_neg=imcomplement(map_img);
map_bin=imbinarize(map_neg);
mapa=binaryOccupancyMap(map_bin);
show(mapa);

% Marcar los puntos de inicio y destino
hold on;
title('Señala los puntos inicial y final de la trayectoria del robot');
origen=ginput(1);
plot(origen(1), origen(2), 'go','MarkerFaceColor','green');  % Dibujamos el origen
destino=ginput(1);
plot(destino(1), destino(2), 'ro','MarkerFaceColor','red');  % Dibujamos el destino

% Configuracion del sensor (laser de barrido)
max_rango=10;
angulos=-pi/2:(pi/180):pi/2; % resolucion angular barrido laser

% Caracteristicas del vehiculo y parametros del metodo
v=0.4;            % Velocidad del robot
D=1.5;           % Rango del efecto del campo de repulsión de los obstáculos
alfa=0.25;           % Coeficiente de la componente de atracción
beta=10000;      % Coeficiente de la componente de repulsión

%% Inicialización

robot=[origen 0];     % El robot empieza en la posición de origen (orientacion cero)
path = [];                 % Se almacena el camino recorrido
path = [path; robot]; % Se añade al camino la posicion actual del robot
iteracion=0;              % Se controla el nº de iteraciones por si se entra en un minimo local
F_rep_v = [0 0];

%% Cálculo de la trayectoria

while norm(destino - robot(1:2)) > v && iteracion < 800
    % Fuerza de atracción (hacia el objetivo)
    distancia = destino' - robot(1:2)';
    F_at_v = alfa * distancia;

    % Fuerza de repulsión (obstáculos)
    obstaculo = SimulaLidar(robot, mapa, angulos, max_rango);
    obstaculo = obstaculo(~any(isnan(obstaculo), 2), :); %Elimina los nan

    F_rep_v = [0 0];
    for i = 1:size(obstaculo, 1)
        delta = robot(1:2) - obstaculo(i, 1:2);
        dist_obs = norm(delta);
        if dist_obs <= D
            %Suma total de todos los obstaculos en el radio D en conjunto
            F_rep_v = F_rep_v + beta * ((1/dist_obs - 1/D) / (dist_obs^3)) * delta;
        end
    end

    % Fuerza resultante
    F_res = F_at_v' + F_rep_v;

    % Dirección normalizada del movimiento
    dir = F_res / norm(F_res);

    % Actualización de posición del robot
    robot(1:2) = robot(1:2) + v * dir;
    robot(3) = atan2(dir(2), dir(1));

    % Almacenamos y dibujamos trayectoria
    path = [path; robot];
    plot(path(:,1), path(:,2), 'r');
    % Dibujar vectores de fuerza (quiver)
    % Dibujar vectores de fuerza normalizados (solo dirección)
    quiver(robot(1), robot(2), F_at_v(1)/norm(F_at_v), F_at_v(2)/norm(F_at_v), 4.5, 'b', 'LineWidth', 2.5);      % Atracción
    if norm(F_rep_v) > 0
        quiver(robot(1), robot(2), F_rep_v(1)/norm(F_rep_v), F_rep_v(2)/norm(F_rep_v), 4.5, 'm', 'LineWidth', 1.5);  % Repulsión
    end
    quiver(robot(1), robot(2), F_res(1)/norm(F_res), F_res(2)/norm(F_res), 4.5, 'y', 'LineWidth', 2);              % Resultante

    drawnow;

    iteracion = iteracion + 1;
end

if iteracion == 800
    fprintf('No se ha podido llegar al destino.\n');
else
    fprintf('Destino alcanzado.\n');
end


%% funcion para simular el sensor
function [obs]=SimulaLidar(robot, mapa, angulos, max_rango)
    obs=rayIntersection(mapa,robot,angulos, max_rango);
    % plot(obs(:,1),obs(:,2),'*r') % Puntos de interseccion lidar
    % plot(robot(1),robot(2),'ob') % Posicion del robot
    % for i = 1:length(angulos)
    %     plot([robot(1),obs(i,1)],...
    %         [robot(2),obs(i,2)],'-b') % Rayos de interseccion
    % end
    % % plot([robot(1),robot(1)-6*sin(angulos(4))],...
    % %     [robot(2),robot(2)+6*cos(angulos(4))],'-b') % Rayos fuera de
    % %     rango
end
