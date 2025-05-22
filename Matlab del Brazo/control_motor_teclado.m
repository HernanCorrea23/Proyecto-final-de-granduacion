% --- Inicializaci�n y Configuraci�n ---
clear all; 
clc;

L = Link('revolute', 'd', 0, 'a', 0, 'alpha', 0, 'offset', 0);
motorRobot = SerialLink(L, 'name', 'MotorUnico');
motorRobot.qlim = [-pi pi];
q_inicial_rad = 0;

t_total_movimiento = 2; 
t_espera = 1;         
pasos_simulacion = 100;
tiempo_vector = linspace(0, t_total_movimiento, pasos_simulacion);

% --- Creaci�n de la Figura ---
fig = figure('Name', 'Control de Motor con Teclado', 'NumberTitle', 'off');
ws = [-0.5 0.5 -0.5 0.5 -0.5 0.5];
motorRobot.plot(q_inicial_rad, 'workspace', ws);
title('Presiona ''d'' (derecha), ''i'' (izquierda), o ''q'' (salir)');
disp('Ventana de gr�ficos activa. Presiona una tecla...');
disp('  ''d'' para girar a la derecha.');
disp('  ''i'' para girar a la izquierda.');
disp('  ''q'' para salir.');

% --- Bucle Principal de Interacci�n ---
terminar_programa = false;
while ishandle(fig) && ~terminar_programa
    
    % Espera a que se presione una tecla EN LA FIGURA
    k = waitforbuttonpress; 
    
    % Si k == 1, fue una tecla. Si k == 0, fue un clic del rat�n.
    if k == 1 
        tecla = get(fig, 'CurrentCharacter'); % Obtiene la tecla presionada
        q_actual = motorRobot.getpos(); % Obtener posici�n actual antes de mover

        switch lower(tecla) % Usar lower para aceptar may�sculas y min�sculas
            case 'd'
                disp('Tecla "d" presionada: Moviendo a la derecha (+90 grados)');
                q_objetivo_rad = deg2rad(90);
                
                [q_tray1, ~, ~] = jtraj(q_actual, q_objetivo_rad, tiempo_vector);
                disp('Moviendo al objetivo...');
                for i_paso = 1:pasos_simulacion
                    motorRobot.animate(q_tray1(i_paso,:));
                    % drawnow limitrate; % Opcional
                end
                title('En +90 grados. Esperando...');
                disp(['En +90 grados. Esperando ', num2str(t_espera), ' segundos...']);
                drawnow; 
                pause(t_espera);
                
                [q_tray2, ~, ~] = jtraj(q_objetivo_rad, q_inicial_rad, tiempo_vector);
                disp('Regresando al origen...');
                for i_paso = 1:pasos_simulacion
                    motorRobot.animate(q_tray2(i_paso,:));
                end
                title('Presiona ''d'', ''i'', o ''q''');
                disp('En posici�n inicial.');

            case 'i'
                disp('Tecla "i" presionada: Moviendo a la izquierda (-90 grados)');
                q_objetivo_rad = deg2rad(-90);
                
                [q_tray1, ~, ~] = jtraj(q_actual, q_objetivo_rad, tiempo_vector);
                disp('Moviendo al objetivo...');
                for i_paso = 1:pasos_simulacion
                    motorRobot.animate(q_tray1(i_paso,:));
                end
                title('En -90 grados. Esperando...');
                disp(['En -90 grados. Esperando ', num2str(t_espera), ' segundos...']);
                drawnow;
                pause(t_espera);
                
                [q_tray2, ~, ~] = jtraj(q_objetivo_rad, q_inicial_rad, tiempo_vector);
                disp('Regresando al origen...');
                for i_paso = 1:pasos_simulacion
                    motorRobot.animate(q_tray2(i_paso,:));
                end
                title('Presiona ''d'', ''i'', o ''q''');
                disp('En posici�n inicial.');
                
            case 'q'
                disp('Tecla "q" presionada: Saliendo...');
                terminar_programa = true;
                
            otherwise
                % Otra tecla presionada, informar y continuar esperando
                 disp(['Tecla no reconocida: ', tecla, '. Esperando d, i, o q.']);
        end
    else
        % Fue un clic del rat�n, no una tecla. Puedes ignorarlo o a�adir l�gica.
        % disp('Clic del rat�n detectado, esperando tecla...');
    end
    
    % Peque�a pausa para asegurar que la figura se actualice si es necesario
    % antes de la siguiente llamada a waitforbuttonpress
    if ishandle(fig) && ~terminar_programa
        drawnow;
    end
end

% Asegurarse de que la figura se cierre si el bucle termin�
if ishandle(fig)
    close(fig);
end
disp('Programa terminado.');