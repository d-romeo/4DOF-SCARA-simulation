function mainFinal()
   clc; close all; 
   evalin('base', 'clear');

    % Prima Finestra
    createFirstWindow();
    l1 = evalin('base', 'l1'); l2 = evalin('base', 'l2'); l3 = evalin('base', 'l3'); l4 = evalin('base', 'l4');
    m1 = evalin('base', 'm1'); m2 = evalin('base', 'm2'); m3 = evalin('base', 'm3'); m4 = evalin('base', 'm4');

    aMotor = evalin('base', 'aMax');dMotor = evalin('base', 'dMax'); vMotor = evalin('base', 'vMax'); mp = evalin('base', 'boxMass');
    tau = eye(4);
    tau(1,1) = evalin('base', 'gear1'); tau(2,2) = evalin('base', 'gear2'); tau(3,3) = evalin('base', 'gear3'); tau(4,4) = evalin('base', 'gear4');
     
    % Definizione dei raggi
    rMax = l2 + l3 + l4; rMin = abs(l2 - l3); zMax = l1;  

    L=[l1; l2; l3; l4]'; 
    % home position 
    home = getHomePosition(L); 
    % homePosition=[0.95; 0.95 ;0.1; pi/4];
    homePosition = [home.Sx;home.Sy;home.Sz;home.phi]; 

    createSecondWindow(rMin, rMax, zMax, l1,l2,l3,l4, homePosition);

    % Seconda Finestra
    xPick = evalin('base', 'pickX'); yPick = evalin('base', 'pickY'); zPick = evalin('base', 'pickZ'); PickAngle = evalin('base', 'pickPhi');
    xPlace = evalin('base', 'placeX'); yPlace = evalin('base', 'placeY'); PlaceAngle = evalin('base', 'placePhi'); ripiano = evalin('base', 'shelf'); zPlace =  str2double(ripiano)*(zMax/3); 
    
    r0 = 0.075; % raggio 
    jg3=1/12*m4*l3^2;
    jg2=1/12*m3*l2^2;
    jg1=1/12*m2*l1^2;
    jg0=1/2*m1*r0^2;
    
    g = -9.81;
    L=[l1; l2; l3; l4]'; 
    
    Fse=zeros(16,1);
    Fse(3)= (m4+mp)*g;
    Fse(7) = m3*g; 
    Fse(11) = m2*g; 
    Fse(15) = m1*g;
    
    
    pickPosition=[xPick;yPick;zPick;PickAngle]; 
    placePosition=[xPlace;yPlace;zPlace;PlaceAngle]; 
    pickPoints = tempPoint(pickPosition', str2double(ripiano)); 
    placePoints = tempPoint(placePosition',str2double(ripiano)); 
    
    %prima traccia
    positions = [homePosition,pickPoints(:,1),pickPosition];
    traceReach = getTrace(positions, tau,L);  
    
    %seconda traccia
    positions = [pickPosition,pickPoints(:,1),placePoints(:,1),placePosition];
    tracePick = getTrace(positions, tau,L); 
    
    %terza traccia 
    positions = [placePosition,placePoints(:,1),homePosition];
    traceHome = getTrace(positions, tau,L); 
    
    %tot traccie
    totalTrace = struct();
    totalTrace.traceReach = traceReach; 
    totalTrace.tracePick = tracePick;
    totalTrace.traceHome = traceHome;
    campi = fieldnames(totalTrace); 
    
    figure(5);
    hold on;
    axis equal;
    xlim([(-rMax-1)  (rMax+1)]);
    ylim([(-rMax-1) (rMax+1)]);
    zlim([0 zMax+1]);
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(3);

    vettore_posizioni_finale = zeros(4,1);
    vettore_posizioni_motor = zeros(4,1);  
    tempo_motor = zeros(4,1);
    tLine = zeros(4,1); 
    posizioni_punti_finale = zeros(4,1);
    totFcq = zeros(4,1); 
    velocita_motor = zeros(4,1); 
    acc_motor = zeros(4,1); 
    totCond = zeros(1);  
    tempo_motor_end  = zeros(4,1); 
    condition_value = -mp * 9.81;
    custom = 0;

    for traccia = 1:length(campi)
        custom_vector = 0;
        posizioni_punti = zeros(4,1); 
        vettore_posizioni = zeros(4,1);
        vettore_posizioniP = zeros(4,1); 
        vettore_posizioniPP = zeros(4,1);
        tLine_end1_vec = 0; 
        conditioning = 0; 
        tempo_motor_end1_vec = 0; 
        J = zeros(16,4);
        jac = 0; 
        Jp = zeros(16,4);
        Spp = zeros(16,1); 
        nomeCampo = campi{traccia};        
        matrice = totalTrace.(nomeCampo);
        Fcq = zeros(4,1); 
        mp = evalin('base', 'boxMass');
        res = linesAndParabola2(matrice,aMotor, dMotor, vMotor);
    
      if traccia ~= 1 
            tLine_end1 = tLine(1, size(tLine, 2)); 
            tempo_motor_end1 = tempo_motor(:, size(tempo_motor, 2)); 
            tempo_motor_end  = [tempo_motor_end,tempo_motor_end1]; 
            tempo_motor_end1_vec = repmat(tempo_motor_end1, 1, size(res.tempo, 2));
            tLine_end1_vec = repmat(tLine_end1, 1, length(res.tLine));
            tempo_motor = [tempo_motor, res.tempo + tempo_motor_end1_vec]; 
            tLine = [tLine, res.tLine + tLine_end1_vec]; 
      else 
            tempo_motor = [tempo_motor, res.tempo]; 
            tLine = [tLine, res.tLine]; 
      end
        
        for i = 1:size(res.q,2)    
                posizioni_punti(:,i) = res.q(:,i) .*diag(tau); 
        end

        % otteniamo valori dei giunti utilizzando la trasmissione
        for i = 1:size(res.vettore_posizioni,2)    
            vettore_posizioni(:,i) = res.vettore_posizioni(:,i) .*diag(tau); 
        end
        lengthPosizioni = size(res.vettore_posizioni,2); 
    
        % velocita/acc dei motori
        tempo = tempo_motor(:,(2:end)); 
        vettore_posizioniP = res.vel; 
        vettore_posizioniPP = [res.acc, zeros(4,1)];

        % velocità e acc dei giunti
        vettore_posizioniP_giunti =  vettore_posizioniP.*diag(tau); 
        vettore_posizioniPP_giunti =  vettore_posizioniPP.*diag(tau); 
    
         if traccia ~= 2
            mp = 0; 
         end 

         % calcolo dei torque
          Fse(3)=(m4+mp)*g;
          M = diag([m4+mp,m4+mp,m4+mp,jg3+mp*(l3^2),m3,m3,m3,jg2,m2,m2,m2,jg1,m1,m1,m1,jg0]);
            
         for i = 1:size(vettore_posizioni,2)    
                J=SCARAjacdin(vettore_posizioni(:,i),L);
                Jp=SCARAjacPdin(vettore_posizioni(:,i),vettore_posizioniP_giunti(:,i),L);
                Spp=Jp*vettore_posizioniP_giunti(:,i)+J*vettore_posizioniPP_giunti(:,i);
                Fsi=-M*Spp;
                Fs=(Fse+Fsi);
                Fcq(:,i)=-J'*Fs;
                jac = jacobiano(L,vettore_posizioni(:,i)); %cond giunti
                conditioning(i) = (cond(jac))^-1; 
                if traccia == 2
                    custom_vector(i) = condition_value;
                else 
                    custom_vector(i) = 0;
                end
         end 
            custom = [custom, custom_vector];
            totCond = [totCond,conditioning];
            totFcq = [totFcq,Fcq]; % giunto
            vettore_posizioni_finale = [vettore_posizioni_finale, vettore_posizioni]; 
            vettore_posizioni_motor = [vettore_posizioni_motor, res.vettore_posizioni]; 
            posizioni_punti_finale = [posizioni_punti_finale, posizioni_punti];
            velocita_motor = [velocita_motor,vettore_posizioniP]; 
            acc_motor = [acc_motor,vettore_posizioniPP];
            
            % plot in 3d (rende lenta la simulazione, commentare per vedere subito i grafici)
            for i = 1:lengthPosizioni
                PlotScara3D(vettore_posizioni(:,i),L,5,traccia,pickPosition,placePosition,homePosition,pickPoints(:,1), placePoints(:,1), ripiano)     
                drawnow;
            end
    end

    % vettore tempo per conditioning
    tempCond= linspace(0,tempo(1,length(tempo)),length(totCond(2:end)));
    % velocità e acc dopo trasmissioni    
    acc = acc_motor.*diag(tau); 
    velocita = velocita_motor.*diag(tau); 
    % torque motore
    torque_motor = (-totFcq).*diag(tau); 
    
    % tempo per plot
    tempo_motor_end  = [tempo_motor_end,tempo(:,end)]; 
    
    %posizioni punti finale plot
    posizioni_punti_finale_reali = posizioni_punti_finale./diag(tau);
    
    % simscape
    q1_sim = [tempo(1,:)',vettore_posizioni_finale(1,2:end)']; q2_sim = [tempo(2,:)',vettore_posizioni_finale(2,2:end)']; q3_sim = [tempo(3,:)',vettore_posizioni_finale(3,2:end)']; q4_sim = [tempo(4,:)',vettore_posizioni_finale(4,2:end)']; 
    v1_sim = [tempo(1,:)',velocita(1,2:end)']; v2_sim = [tempo(2,:)',velocita(2,2:end)']; v3_sim = [tempo(3,:)',velocita(3,2:end)']; v4_sim = [tempo(4,:)',velocita(4,2:end)']; 
    a1_sim = [tempo(1,:)',acc(1,2:end)']; a2_sim = [tempo(2,:)',acc(2,2:end)']; a3_sim = [tempo(3,:)',acc(3,2:end)']; a4_sim = [tempo(4,:)',acc(4,2:end)']; 
    assignin('base','q1_sim', q1_sim); assignin('base','q2_sim', q2_sim); assignin('base','q3_sim', q3_sim); assignin('base','q4_sim', q4_sim); 
    assignin('base','v1_sim', v1_sim); assignin('base','v2_sim', v2_sim); assignin('base','v3_sim', v3_sim); assignin('base','v4_sim', v4_sim); 
    assignin('base','a1_sim', a1_sim); assignin('base','a2_sim', a2_sim); assignin('base','a3_sim', a3_sim); assignin('base','a4_sim', a4_sim); 
    force_sim = [tempCond',custom(2:end)']; assignin('base','force', force_sim); assignin('base', 'custom_vector', custom(2:end));

    assignin('base','acc', acc); assignin('base','vec',velocita);  assignin('base', 'J', J); assignin('base', 'cond', totCond);
    assignin('base', 'tLine', tLine(:,2:end));assignin('base', 'vettore_posizioni_finale', vettore_posizioni_finale(:,2:end)); assignin('base', 'tempo', tempo);
    
    
    %% SEZIONE PLOT
    figure(1)
    grid on
    hold on
    title('Position Joint Space')
    
    p1 = plot(tempo(1,:),vettore_posizioni_finale(1,2:end),'LineWidth', 2);
    p2 = plot(tempo(2,:),vettore_posizioni_finale(2,2:end), 'LineWidth', 2);  
    p3 = plot(tempo(3,:),vettore_posizioni_finale(3,2:end),'LineWidth', 2); 
    p4 = plot(tempo(4,:),vettore_posizioni_finale(4,2:end), 'LineWidth', 2); 

    plot(tLine(1,2:end), posizioni_punti_finale(1,2:end), 'b'); plot(tLine(1,2:end), posizioni_punti_finale(1,2:end), 'b*');       
    plot(tLine(2,2:end), posizioni_punti_finale(2,2:end), 'r'); plot(tLine(2,2:end), posizioni_punti_finale(2,2:end), 'r*')       
    plot(tLine(3,2:end), posizioni_punti_finale(3,2:end), 'g'); plot(tLine(3,2:end), posizioni_punti_finale(3,2:end), 'g*')       
    plot(tLine(4,2:end), posizioni_punti_finale(4,2:end), 'm'); plot(tLine(4,2:end), posizioni_punti_finale(4,2:end), 'm*'); 

    for giunto = 2:size(tempo_motor_end, 1)
        x_values = tempo_motor_end(giunto, :); 
    
        for idx = 1:length(x_values)-1
            xline(x_values(idx), '--', 'LineWidth', 0.5); 
            x_text = (x_values(idx) + x_values(idx+1)) / 2; 
            y_text = max(vettore_posizioni_finale(:)) * 0.8; 
            text(x_text, y_text, sprintf('Trace %d', idx), ...
                 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
        end
    end
    xlabel('Time [s]'); ylabel('q [rad]');
    legend([p1, p2, p3, p4], 'q1', 'q2', 'q3', 'q4', 'Location', 'best');
    hold off
    
    % PLOT POSIZIONI MOTORI
    figure(2)
    grid on
    hold on
    title('Position Joint Space Motor')
    plot(tempo(1,:),vettore_posizioni_motor(1,2:end),'LineWidth',2); 
    plot(tempo(2,:),vettore_posizioni_motor(2,2:end), 'LineWidth',2); 
    plot(tempo(3,:),vettore_posizioni_motor(3,2:end),'LineWidth',2);
    plot(tempo(4,:),vettore_posizioni_motor(4,2:end),'LineWidth',2); 
    plot(tLine(1,2:end), posizioni_punti_finale_reali(1,2:end), 'b'); plot(tLine(1,2:end), posizioni_punti_finale_reali(1,2:end), 'b*');       
    plot(tLine(2,2:end), posizioni_punti_finale_reali(2,2:end), 'r'); plot(tLine(2,2:end), posizioni_punti_finale_reali(2,2:end), 'r*')       
    plot(tLine(3,2:end), posizioni_punti_finale_reali(3,2:end), 'g'); plot(tLine(3,2:end), posizioni_punti_finale_reali(3,2:end), 'g*')       
    plot(tLine(4,2:end), posizioni_punti_finale_reali(4,2:end), 'm'); plot(tLine(4,2:end), posizioni_punti_finale_reali(4,2:end), 'm*'); 
   
    for giunto = 2:size(tempo_motor_end, 1)
        x_values = tempo_motor_end(giunto, :); 
    
        for idx = 1:length(x_values)-1
            xline(x_values(idx), '--', 'LineWidth', 0.5); 
            x_text = (x_values(idx) + x_values(idx+1)) / 2; 
            y_text = max(posizioni_punti_finale_reali(:)) * 0.8; 
            text(x_text, y_text, sprintf('Traccia %d', idx), ...
                 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
        end
    end
    xlabel('Time [s]'); ylabel('q [rad]');
    legend([p1, p2, p3, p4], 'q1', 'q2', 'q3', 'q4', 'Location', 'best');
    hold off
  
    figure(8)
    grid on
    hold on
    title('Conditioning')
    plot(tempCond,totCond(2:end),'LineWidth',2); 
    
    figure(4)
    %velocità 
    subplot(3,4,1); plot(tempo(1,:),velocita_motor(1,2:end), 'LineWidth',1.5); title('Vel Motor 1'); xlabel('Time [s]'); ylabel('dq1/dt [rad/s]'); grid on; 
    subplot(3,4,2); plot(tempo(2,:),velocita_motor(2,2:end), 'LineWidth',1.5); title('Vel Motor 2'); xlabel('Time [s]'); ylabel('dq2/dt [rad/s]'); grid on;
    subplot(3,4,3); plot(tempo(3,:),velocita_motor(3,2:end), 'LineWidth',1.5); title('Vel Motor 3'); xlabel('Time [s]'); ylabel('dq3/dt [rad/s]'); grid on;
    subplot(3,4,4); plot(tempo(4,:),velocita_motor(4,2:end), 'LineWidth',1.5); title('Vel Motor 4'); xlabel('Time [s]'); ylabel('dq4/dt [rad/s]'); grid on;
    % accelerazioni
    subplot(3,4,5); plot(tempo(1,:),acc_motor(1,2:end), 'LineWidth',1.5); title('Acc Motor 1'); xlabel('Time [s]'); ylabel('dq1/dt [rad/s^2]'); grid on;
    subplot(3,4,6); plot(tempo(2,:),acc_motor(2,2:end), 'LineWidth',1.5); title('Acc Motor 2'); xlabel('Time [s]'); ylabel('dq2/dt [rad/s^2]'); grid on;
    subplot(3,4,7); plot(tempo(3,:),acc_motor(3,2:end), 'LineWidth',1.5); title('Acc Motor 3'); xlabel('Time [s]'); ylabel('dq3/dt [rad/s^2]'); grid on;
    subplot(3,4,8); plot(tempo(4,:),acc_motor(4,2:end), 'LineWidth',1.5); title('Acc Motor 4'); xlabel('Time [s]'); ylabel('dq4/dt [rad/s^2]'); grid on;
    % torque
    subplot(3,4,9); plot(tempo(1,:),torque_motor(1,2:end), 'LineWidth',1.5); title('Torque Motor 1'); xlabel('Time [s]'); ylabel('T1 [N*m]'); grid on;
    subplot(3,4,10); plot(tempo(2,:),torque_motor(2,2:end), 'LineWidth',1.5); title('Torque Motor 2'); xlabel('Time [s]'); ylabel('T2 [N*m]'); grid on;
    subplot(3,4,11); plot(tempo(3,:),torque_motor(3,2:end), 'LineWidth',1.5); title('Torque Motor 3'); xlabel('Time [s]'); ylabel('T3 [N*m]'); grid on;
    subplot(3,4,12); plot(tempo(4,:),torque_motor(4,2:end), 'LineWidth',1.5); title('Torque Motor 4'); xlabel('Time [s]'); ylabel('T4 [N*m]'); grid on;
    sgtitle('Motor Side');
    
    figure(9)
    %velocità 
    subplot(3,4,1); plot(tempo(1,:),velocita(1,2:end), 'LineWidth',1.5); title('Vel Joint 1'); xlabel('Time [s]'); ylabel('dq1/dt [rad/s]'); grid on; 
    subplot(3,4,2); plot(tempo(2,:),velocita(2,2:end), 'LineWidth',1.5); title('Vel Joint 2'); xlabel('Time [s]'); ylabel('dq2/dt [rad/s]'); grid on;
    subplot(3,4,3); plot(tempo(3,:),velocita(3,2:end), 'LineWidth',1.5); title('Vel Joint 3'); xlabel('Time [s]'); ylabel('dq3/dt [rad/s]'); grid on;
    subplot(3,4,4); plot(tempo(4,:),velocita(4,2:end), 'LineWidth',1.5); title('Vel Joint 4'); xlabel('Time [s]'); ylabel('dq4/dt [rad/s]'); grid on;
    % accelerazioni
    subplot(3,4,5); plot(tempo(1,:),acc(1,2:end), 'LineWidth',1.5); title('Acc Joint 1'); xlabel('Time [s]'); ylabel('dq1/dt [rad/s^2]'); grid on;
    subplot(3,4,6); plot(tempo(2,:),acc(2,2:end), 'LineWidth',1.5); title('Acc Joint 2'); xlabel('Time [s]'); ylabel('dq2/dt [rad/s^2]'); grid on;
    subplot(3,4,7); plot(tempo(3,:),acc(3,2:end), 'LineWidth',1.5); title('Acc Joint 3'); xlabel('Time [s]'); ylabel('dq3/dt [rad/s^2]'); grid on;
    subplot(3,4,8); plot(tempo(4,:),acc(4,2:end), 'LineWidth',1.5); title('Acc Joint 4'); xlabel('Time [s]'); ylabel('dq4/dt [rad/s^2]'); grid on;
    % torque
    subplot(3,4,9); plot(tempo(1,:),totFcq(1,2:end), 'LineWidth',1.5); title('Torque Joint 1'); xlabel('Time [s]'); ylabel('T1 [N*m]'); grid on;
    subplot(3,4,10); plot(tempo(2,:),totFcq(2,2:end), 'LineWidth',1.5); title('Torque Joint 2'); xlabel('Time [s]'); ylabel('T2 [N*m]'); grid on;
    subplot(3,4,11); plot(tempo(3,:),totFcq(3,2:end), 'LineWidth',1.5); title('Torque Joint 3'); xlabel('Time [s]'); ylabel('T3 [N*m]'); grid on;
    subplot(3,4,12); plot(tempo(4,:),totFcq(4,2:end), 'LineWidth',1.5); title('Torque Joint 4'); xlabel('Time [s]'); ylabel('T4 [N*m]'); grid on;
    sgtitle('Joint Side');
    
    %torque totale giunti
    figure(10)
    grid on; 
    hold on; 
    xlabel('Time [s]'); 
    ylabel('T [N*m]');
    title('Torque');
    plot(tempo(1,:), totFcq(1,2:end), 'LineWidth', 1.5); 
    plot(tempo(2,:), totFcq(2,2:end), 'LineWidth', 1.5); 
    plot(tempo(3,:), totFcq(3,2:end), 'LineWidth', 1.5); 
    plot(tempo(4,:), totFcq(4,2:end), 'LineWidth', 1.5); 
    legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4');


    %torque totale motor
    figure(11)
    grid on; 
    hold on; 
    xlabel('Time [s]'); 
    ylabel('T [N*m]');
    title('Motor Torque');
    plot(tempo(1,:), torque_motor(1,2:end), 'LineWidth', 1.5); 
    plot(tempo(2,:), torque_motor(2,2:end), 'LineWidth', 1.5); 
    plot(tempo(3,:), torque_motor(3,2:end), 'LineWidth', 1.5); 
    plot(tempo(4,:), torque_motor(4,2:end), 'LineWidth', 1.5); 
    legend('Motor 1', 'Motor 2', 'Motor 3', 'Motor 4');

end

%% FUNZIONI PER INSEIRMENTI DATI
function [l1, l2, l3, l4, m1, m2, m3, m4, aMax, dMax, vMax, boxMass, gear1, gear2, gear3, gear4] = createFirstWindow()
    windowWidth = 900;
    windowHeight = 700;
    screenSize = get(0, 'ScreenSize');
    xPos = (screenSize(3) - windowWidth) / 2;
    yPos = (screenSize(4) - windowHeight) / 2;
    fig1 = uifigure('Name', 'Parameter insertion', 'Position', [xPos, yPos, windowWidth, windowHeight]);
    
    spacing = 35;  
    startY = 480;
    uilabel(fig1, 'Position', [30, startY, 200, 30], 'Text', 'First link length [m]:', 'FontSize', 12);
    l1 = uieditfield(fig1, 'numeric', 'Position', [240, startY, 100, 30]);

    uilabel(fig1, 'Position', [30, startY-spacing, 200, 30], 'Text', 'Second link length [m]:', 'FontSize', 12);
    l2 = uieditfield(fig1, 'numeric', 'Position', [240, startY-spacing, 100, 30]);

    uilabel(fig1, 'Position', [30, startY-2*spacing, 200, 30], 'Text', 'Third link length[m]:', 'FontSize', 12);
    l3 = uieditfield(fig1, 'numeric', 'Position', [240, startY-2*spacing, 100, 30]);

    uilabel(fig1, 'Position', [30, startY-3*spacing, 200, 30], 'Text', 'Fourth link length [m]:', 'FontSize', 12);
    l4 = uieditfield(fig1, 'numeric', 'Position', [240, startY-3*spacing, 100, 30]);

    uilabel(fig1, 'Position', [30, startY-4*spacing, 200, 30], 'Text', 'First link weight [kg]:', 'FontSize', 12);
    m1 = uieditfield(fig1, 'numeric', 'Position', [240, startY-4*spacing, 100, 30]);

    uilabel(fig1, 'Position', [30, startY-5*spacing, 200, 30], 'Text', 'Second link weight [kg]:', 'FontSize', 12);
    m2 = uieditfield(fig1, 'numeric', 'Position', [240, startY-5*spacing, 100, 30]);

    uilabel(fig1, 'Position', [30, startY-6*spacing, 200, 30], 'Text', 'Third link weight [kg]:', 'FontSize', 12);
    m3 = uieditfield(fig1, 'numeric', 'Position', [240, startY-6*spacing, 100, 30]);

    uilabel(fig1, 'Position', [30, startY-7*spacing, 200, 30], 'Text', 'Fourth link weight [kg]:', 'FontSize', 12);
    m4 = uieditfield(fig1, 'numeric', 'Position', [240, startY-7*spacing, 100, 30]);

    uilabel(fig1, 'Position', [30, startY-8*spacing, 200, 30], 'Text', 'Weight box [kg]:', 'FontSize', 12);
    boxMass = uieditfield(fig1, 'numeric', 'Position', [240, startY-8*spacing, 100, 30]);

    uilabel(fig1, 'Position', [30, startY-10*spacing, 100, 30], 'Text', 'Acc. Max [m/s²]:', 'FontSize', 12);
    aMax = uieditfield(fig1, 'numeric', 'Position', [130, startY-10*spacing, 70, 30]);

    uilabel(fig1, 'Position', [210, startY-10*spacing, 100, 30], 'Text', 'Dec. Max [m/s²]:', 'FontSize', 12);
    dMax = uieditfield(fig1, 'numeric', 'Position', [310, startY-10*spacing, 70, 30]);

    uilabel(fig1, 'Position', [390, startY-10*spacing, 100, 30], 'Text', 'Velocity Max [m/s]:', 'FontSize', 12);
    vMax = uieditfield(fig1, 'numeric', 'Position', [490, startY-10*spacing, 70, 30]);

    uilabel(fig1, 'Position', [30, startY-11*spacing, 100, 30], 'Text', 'Trasmissions: ', 'FontSize', 12);

    uilabel(fig1, 'Position', [30, startY-12*spacing, 100, 30], 'Text', 'Joint 1:', 'FontSize', 12);
    gear1 = uieditfield(fig1, 'numeric', 'Position', [80, startY-12*spacing, 70, 30]);

    uilabel(fig1, 'Position', [160, startY-12*spacing, 100, 30], 'Text', 'Joint 2:', 'FontSize', 12);
    gear2 = uieditfield(fig1, 'numeric', 'Position', [210, startY-12*spacing, 70, 30]);

    uilabel(fig1, 'Position', [290, startY-12*spacing, 100, 30], 'Text', 'Joint 3:', 'FontSize', 12);
    gear3= uieditfield(fig1, 'numeric', 'Position', [340, startY-12*spacing, 70, 30]);

    uilabel(fig1, 'Position', [420, startY-12*spacing, 100, 30], 'Text', 'Joint 4:', 'FontSize', 12);
    gear4 = uieditfield(fig1, 'numeric', 'Position', [470, startY-12*spacing, 70, 30]);

    uibutton(fig1, 'push', 'Position', [250, 20, 100, 30], 'Text', 'Insert', ...
        'ButtonPushedFcn', @(btn, event) onConfirmFirstWindow(fig1, l1,l2, l3, l4, m1, m2, m3, m4, ...
                                                              aMax, dMax, vMax, boxMass, ...
                                                              gear1, gear2, gear3, gear4));
    waitfor(fig1); 
end


function onConfirmFirstWindow(fig1, l1, l2, l3, l4, m1, m2, m3, m4, ...
                               aMax, dMax, vMax, boxMass, ...
                               gear1, gear2, gear3, gear4)

    l1 = l1.Value; l2 = l2.Value; l3 = l3.Value; l4 = l4.Value;
    m1 = m1.Value; m2 = m2.Value; m3 = m3.Value; m4 = m4.Value;
    aMax = aMax.Value; dMax = dMax.Value; vMax = vMax.Value; boxMass = boxMass.Value;
    gear1 = gear1.Value; gear2 = gear2.Value; gear3 = gear3.Value; gear4 = gear4.Value;

    close(fig1);
    
    % salva in workspace
    assignin('base', 'l1', l1); assignin('base', 'l2', l2);assignin('base', 'l3', l3); assignin('base', 'l4', l4);
    assignin('base', 'm1', m1); assignin('base', 'm2', m2);assignin('base', 'm3', m3); assignin('base', 'm4', m4); 
    assignin('base', 'aMax', aMax); assignin('base', 'dMax', dMax); assignin('base', 'vMax', vMax); assignin('base', 'boxMass', boxMass);
    assignin('base', 'gear1', gear1); assignin('base', 'gear2', gear2); assignin('base', 'gear3', gear3);  assignin('base', 'gear4', gear4);
end


function [pickX, pickY, pickZ, pickPhi, placeX, placeY, placePhi, shelfDropdown] = createSecondWindow(rMin, rMax, zMax,l1,l2,l3,l4, homePosition)
    windowWidth = 1200;
    windowHeight = 550;
    screenSize = get(0, 'ScreenSize');
    xPos = (screenSize(3) - windowWidth) / 2;
    yPos = (screenSize(4) - windowHeight) / 2;
    fig2 = uifigure('Name', 'Position insertion', 'Position', [xPos, yPos, windowWidth, windowHeight]);
    uilabel(fig2, ...
        'Position', [30, 320, 850, 60], ...
        'Text', sprintf(['The allowed range for the pick and place point must be contained in the circle represented on the right.\n' ...
        'This circle shows the maximum circumference at which the mass can be placed (c=l2+l3+l4)\n' ...
        'also enter the angle with which you want to direct the endeffector (ex: pi/2= fork directed upwards)'], rMin, rMax, zMax), ...
        'FontSize', 12, ...
        'HorizontalAlignment', 'left'); 

    spacing = 35;

    % Pick
    uilabel(fig2, 'Position', [30, 280, 200, 30], 'Text', 'Point of Pick:', 'FontSize', 12);
    pickX = uieditfield(fig2, 'numeric', 'Position', [240, 280, 50, 30], 'Placeholder', 'x');
    uilabel(fig2, 'Position', [295, 280, 20, 30], 'Text', '[m]', 'FontSize', 12);
    pickY = uieditfield(fig2, 'numeric', 'Position', [320, 280, 50, 30], 'Placeholder', 'y');
    uilabel(fig2, 'Position', [375, 280, 20, 30], 'Text', '[m]', 'FontSize', 12);
    pickZ = uieditfield(fig2, 'numeric', 'Position', [400, 280, 50, 30], 'Placeholder', 'z');
    uilabel(fig2, 'Position', [455, 280, 20, 30], 'Text', '[m]', 'FontSize', 12);
    pickPhi = uidropdown(fig2, 'Position', [480, 280, 50, 30], ...
                         'Items', {'0', 'pi', 'pi/2', '-pi/2'}, 'Value', '0', 'FontSize', 12);

    % Place
    uilabel(fig2, 'Position', [30, 280-spacing, 200, 30], 'Text', 'Point of Place:', 'FontSize', 12);
    placeX = uieditfield(fig2, 'numeric', 'Position', [240, 280-spacing, 50, 30], 'Placeholder', 'x');
    uilabel(fig2, 'Position', [295, 280-spacing, 20, 30], 'Text', '[m]', 'FontSize', 12);
    placeY = uieditfield(fig2, 'numeric', 'Position', [320, 280-spacing, 50, 30], 'Placeholder', 'y');
    uilabel(fig2, 'Position', [375, 280-spacing, 20, 30], 'Text', '[m]', 'FontSize', 12);
    placePhi = uidropdown(fig2, 'Position', [480, 280-spacing, 50, 30], ...
                          'Items', {'0', 'pi', 'pi/2', '-pi/2'}, 'Value', '0', 'FontSize', 12);

    % Ripiano di Place
    uilabel(fig2, 'Position', [30, 180, 200, 30], 'Text', 'Shelf of Place:', 'FontSize', 12);
    shelfDropdown = uidropdown(fig2, 'Position', [240, 180, 100, 30], ...
        'Items', {'1', '2', '3'}, 'FontSize', 12, 'Value', '1');
    margin = 0.5;

   ax = uiaxes(fig2, 'Position', [760, 200, 300, 300]); 
    title(ax, 'Area permitted for the Pick/Place');
    xlabel(ax, 'X (m)');
    ylabel(ax, 'Y (m)');
    axis(ax, 'equal');
    grid(ax, 'on');
    
    % Definizione raggi
    radius = l2 + l3 - l4;  % Raggio minore
    rMax = l2 + l3 + l4;    % Raggio massimo
    
    
    theta = linspace(0, 2*pi, 100);
    
    
    fill(ax, rMax * cos(theta), rMax * sin(theta), 'b', 'FaceAlpha', 0.1, 'EdgeColor', 'b', 'LineWidth', 1.5); % Cerchio con raggio massimo
    hold(ax, 'on');
    fill(ax, radius * cos(theta), radius * sin(theta), 'b', 'FaceAlpha', 0, 'EdgeColor', 'k', 'LineWidth', 0.5); % Cerchio con raggio minore
    margin = 0.1 * rMax; 
    xlim(ax, [-rMax-margin, rMax+margin]);
    ylim(ax, [-rMax-margin, rMax+margin]);
    
    hold(ax, 'on');
    scatter(ax, 0, 0, 50, 'r', 'filled'); % centro
    text(ax, 0, 0, ' Robot', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 10, 'Color', 'r');
    
    scatter(ax, homePosition(2), homePosition(3), 50, 'g', 'filled'); % home
    text(ax, homePosition(2), homePosition(3), ' Home Position', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', 'FontSize', 8, 'Color', 'r');
    hold(ax, 'off');

    uibutton(fig2, 'push', 'Position', [300, 30, 200, 30], 'Text', 'Insert Position', ...
        'ButtonPushedFcn', @(btn, event) onConfirmSecondWindow(fig2, pickX, pickY, pickZ, pickPhi, ...
                                                               placeX, placeY, placePhi, ...
                                                               shelfDropdown, l1,l2,l3,l4));
    waitfor(fig2);
end

function onConfirmSecondWindow(fig2, pickX, pickY, pickZ, pickPhi, placeX, placeY, placePhi, shelfDropdown,l1,l2,l3,l4)
    pickXVal = pickX.Value;pickYVal = pickY.Value;pickZVal = pickZ.Value;pickPhiVal = getPhiValue(pickPhi.Value);  
    placePhiVal = getPhiValue(placePhi.Value); placeXVal = placeX.Value; placeYVal = placeY.Value; shelfVal = shelfDropdown.Value;

    try
    SCARAinv([pickXVal, pickYVal, pickZVal, pickPhiVal]', [l1, l2, l3, l4]', 1); 
    catch e
        uialert(fig2, sprintf('Il punto di Pick non è nel range consentito.'),'Errore');
        return;
    end
    
    try
        SCARAinv([placeXVal, placeYVal, 0.2, placePhiVal]', [l1, l2, l3, l4]', 1); 
    catch e
        uialert(fig2, sprintf('Il punto di Place non è nel range consentito.'), 'Errore'); 
        return;      
    end

    assignin('base', 'pickX', pickXVal); assignin('base', 'pickY', pickYVal); assignin('base', 'pickZ', pickZVal); assignin('base', 'pickPhi', pickPhiVal); 
    assignin('base', 'placeX', placeXVal); assignin('base', 'placeY', placeYVal); assignin('base', 'placePhi', placePhiVal); assignin('base', 'shelf', shelfVal);    
    close(fig2);
end

% funzione di conversione angoli elenco a discesa
function phiVal = getPhiValue(phiStr)
    switch phiStr
        case '0'
            phiVal = 0;
        case 'pi'
            phiVal = pi;
        case 'pi/2'
            phiVal = pi / 2;
        case '-pi/2'
            phiVal = -pi / 2;
        otherwise
            phiVal = 0; 
    end
end
