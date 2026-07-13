% Dinâmica inversa: torque "ideal" (feedforward completo) que o robô exige
% para executar exatamente a trajetória planejada (q, qd, qdd). Imprime no
% console a análise por junta (máximo, instante do máximo e RMS).
function tauTraj = calcularTorques(robo, qTraj, qdTraj, qddTraj, tempo)
    arguments
        robo (1,1) rigidBodyTree
        qTraj double
        qdTraj double
        qddTraj double
        tempo (1,:) double
    end

    numJuntas = size(qTraj, 1);
    numPassos = size(qTraj, 2);
    tauTraj = zeros(numJuntas, numPassos);

    for passo = 1:numPassos
        tauTraj(:,passo) = inverseDynamics(robo, qTraj(:,passo)', qdTraj(:,passo)', qddTraj(:,passo)');
    end

    exibirAnaliseTorques(tauTraj, tempo, numJuntas);
end

% Análise textual dos torques calculados pela dinâmica inversa
function exibirAnaliseTorques(tauTraj, tempo, numJuntas)
    fprintf('\n--- Análise dos Torques (Dinâmica Inversa) ---\n');
    nomesJuntas = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6'};
    for junta = 1:numJuntas
        torqueMax = max(abs(tauTraj(junta,:)));
        torqueRms = rms(tauTraj(junta,:));
        [~, idxMaximo] = max(abs(tauTraj(junta,:)));
        fprintf('%s: torque máx = %6.2f Nm (em t = %5.2f s)  |  RMS = %6.2f Nm\n', ...
            nomesJuntas{junta}, torqueMax, tempo(idxMaximo), torqueRms);
    end
    fprintf('-----------------------------------------------\n\n');
end
