% Análise do desempenho do controlador: erro de rastreamento por junta,
% torque aplicado e diferença RMS em relação ao torque ideal (feedforward).
function analisarControle(qTraj, qSim, tauTraj, tauSim, numJuntas)
    arguments
        qTraj double
        qSim double
        tauTraj double
        tauSim double
        numJuntas (1,1) double {mustBePositive}
    end

    nomesJuntas = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6'};
    erro = qTraj - qSim;

    fprintf('\n--- Análise do Controle PD + Compensação de Gravidade ---\n');
    for junta = 1:numJuntas
        erroMax = max(abs(erro(junta,:)), [], 'omitnan');
        erroRms = rms(erro(junta,:), 'omitnan');
        torqueMaxAplicado = max(abs(tauSim(junta,:)), [], 'omitnan');
        fprintf('%s: erro máx = %7.4f rad | erro RMS = %7.4f rad | torque aplicado máx = %6.2f Nm\n', ...
            nomesJuntas{junta}, erroMax, erroRms, torqueMaxAplicado);
    end

    difTorque = tauTraj - tauSim;
    rmsPorJunta = zeros(numJuntas, 1);
    for junta = 1:numJuntas
        rmsPorJunta(junta) = rms(difTorque(junta,:), 'omitnan');
    end
    fprintf('Diferença RMS (torque ideal - torque do controlador): %.3f Nm (média entre juntas)\n', ...
        mean(rmsPorJunta, 'omitnan'));
    fprintf('-----------------------------------------------------------\n\n');
end
