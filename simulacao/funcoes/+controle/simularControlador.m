% Simula o robô em malha fechada com a lei de controle escolhida:
%   'pdGravidade'     tau = Kp.*e + Kd.*ed + G(q)   (PD + feedforward de gravidade)
%   'torqueComputado' tau = ID(q, qd, qddRef + 2*zeta*wn*ed + wn^2*e)
% Integração em passo fixo (Euler semi-implícito), com dois sub-passos por
% amostra. O torque de comando é saturado nos limites físicos de cada junta.
function [qSim, qdSim, tauSim] = simularControlador(robo, tempo, qTraj, qdTraj, qddTraj, Kp, Kd, wn, zeta, tauLimite, modo)
    arguments
        robo (1,1) rigidBodyTree
        tempo (1,:) double
        qTraj double
        qdTraj double
        qddTraj double
        Kp (:,1) double
        Kd (:,1) double
        wn (1,1) double {mustBePositive}
        zeta (1,1) double {mustBePositive}
        tauLimite (:,1) double
        modo (1,:) char {mustBeMember(modo, {'pdGravidade','torqueComputado'})}
    end

    numJuntas = size(qTraj, 1);
    numPassos = numel(tempo);
    dt = tempo(2) - tempo(1);
    % Passo de integração interno limitado a ~0.02 s para estabilidade: se o dt
    % da grade for grande, subdivide-se em mais subpassos (com DT=0.04 dá 2).
    numSubPassos = max(2, ceil(dt / 0.02));
    hSub = dt / numSubPassos;

    qSim   = nan(numJuntas, numPassos);
    qdSim  = nan(numJuntas, numPassos);
    tauSim = nan(numJuntas, numPassos);
    qSim(:,1)  = qTraj(:,1);
    qdSim(:,1) = qdTraj(:,1);

    for passo = 1:numPassos-1
        q = qSim(:,passo);
        qd = qdSim(:,passo);

        for sub = 1:numSubPassos
            erro = qTraj(:,passo) - q;
            erroVel = qdTraj(:,passo) - qd;
            if strcmp(modo, 'pdGravidade')
                torqueGravidade = gravityTorque(robo, q')';
                tau = Kp.*erro + Kd.*erroVel + torqueGravidade;
            else
                acelDesejada = qddTraj(:,passo) + 2*zeta*wn*erroVel + wn^2*erro;
                tau = inverseDynamics(robo, q', qd', acelDesejada')';
            end
            tau = max(min(tau, tauLimite), -tauLimite);
            qdd = forwardDynamics(robo, q', qd', tau')';
            qd = qd + hSub*qdd;
            q = q + hSub*qd;
        end

        qSim(:,passo+1)  = q;
        qdSim(:,passo+1) = qd;
        tauSim(:,passo) = tau;
    end
    tauSim(:,end) = tauSim(:,end-1);
end
