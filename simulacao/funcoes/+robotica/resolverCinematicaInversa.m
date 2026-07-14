% Resolve a cinemática inversa ponto a ponto (usando a solução anterior como
% estimativa inicial, para manter a continuidade do movimento) e valida cada
% solução por cinemática direta: erro de posição, erro de orientação e menor
% valor singular do Jacobiano (proximidade de singularidade).
function [qTraj, caminhoEfetuador, erroPos, erroOri, sigmaMin] = resolverCinematicaInversa(robo, posCart, rFerramenta, pesosIk)
    arguments
        robo (1,1) rigidBodyTree
        posCart (3,:) double
        rFerramenta (3,3) double
        pesosIk (1,6) double
    end

    numJuntas = numel(homeConfiguration(robo));
    numPassos = size(posCart, 2);

    solverIK = inverseKinematics('RigidBodyTree', robo);
    estimativaInicial = homeConfiguration(robo);
    qTraj = zeros(numJuntas, numPassos);

    for passo = 1:numPassos
        poseAlvo = trvec2tform(posCart(:,passo)') * rotm2tform(rFerramenta);
        [qSolucao, ~] = solverIK('paint_tcp', poseAlvo, pesosIk, estimativaInicial);
        qTraj(:,passo) = qSolucao';
        estimativaInicial = qSolucao;
    end

    % Reconstrução por cinemática direta: erros e Jacobiano
    caminhoEfetuador = zeros(3, numPassos);
    erroPos = zeros(1, numPassos);
    erroOri = zeros(1, numPassos);
    sigmaMin = zeros(1, numPassos);
    for passo = 1:numPassos
        poseAtual = getTransform(robo, qTraj(:,passo)', 'paint_tcp');
        caminhoEfetuador(:,passo) = poseAtual(1:3,4);
        erroPos(passo) = norm(poseAtual(1:3,4) - posCart(:,passo));
        erroRotacao = rotm2axang(rFerramenta * poseAtual(1:3,1:3)');
        erroOri(passo) = abs(erroRotacao(4));
        jacobiano = geometricJacobian(robo, qTraj(:,passo)', 'paint_tcp');
        valoresSingulares = svd(jacobiano);
        sigmaMin(passo) = min(valoresSingulares);
    end
end
