%% Simulação do Stäubli TX90 — ES827 (Robótica Industrial)
%  Pintura da bandeira do Brasil: troca de ferramentas (cores), trajetória
%  segmentada com perfil LSPB, cinemática inversa, dinâmica inversa (torques)
%  e controle PD + compensação de gravidade / torque computado.
%
%  Requisitos: Robotics System Toolbox.
%  Executar a partir da pasta simulacao/ (o URDF é carregado por caminho relativo).
clearvars; close all; clc;

%% Parâmetros
% --- Amostragem e limites cartesianos da trajetória ---
DT             = 0.04;   % passo de amostragem [s]
VEL_PINTURA    = 0.12;   % velocidade máxima com spray ligado [m/s]
ACEL_PINTURA   = 0.30;   % aceleração máxima com spray ligado [m/s^2]
VEL_TRANSICAO  = 0.35;   % velocidade máxima em movimento livre [m/s]
ACEL_TRANSICAO = 0.60;   % aceleração máxima em movimento livre [m/s^2]

% --- Ferramenta (aplicador de tinta: 15 cm e 3 kg) ---
COMPRIMENTO_TCP         = 0.15;             % flange até a ponta [m]
MASSA_FERRAMENTA        = 3.0;              % [kg]
CENTRO_MASSA_FERRAMENTA = [0 0 -0.075];     % no referencial do TCP [m]
INERCIA_FERRAMENTA      = [0.006825 0.006825 0.0024 0 0 0]; % [kg*m^2]

% --- Limites físicos por junta (manual Stäubli / URDF) ---
LIMITE_VELOCIDADE = deg2rad([400; 400; 430; 540; 475; 760]); % [rad/s]
LIMITE_TORQUE     = [318; 166; 76; 34; 29; 11];              % [N*m]

% --- Cinemática inversa ---
PESOS_IK          = [0.5 0.5 0.5 1 1 1];
TOL_POSICAO_IK    = 2e-4;  % [m]
TOL_ORIENTACAO_IK = 2e-3;  % [rad]
R_FERRAMENTA = [ 0  0  1;   % orientação frontal (+X) mantida em toda a operação
                 0  1  0;
                -1  0  0 ];

% --- Controle em malha fechada ---
WN   = 5;    % frequência natural desejada [rad/s]
ZETA = 1.0;  % fator de amortecimento (1 = criticamente amortecido)

% --- Verificação da área de trabalho ---
NUM_AMOSTRAS_WORKSPACE = 50000;

% --- Geometria da bandeira (plano X constante) e estação de troca ---
CENTRO_BANDEIRA   = [0.65; 0.00; 0.45];  % [m]
PONTO_ESTACAO     = [0.50; 0.60; 0.20];  % estação de troca de cores [m]
LARGURA_RETANGULO = 0.70;   ALTURA_RETANGULO = 0.50;   % [m]
LARGURA_LOSANGO   = 0.581;  ALTURA_LOSANGO   = 0.381;  % [m]
RAIO_CIRCULO      = 0.1225;                            % [m]
CENTRO_ARCO       = [0.65; -0.07; 0.20]; RAIO_ARCO = 0.28875; % faixa branca [m]

% --- Códigos de cor do rastro (índices no colormap CORES_RASTRO) ---
COR_VERDE = 1; COR_TRANSICAO = 2; COR_AMARELO = 3; COR_AZUL = 4; COR_BRANCO = 5;
CORES_RASTRO = [ ...
    0.00 0.80 0.30;    % verde    (retângulo)
    0.40 0.40 0.45;    % cinza    (transição / movimento livre)
    1.00 0.85 0.00;    % amarelo  (losango)
    0.00 0.50 1.00;    % azul     (círculo)
    1.00 1.00 1.00 ];  % branco   (arco)

%% Modelo do robô e da ferramenta
robo = importrobot('tx90.urdf');
robo.DataFormat = 'row';
robo.Gravity = [0 0 -9.81];   % gravidade padrão [m/s^2]
numJuntas = numel(homeConfiguration(robo));

aplicador = rigidBody('paint_tcp');
juntaAplicador = rigidBodyJoint('paint_tcp_fixed', 'fixed');
setFixedTransform(juntaAplicador, trvec2tform([0 0 COMPRIMENTO_TCP]));
aplicador.Joint = juntaAplicador;
aplicador.Mass = MASSA_FERRAMENTA;
aplicador.CenterOfMass = CENTRO_MASSA_FERRAMENTA;
aplicador.Inertia = INERCIA_FERRAMENTA;
addBody(robo, aplicador, 'tool0');

%% Waypoints cartesianos das formas da bandeira
% Retângulo (contorno externo, verde)
metadeLargRet = LARGURA_RETANGULO/2;  metadeAltRet = ALTURA_RETANGULO/2;
pontosRetangulo = CENTRO_BANDEIRA + [ ...
    0              0             0              0             0;
   -metadeLargRet  metadeLargRet metadeLargRet -metadeLargRet -metadeLargRet;
    metadeAltRet   metadeAltRet -metadeAltRet  -metadeAltRet   metadeAltRet ];

% Losango (amarelo)
metadeLargLos = LARGURA_LOSANGO/2;  metadeAltLos = ALTURA_LOSANGO/2;
pontosLosango = CENTRO_BANDEIRA + [ ...
    0             0              0             0              0;
    0             metadeLargLos  0            -metadeLargLos  0;
    metadeAltLos  0             -metadeAltLos  0              metadeAltLos ];

% Círculo (azul)
anguloCirculo = linspace(pi/2, -3*pi/2, 40);
pontosCirculo = CENTRO_BANDEIRA + [ zeros(1,40);
    RAIO_CIRCULO*cos(anguloCirculo);
    RAIO_CIRCULO*sin(anguloCirculo) ];

% Arco (faixa branca) — ângulos inicial e final derivados da geometria
anguloInicialArco = atan2(0.28483, -0.11745 + 0.07);
anguloFinalArco   = atan2(0.21877,  0.11845 + 0.07);
anguloArco = linspace(anguloInicialArco, anguloFinalArco, 40);
pontosArco = CENTRO_ARCO + [ zeros(1,40);
    RAIO_ARCO*cos(anguloArco);
    RAIO_ARCO*sin(anguloArco) ];

%% Construção da trajetória (idas e vindas à estação)
% Estação -> retângulo -> estação
[pT1, cT1, vT1, aT1] = interpolarTransicao(PONTO_ESTACAO, pontosRetangulo(:,1), VEL_TRANSICAO, ACEL_TRANSICAO, DT, COR_TRANSICAO);
[pR,  cR,  vR,  aR ] = interpolarCartesianaConstante(pontosRetangulo, VEL_PINTURA, ACEL_PINTURA, DT, COR_VERDE);
[pT2, cT2, vT2, aT2] = interpolarTransicao(pontosRetangulo(:,end), PONTO_ESTACAO, VEL_TRANSICAO, ACEL_TRANSICAO, DT, COR_TRANSICAO);

% Estação -> losango -> estação
[pT3, cT3, vT3, aT3] = interpolarTransicao(PONTO_ESTACAO, pontosLosango(:,1), VEL_TRANSICAO, ACEL_TRANSICAO, DT, COR_TRANSICAO);
[pL,  cL,  vL,  aL ] = interpolarCartesianaConstante(pontosLosango, VEL_PINTURA, ACEL_PINTURA, DT, COR_AMARELO);
[pT4, cT4, vT4, aT4] = interpolarTransicao(pontosLosango(:,end), PONTO_ESTACAO, VEL_TRANSICAO, ACEL_TRANSICAO, DT, COR_TRANSICAO);

% Estação -> círculo -> estação
[pT5, cT5, vT5, aT5] = interpolarTransicao(PONTO_ESTACAO, pontosCirculo(:,1), VEL_TRANSICAO, ACEL_TRANSICAO, DT, COR_TRANSICAO);
[pC,  cC,  vC,  aC ] = interpolarArco(CENTRO_BANDEIRA, RAIO_CIRCULO, pi/2, -3*pi/2, VEL_PINTURA, ACEL_PINTURA, DT, COR_AZUL);
[pT6, cT6, vT6, aT6] = interpolarTransicao(pontosCirculo(:,1), PONTO_ESTACAO, VEL_TRANSICAO, ACEL_TRANSICAO, DT, COR_TRANSICAO);

% Estação -> arco -> estação (guarda a ferramenta)
[pT7, cT7, vT7, aT7] = interpolarTransicao(PONTO_ESTACAO, pontosArco(:,1), VEL_TRANSICAO, ACEL_TRANSICAO, DT, COR_TRANSICAO);
[pA,  cA,  vA,  aA ] = interpolarArco(CENTRO_ARCO, RAIO_ARCO, anguloInicialArco, anguloFinalArco, VEL_PINTURA, ACEL_PINTURA, DT, COR_BRANCO);
[pT8, cT8, vT8, aT8] = interpolarTransicao(pontosArco(:,end), PONTO_ESTACAO, VEL_TRANSICAO, ACEL_TRANSICAO, DT, COR_TRANSICAO);

% Concatena todos os trechos (removendo o 1º ponto dos blocos subsequentes)
posCart  = [pT1, pR(:,2:end), pT2(:,2:end), pT3(:,2:end), pL(:,2:end), ...
            pT4(:,2:end), pT5(:,2:end), pC(:,2:end), pT6(:,2:end), ...
            pT7(:,2:end), pA(:,2:end), pT8(:,2:end)];
corTraj  = [cT1, cR(2:end), cT2(2:end), cT3(2:end), cL(2:end), ...
            cT4(2:end), cT5(2:end), cC(2:end), cT6(2:end), ...
            cT7(2:end), cA(2:end), cT8(2:end)];
velCart  = [vT1, vR(2:end), vT2(2:end), vT3(2:end), vL(2:end), ...
            vT4(2:end), vT5(2:end), vC(2:end), vT6(2:end), ...
            vT7(2:end), vA(2:end), vT8(2:end)];
acelCart = [aT1, aR(2:end), aT2(2:end), aT3(2:end), aL(2:end), ...
            aT4(2:end), aT5(2:end), aC(2:end), aT6(2:end), ...
            aT7(2:end), aA(2:end), aT8(2:end)];

numPassos = size(posCart, 2);
tempo = (0:numPassos-1) * DT;

fprintf('Trajetória: %d pontos, duração %.2f s.\n', numPassos, tempo(end));
fprintf('Velocidade cartesiana comandada máxima: %.4f m/s.\n', max(velCart));
fprintf('Aceleração cartesiana comandada máxima: %.4f m/s^2.\n', max(abs(acelCart)));

%% Verificação da área de trabalho (workspace)
% Amostra o espaço de juntas (dentro dos limites do URDF) para estimar a
% nuvem de pontos alcançáveis pela ponta do aplicador, e verifica se todos
% os pontos da trajetória desejada (incluindo a estação) caem dentro dela.
fprintf('Verificando área de trabalho do manipulador...\n');
nuvemWorkspace = amostrarAreaTrabalho(robo, NUM_AMOSTRAS_WORKSPACE);

pontosDesejados = [pontosRetangulo, pontosLosango, pontosCirculo, pontosArco, PONTO_ESTACAO];
verificarAlcancePontos(pontosDesejados, nuvemWorkspace);
plotarAreaTrabalho(nuvemWorkspace, pontosDesejados);

%% Cinemática inversa (IK)
fprintf('Calculando cinemática inversa. Aguarde...\n');
solverIK = inverseKinematics('RigidBodyTree', robo);
estimativaInicial = homeConfiguration(robo);
qTraj = zeros(numJuntas, numPassos);
erroPosIK = zeros(1, numPassos);
erroOriIK = zeros(1, numPassos);
sigmaMin = zeros(1, numPassos);

estadoAvisos = warning('off', 'all');
for passo = 1:numPassos
    poseAlvo = trvec2tform(posCart(:,passo)') * rotm2tform(R_FERRAMENTA);
    [qSolucao, ~] = solverIK('paint_tcp', poseAlvo, PESOS_IK, estimativaInicial);
    qTraj(:,passo) = qSolucao';
    estimativaInicial = qSolucao;
end
warning(estadoAvisos);

% Reconstrução por cinemática direta: erros de posição/orientação e Jacobiano
caminhoEfetuador = zeros(3, numPassos);
for passo = 1:numPassos
    poseAtual = getTransform(robo, qTraj(:,passo)', 'paint_tcp');
    caminhoEfetuador(:,passo) = poseAtual(1:3,4);
    erroPosIK(passo) = norm(poseAtual(1:3,4) - posCart(:,passo));
    erroRotacao = rotm2axang(R_FERRAMENTA * poseAtual(1:3,1:3)');
    erroOriIK(passo) = abs(erroRotacao(4));
    jacobiano = geometricJacobian(robo, qTraj(:,passo)', 'paint_tcp');
    valoresSingulares = svd(jacobiano);
    sigmaMin(passo) = min(valoresSingulares);
end

if max(erroPosIK) > TOL_POSICAO_IK || max(erroOriIK) > TOL_ORIENTACAO_IK
    error('A cinemática inversa não atingiu a tolerância esperada.');
end

% Velocidades e acelerações articulares por diferenças finitas
qdTraj = zeros(size(qTraj));
qddTraj = zeros(size(qTraj));
for junta = 1:numJuntas
    qdTraj(junta,:) = gradient(qTraj(junta,:), DT);
    qddTraj(junta,:) = gradient(qdTraj(junta,:), DT);
end

if any(max(abs(qdTraj), [], 2) > LIMITE_VELOCIDADE)
    error('A trajetória ultrapassa o limite de velocidade de uma junta.');
end

%% Dinâmica: cálculo dos torques necessários (a partir do URDF)
% Torque "ideal" (feedforward completo) que a dinâmica inversa do robô
% exige para executar exatamente a trajetória planejada (q, qd, qdd).
fprintf('Calculando torques via dinâmica inversa (modelo URDF)...\n');
tauTraj = calcularTorques(robo, qTraj, qdTraj, qddTraj);

analisarTorques(tauTraj, tempo, numJuntas);

torqueMaximo = max(abs(tauTraj), [], 2);
if any(torqueMaximo > LIMITE_TORQUE)
    error('A trajetória ultrapassa o limite de torque de uma junta.');
end

%% Controle de torque com compensação de gravidade (feedforward)
% Lei de controle:  tau = Kp*(qRef - q) + Kd*(qdRef - qd) + G(q)
% onde G(q) é o torque gravitacional calculado pelo modelo URDF (feedforward),
% e o termo PD realimenta o erro de posição/velocidade em malha fechada.
fprintf('Simulando controle PD + compensação de gravidade (feedforward)...\n');

% Os ganhos são calculados a partir da matriz de massa M(q0) do próprio
% robô, e não fixados arbitrariamente. Isso é necessário porque juntas com
% pouca inércia (ex.: punho, J4-J6) ficam instáveis com ganhos altos demais
% (frequência natural de malha fechada incompatível com o passo de
% integração), enquanto juntas de base (J1-J3, mais massa) toleram e
% precisam de ganhos maiores.
M0 = massMatrix(robo, qTraj(:,1)');
Kp = (WN^2) * diag(M0);
Kd = (2*ZETA*WN) * diag(M0);

[qSimPD, ~, tauSimPD] = controlePDGravidade(robo, tempo, qTraj, qdTraj, Kp, Kd, LIMITE_TORQUE);
[qSimTC, ~, ~] = controleTorqueComputado(robo, tempo, qTraj, qdTraj, qddTraj, WN, ZETA, LIMITE_TORQUE);

analisarControle(qTraj, qSimPD, tauTraj, tauSimPD, numJuntas);

erroRmsPD = sqrt(mean((qTraj - qSimPD).^2, 2));
erroRmsTC = sqrt(mean((qTraj - qSimTC).^2, 2));

fprintf('\n--- RESULTADOS VALIDADOS ---\n');
fprintf('Duração: %.2f s | pontos: %d\n', tempo(end), numPassos);
fprintf('Erro IK máximo: %.3f mm | orientação: %.3e rad\n', 1e3*max(erroPosIK), max(erroOriIK));
fprintf('Sigma mínimo do Jacobiano: %.3e\n', min(sigmaMin));
fprintf('Torque máximo por junta [Nm]: %s\n', mat2str(torqueMaximo', 4));
fprintf('RMS erro PD+G [rad]: %s\n', mat2str(erroRmsPD', 4));
fprintf('RMS erro torque computado [rad]: %s\n', mat2str(erroRmsTC', 4));

%% Animação: setup da figura
figAnim = figure('Name', 'TX90 – Troca de Ferramentas', ...
    'NumberTitle', 'off', 'Color', [0.07 0.07 0.10], 'Position', [40 40 1200 730]);

eixoAnim = axes('Parent', figAnim, 'Color', [0.07 0.07 0.10], 'XColor', [0.55 0.65 0.75], ...
    'YColor', [0.55 0.65 0.75], 'ZColor', [0.55 0.65 0.75], ...
    'GridColor', [0.20 0.25 0.30], 'GridAlpha', 0.55, 'FontSize', 10);

hold(eixoAnim, 'on'); grid(eixoAnim, 'on'); view(eixoAnim, 140, 22); axis(eixoAnim, 'equal');
xlim(eixoAnim, [-0.10, 1.10]); ylim(eixoAnim, [-0.80, 0.80]); zlim(eixoAnim, [0.00, 1.00]);

camlight(eixoAnim, 'headlight'); camlight(eixoAnim, 'right'); lighting(eixoAnim, 'gouraud');

colormap(eixoAnim, CORES_RASTRO);
set(eixoAnim, 'CLim', [1 size(CORES_RASTRO,1)]);

% Marcador da estação de troca
plot3(eixoAnim, PONTO_ESTACAO(1), PONTO_ESTACAO(2), PONTO_ESTACAO(3), 's', 'MarkerSize', 16, ...
    'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
text(eixoAnim, PONTO_ESTACAO(1), PONTO_ESTACAO(2), PONTO_ESTACAO(3)+0.12, 'Estação de Cores', ...
    'Color', [0.8 0.8 0.9], 'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');

% Rastro da pintura e marcador do TCP
rastro = scatter3(eixoAnim, NaN, NaN, NaN, 18, NaN, 'filled', 'MarkerEdgeColor', 'none');
marcadorTCP = plot3(eixoAnim, NaN, NaN, NaN, 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);

xlabel(eixoAnim, 'X (m)', 'Color', 'w'); ylabel(eixoAnim, 'Y (m)', 'Color', 'w'); zlabel(eixoAnim, 'Z (m)', 'Color', 'w');
tituloAnim = title(eixoAnim, 'TX90  |  Iniciando...', 'Color', [0.95 0.95 1.0], 'FontSize', 13, 'FontWeight', 'bold');

%% Animação: loop principal
show(robo, qTraj(:,1)', 'Parent', eixoAnim, 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false);

tInicio = tic;
for passo = 1:numPassos
    if ~isvalid(figAnim), break; end
    if toc(tInicio) > tempo(passo) + DT*1.5, continue; end   % pula frames atrasados

    show(robo, qTraj(:,passo)', 'Parent', eixoAnim, 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false);

    set(marcadorTCP, 'XData', caminhoEfetuador(1,passo), 'YData', caminhoEfetuador(2,passo), ...
        'ZData', caminhoEfetuador(3,passo));
    set(rastro, 'XData', caminhoEfetuador(1,1:passo), 'YData', caminhoEfetuador(2,1:passo), ...
        'ZData', caminhoEfetuador(3,1:passo), 'CData', corTraj(1:passo));

    if corTraj(passo) == COR_TRANSICAO
        status = 'Equipando Tinta';
    else
        status = 'Pintando a Bandeira';
    end
    set(tituloAnim, 'String', sprintf('TX90  |  %s  |  t = %.2f s  |  %d%%', ...
        status, tempo(passo), round(100*passo/numPassos)));
    drawnow limitrate;

    tRestante = tempo(passo) - toc(tInicio);
    if tRestante > 0, pause(tRestante); end
end

if isvalid(figAnim)
    set(tituloAnim, 'String', 'TX90  |  Bandeira Concluída', 'Color', [0.35 1.0 0.55]);
end

%% Gráficos: perfis de junta, torques e controle
plotarPerfisJuntas(tempo, qTraj, qdTraj, qddTraj, numJuntas);
plotarTorques(tempo, tauTraj, numJuntas);
plotarControle(tempo, qTraj, qSimPD, tauTraj, tauSimPD, numJuntas);

%% ------------------------------------------------------------------------
%  FUNÇÕES DE GERAÇÃO DE TRAJETÓRIA
%  ------------------------------------------------------------------------
function [pontos, cores, vel, acel] = interpolarCartesianaConstante(pontosApoio, velMax, acelMax, dt, codigoCor)
    pontos = [];
    vel = [];
    acel = [];
    for seg = 1:size(pontosApoio,2)-1
        delta = pontosApoio(:,seg+1) - pontosApoio(:,seg);
        distancia = norm(delta);
        [s, v, a] = perfilLSPB(distancia, velMax, acelMax, dt);
        segmento = pontosApoio(:,seg) + (delta/distancia)*s;
        if isempty(pontos)
            pontos = segmento;
            vel = v;
            acel = a;
        else
            pontos = [pontos, segmento(:,2:end)]; %#ok<AGROW>
            vel = [vel, v(2:end)];                %#ok<AGROW>
            acel = [acel, a(2:end)];              %#ok<AGROW>
        end
    end
    cores = repmat(codigoCor, 1, size(pontos,2));
end

function [pontos, cores, vel, acel] = interpolarTransicao(pInicio, pFim, velMax, acelMax, dt, codigoCor)
    delta = pFim - pInicio;
    distancia = norm(delta);
    [s, vel, acel] = perfilLSPB(distancia, velMax, acelMax, dt);
    pontos = pInicio + (delta/distancia)*s;
    cores = repmat(codigoCor, 1, size(pontos,2));
end

function [pontos, cores, vel, acel] = interpolarArco(centro, raio, anguloInicial, anguloFinal, velMax, acelMax, dt, codigoCor)
    sentido = sign(anguloFinal - anguloInicial);
    comprimento = abs(anguloFinal - anguloInicial)*raio;
    [s, vel, acel] = perfilLSPB(comprimento, velMax, acelMax, dt);
    angulo = anguloInicial + sentido*s/raio;
    pontos = [centro(1)*ones(size(angulo));
              centro(2) + raio*cos(angulo);
              centro(3) + raio*sin(angulo)];
    cores = repmat(codigoCor, 1, size(pontos,2));
end

% Perfil LSPB (segmento linear com concordância parabólica) ajustado à grade
% de amostragem: procura o menor número de passos que respeita velMax/acelMax.
function [s, v, a] = perfilLSPB(distancia, velMax, acelMax, dt)
    if distancia <= velMax^2/acelMax
        tempoMinimo = 2*sqrt(distancia/acelMax);
    else
        tempoMinimo = distancia/velMax + velMax/acelMax;
    end

    numIntervalos = max(2, ceil(tempoMinimo/dt));
    achouPerfil = false;
    while ~achouPerfil
        tempoTotal = numIntervalos*dt;
        for numPassosAcel = 1:floor(numIntervalos/2)
            tempoAcel = numPassosAcel*dt;
            vel = distancia/(tempoTotal - tempoAcel);
            acel = vel/tempoAcel;
            if vel <= velMax*(1+1e-10) && acel <= acelMax*(1+1e-10)
                achouPerfil = true;
                break;
            end
        end
        if ~achouPerfil
            numIntervalos = numIntervalos + 1;
        end
    end

    instantes = (0:numIntervalos)*dt;
    s = zeros(size(instantes));
    v = zeros(size(instantes));
    a = zeros(size(instantes));
    for k = 1:numel(instantes)
        if instantes(k) <= tempoAcel + eps
            s(k) = 0.5*acel*instantes(k)^2;
            v(k) = acel*instantes(k);
            a(k) = acel;
        elseif instantes(k) < tempoTotal - tempoAcel - eps
            s(k) = 0.5*acel*tempoAcel^2 + vel*(instantes(k) - tempoAcel);
            v(k) = vel;
        else
            restante = tempoTotal - instantes(k);
            s(k) = distancia - 0.5*acel*restante^2;
            v(k) = acel*restante;
            a(k) = -acel;
        end
    end
    s(1) = 0;
    s(end) = distancia;
    v([1 end]) = 0;
end

%% ------------------------------------------------------------------------
%  FUNÇÕES DE DINÂMICA E CONTROLE
%  ------------------------------------------------------------------------
% Dinâmica inversa: torque necessário para (q, qd, qdd) ao longo do tempo
function tauTraj = calcularTorques(robo, qTraj, qdTraj, qddTraj)
    numJuntas = size(qTraj, 1);
    numPassos = size(qTraj, 2);
    tauTraj = zeros(numJuntas, numPassos);

    estadoAvisos = warning('off', 'all');
    for passo = 1:numPassos
        tauTraj(:,passo) = inverseDynamics(robo, qTraj(:,passo)', qdTraj(:,passo)', qddTraj(:,passo)');
    end
    warning(estadoAvisos);
end

% Análise textual dos torques calculados pela dinâmica inversa
function analisarTorques(tauTraj, tempo, numJuntas)
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

% Controle PD + compensação de gravidade (feedforward)
% Integração em passo fixo, com dois sub-passos em cada amostra.
function [qSim, qdSim, tauSim] = controlePDGravidade(robo, tempo, qTraj, qdTraj, Kp, Kd, tauLimite)
    numJuntas = size(qTraj, 1);
    numPassos = numel(tempo);
    dt = tempo(2) - tempo(1);

    numSubPassos = 2;
    hSub = dt / numSubPassos;

    qSim   = nan(numJuntas, numPassos);
    qdSim  = nan(numJuntas, numPassos);
    tauSim = nan(numJuntas, numPassos);

    qSim(:,1)  = qTraj(:,1);
    qdSim(:,1) = qdTraj(:,1);

    for passo = 1:numPassos-1
        q = qSim(:,passo);
        qd = qdSim(:,passo);
        qRef  = qTraj(:,passo);
        qdRef = qdTraj(:,passo);

        for sub = 1:numSubPassos
            torqueGravidade = gravityTorque(robo, q')';
            tau = Kp.*(qRef - q) + Kd.*(qdRef - qd) + torqueGravidade;
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

% Controle por torque computado (dinâmica inversa em malha fechada)
function [qSim, qdSim, tauSim] = controleTorqueComputado(robo, tempo, qTraj, qdTraj, qddTraj, wn, zeta, tauLimite)
    numJuntas = size(qTraj, 1);
    numPassos = numel(tempo);
    dt = tempo(2) - tempo(1);
    numSubPassos = 2;
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
            acelDesejada = qddTraj(:,passo) + 2*zeta*wn*erroVel + wn^2*erro;
            tau = inverseDynamics(robo, q', qd', acelDesejada')';
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

% Um passo de integração RK4 da dinâmica direta (torque constante no passo)
function [qProximo, qdProximo] = passoRK4Dinamica(robo, q, qd, tau, h)
    f = @(qq, qqd) forwardDynamics(robo, qq', qqd', tau')';

    k1q = qd;                 k1qd = f(q, qd);
    k2q = qd + (h/2)*k1qd;    k2qd = f(q + (h/2)*k1q, qd + (h/2)*k1qd);
    k3q = qd + (h/2)*k2qd;    k3qd = f(q + (h/2)*k2q, qd + (h/2)*k2qd);
    k4q = qd + h*k3qd;        k4qd = f(q + h*k3q,     qd + h*k3qd);

    qProximo  = q  + (h/6)*(k1q  + 2*k2q  + 2*k3q  + k4q);
    qdProximo = qd + (h/6)*(k1qd + 2*k2qd + 2*k3qd + k4qd);
end

% Análise do desempenho do controlador (erro de rastreamento e torque)
function analisarControle(qTraj, qSim, tauTraj, tauSim, numJuntas)
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

%% ------------------------------------------------------------------------
%  FUNÇÕES DE ÁREA DE TRABALHO
%  ------------------------------------------------------------------------
% Amostra o espaço de juntas dentro dos limites do URDF e calcula a nuvem
% de pontos alcançáveis pela ponta do aplicador (estimativa do workspace)
function nuvem = amostrarAreaTrabalho(robo, numAmostras)
    numJuntas = numel(homeConfiguration(robo));
    limites = zeros(numJuntas, 2);

    idxJunta = 0;
    for idxCorpo = 1:numel(robo.Bodies)
        junta = robo.Bodies{idxCorpo}.Joint;
        if ~strcmp(junta.Type, 'fixed')
            idxJunta = idxJunta + 1;
            limitesJunta = junta.PositionLimits;
            if any(isinf(limitesJunta)) || any(isnan(limitesJunta))
                limitesJunta = [-pi, pi];   % fallback para juntas sem limite no URDF
            end
            limites(idxJunta,:) = limitesJunta;
        end
    end

    nuvem = zeros(3, numAmostras);
    estadoAvisos = warning('off', 'all');
    for amostra = 1:numAmostras
        qAleatorio = limites(:,1) + rand(numJuntas,1) .* (limites(:,2) - limites(:,1));
        pose = getTransform(robo, qAleatorio', 'paint_tcp');
        nuvem(:,amostra) = pose(1:3,4);
    end
    warning(estadoAvisos);
end

% Verifica se os pontos desejados da trajetória estão dentro da nuvem
% de alcance estimada (usando o casco convexo da nuvem amostrada)
function verificarAlcancePontos(pontosDesejados, nuvem)
    origemBase = [0; 0; 0];
    raios = vecnorm(nuvem - origemBase);
    raioMax = max(raios);
    raioMin = min(raios);
    raiosPontos = vecnorm(pontosDesejados - origemBase);

    try
        casco = alphaShape(nuvem(1,:)', nuvem(2,:)', nuvem(3,:)', Inf); % Inf = casco convexo
        dentro = inShape(casco, pontosDesejados(1,:)', pontosDesejados(2,:)', pontosDesejados(3,:)');
    catch
        % Fallback: verificação simplificada por raio (casca esférica aproximada)
        dentro = (raiosPontos <= raioMax) & (raiosPontos >= raioMin);
    end

    fprintf('\n--- Verificação da Área de Trabalho ---\n');
    fprintf('Alcance estimado (amostragem): %.3f m (mín) a %.3f m (máx) a partir da base\n', raioMin, raioMax);
    numFora = sum(~dentro);
    if numFora > 0
        fprintf(2, 'ATENÇÃO: %d de %d pontos da trajetória estão FORA da área de trabalho estimada!\n', ...
            numFora, numel(dentro));
        idxFora = find(~dentro);
        for k = 1:min(10, numel(idxFora))
            ponto = pontosDesejados(:, idxFora(k));
            fprintf('  Ponto fora do alcance: [%.3f, %.3f, %.3f] m  (raio = %.3f m)\n', ...
                ponto(1), ponto(2), ponto(3), raiosPontos(idxFora(k)));
        end
        if numel(idxFora) > 10
            fprintf('  ... e mais %d ponto(s).\n', numel(idxFora) - 10);
        end
    else
        fprintf('Todos os %d pontos da trajetória estão dentro da área de trabalho estimada.\n', numel(dentro));
    end
    fprintf('----------------------------------------\n\n');
end

% Plota a nuvem de pontos da área de trabalho junto com a trajetória desejada
function plotarAreaTrabalho(nuvem, pontosDesejados)
    corFundo = [0.07 0.07 0.10];
    corEixos = [0.65 0.75 0.85];
    figWorkspace = figure('Name', 'TX90 – Área de Trabalho (Workspace)', 'NumberTitle', 'off', ...
        'Color', corFundo, 'Position', [40 40 700 600]);
    eixo = axes('Parent', figWorkspace);
    set(eixo, 'Color', corFundo, 'XColor', corEixos, 'YColor', corEixos, 'ZColor', corEixos, ...
        'GridColor', [0.22 0.27 0.32], 'GridAlpha', 0.6, 'FontSize', 9);
    hold(eixo, 'on'); grid(eixo, 'on'); view(eixo, 140, 22); axis(eixo, 'equal');

    scatter3(eixo, nuvem(1,:), nuvem(2,:), nuvem(3,:), 4, ...
        [0.30 0.45 0.65], 'filled', 'MarkerFaceAlpha', 0.15, 'DisplayName', 'Alcance amostrado');
    plot3(eixo, pontosDesejados(1,:), pontosDesejados(2,:), pontosDesejados(3,:), ...
        'o', 'MarkerSize', 5, 'MarkerFaceColor', [1 0.4 0.2], 'MarkerEdgeColor', 'w', ...
        'DisplayName', 'Pontos da trajetória');

    xlabel(eixo, 'X (m)', 'Color', 'w'); ylabel(eixo, 'Y (m)', 'Color', 'w'); zlabel(eixo, 'Z (m)', 'Color', 'w');
    title(eixo, 'Área de Trabalho Estimada vs. Trajetória Desejada', ...
        'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
    legend(eixo, 'Location', 'best', 'TextColor', 'w', 'Color', [0.12 0.12 0.18], 'EdgeColor', [0.30 0.30 0.40]);
end

%% ------------------------------------------------------------------------
%  FUNÇÕES DE GRÁFICOS
%  ------------------------------------------------------------------------
function plotarPerfisJuntas(tempo, qTraj, qdTraj, qddTraj, numJuntas)
    corFundo = [0.07 0.07 0.10];
    corEixos = [0.65 0.75 0.85];
    mapaCores = lines(numJuntas);
    nomesJuntas = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6'};
    figPerfis = figure('Name', 'TX90 – Perfis de Junta', 'NumberTitle', 'off', ...
        'Color', corFundo, 'Position', [1260 40 680 760]);
    dados   = {qTraj, qdTraj, qddTraj};
    titulos = {'Posição (rad)', 'Velocidade (rad/s)', 'Aceleração (rad/s²)'};

    for grafico = 1:3
        eixoSub = subplot(3, 1, grafico, 'Parent', figPerfis);
        set(eixoSub, 'Color', corFundo, 'XColor', corEixos, 'YColor', corEixos, ...
            'GridColor', [0.22 0.27 0.32], 'GridAlpha', 0.6, 'FontSize', 9);
        hold(eixoSub, 'on'); grid(eixoSub, 'on');
        for junta = 1:numJuntas
            plot(eixoSub, tempo, dados{grafico}(junta,:), 'LineWidth', 1.8, ...
                'Color', mapaCores(junta,:), 'DisplayName', nomesJuntas{junta});
        end
        ylabel(eixoSub, titulos{grafico}, 'Color', 'w', 'FontSize', 10);
        title(eixoSub, titulos{grafico}, 'Color', 'w', 'FontWeight', 'bold');
        if grafico == 3, xlabel(eixoSub, 'Tempo (s)', 'Color', 'w', 'FontSize', 10); end
        legend(eixoSub, nomesJuntas, 'Location', 'best', 'TextColor', 'w', ...
            'Color', [0.12 0.12 0.18], 'EdgeColor', [0.30 0.30 0.40]);
    end
    sgtitle(figPerfis, 'TX90 – Perfis de Junta (Troca de Ferramenta)', ...
        'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
end

% Gráfico torque x tempo (dinâmica inversa) por junta
function plotarTorques(tempo, tauTraj, numJuntas)
    corFundo = [0.07 0.07 0.10];
    corEixos = [0.65 0.75 0.85];
    mapaCores = lines(numJuntas);
    nomesJuntas = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6'};

    figTorques = figure('Name', 'TX90 – Torques (Dinâmica Inversa)', 'NumberTitle', 'off', ...
        'Color', corFundo, 'Position', [40 800 900 480]);
    eixo = axes('Parent', figTorques);
    set(eixo, 'Color', corFundo, 'XColor', corEixos, 'YColor', corEixos, ...
        'GridColor', [0.22 0.27 0.32], 'GridAlpha', 0.6, 'FontSize', 9);
    hold(eixo, 'on'); grid(eixo, 'on');
    for junta = 1:numJuntas
        plot(eixo, tempo, tauTraj(junta,:), 'LineWidth', 1.8, ...
            'Color', mapaCores(junta,:), 'DisplayName', nomesJuntas{junta});
    end
    xlabel(eixo, 'Tempo (s)', 'Color', 'w', 'FontSize', 10);
    ylabel(eixo, 'Torque (Nm)', 'Color', 'w', 'FontSize', 10);
    title(eixo, 'Torque por Junta ao Longo da Trajetória (Dinâmica Inversa)', ...
        'Color', 'w', 'FontWeight', 'bold');
    legend(eixo, nomesJuntas, 'Location', 'best', 'TextColor', 'w', ...
        'Color', [0.12 0.12 0.18], 'EdgeColor', [0.30 0.30 0.40]);
end

% Gráfico comparando torque ideal vs torque do controlador e erro de rastreamento
function plotarControle(tempo, qTraj, qSim, tauTraj, tauSim, numJuntas)
    corFundo = [0.07 0.07 0.10];
    corEixos = [0.65 0.75 0.85];
    mapaCores = lines(numJuntas);
    nomesJuntas = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6'};
    erro = qTraj - qSim;

    figControle = figure('Name', 'TX90 – Controle PD + Compensação de Gravidade', 'NumberTitle', 'off', ...
        'Color', corFundo, 'Position', [960 800 900 760]);

    % Subplot 1: erro de rastreamento por junta
    eixoErro = subplot(3, 1, 1, 'Parent', figControle);
    set(eixoErro, 'Color', corFundo, 'XColor', corEixos, 'YColor', corEixos, ...
        'GridColor', [0.22 0.27 0.32], 'GridAlpha', 0.6, 'FontSize', 9);
    hold(eixoErro, 'on'); grid(eixoErro, 'on');
    for junta = 1:numJuntas
        plot(eixoErro, tempo, erro(junta,:), 'LineWidth', 1.6, ...
            'Color', mapaCores(junta,:), 'DisplayName', nomesJuntas{junta});
    end
    ylabel(eixoErro, 'Erro (rad)', 'Color', 'w', 'FontSize', 10);
    title(eixoErro, 'Erro de Rastreamento (q_{ref} - q_{sim})', 'Color', 'w', 'FontWeight', 'bold');
    legend(eixoErro, nomesJuntas, 'Location', 'best', 'TextColor', 'w', ...
        'Color', [0.12 0.12 0.18], 'EdgeColor', [0.30 0.30 0.40]);

    % Subplot 2: torque aplicado pelo controlador (PD + gravidade)
    eixoTorque = subplot(3, 1, 2, 'Parent', figControle);
    set(eixoTorque, 'Color', corFundo, 'XColor', corEixos, 'YColor', corEixos, ...
        'GridColor', [0.22 0.27 0.32], 'GridAlpha', 0.6, 'FontSize', 9);
    hold(eixoTorque, 'on'); grid(eixoTorque, 'on');
    for junta = 1:numJuntas
        plot(eixoTorque, tempo, tauSim(junta,:), 'LineWidth', 1.6, ...
            'Color', mapaCores(junta,:), 'DisplayName', nomesJuntas{junta});
    end
    ylabel(eixoTorque, 'Torque (Nm)', 'Color', 'w', 'FontSize', 10);
    title(eixoTorque, 'Torque Aplicado pelo Controlador (PD + Gravidade)', 'Color', 'w', 'FontWeight', 'bold');
    legend(eixoTorque, nomesJuntas, 'Location', 'best', 'TextColor', 'w', ...
        'Color', [0.12 0.12 0.18], 'EdgeColor', [0.30 0.30 0.40]);

    % Subplot 3: diferença entre torque ideal (dinâmica inversa) e o do controlador
    eixoDif = subplot(3, 1, 3, 'Parent', figControle);
    set(eixoDif, 'Color', corFundo, 'XColor', corEixos, 'YColor', corEixos, ...
        'GridColor', [0.22 0.27 0.32], 'GridAlpha', 0.6, 'FontSize', 9);
    hold(eixoDif, 'on'); grid(eixoDif, 'on');
    difTorque = tauTraj - tauSim;
    for junta = 1:numJuntas
        plot(eixoDif, tempo, difTorque(junta,:), 'LineWidth', 1.6, ...
            'Color', mapaCores(junta,:), 'DisplayName', nomesJuntas{junta});
    end
    xlabel(eixoDif, 'Tempo (s)', 'Color', 'w', 'FontSize', 10);
    ylabel(eixoDif, 'Torque (Nm)', 'Color', 'w', 'FontSize', 10);
    title(eixoDif, 'Diferença: Torque Ideal (Din. Inversa) − Torque do Controlador', ...
        'Color', 'w', 'FontWeight', 'bold');
    legend(eixoDif, nomesJuntas, 'Location', 'best', 'TextColor', 'w', ...
        'Color', [0.12 0.12 0.18], 'EdgeColor', [0.30 0.30 0.40]);

    sgtitle(figControle, 'TX90 – Desempenho do Controle PD + Compensação de Gravidade', ...
        'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
end
