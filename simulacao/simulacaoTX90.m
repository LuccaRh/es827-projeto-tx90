%% Simulação do Stäubli TX90 — ES827 (Robótica Industrial)
%  Pintura da bandeira do Brasil: troca de ferramentas (cores), trajetória
%  segmentada com perfil LSPB, cinemática inversa, dinâmica inversa (torques)
%  e controle PD + compensação de gravidade / torque computado.
%
%  Script principal: define os parâmetros e orquestra as funções do projeto
%  (carregarRobo, gerarTrajetoria, resolverCinematicaInversa, calcularTorques,
%  simularControlador, verificarAreaTrabalho, animarRobo, plotarResultados).
%
%  Requisitos: Robotics System Toolbox.
%  Executar a partir da pasta simulacao/ (o URDF é carregado por caminho relativo).
clearvars; close all; clc;

%% Imports

% Pega o caminho do script atual e adiciona a pasta "funcoes" ao path do MATLAB
scriptDir = fileparts(mfilename('fullpath'));
% Adiciona a pasta "funcoes" ao path do MATLAB para que as funções auxiliares possam ser chamadas
addpath(fullfile(scriptDir, "funcoes"));

%% Parâmetros
% --- Amostragem e limites cartesianos da trajetória ---
DT             = 0.04;   % passo de amostragem [s]
VEL_PINTURA    = 0.12;   % velocidade máxima com spray ligado [m/s]
ACEL_PINTURA   = 0.30;   % aceleração máxima com spray ligado [m/s^2]
VEL_TRANSICAO  = 0.35;   % velocidade máxima em movimento livre [m/s]
ACEL_TRANSICAO = 0.60;   % aceleração máxima em movimento livre [m/s^2]

% --- Preenchimento da bandeira (ver +trajetoria/gerarTrajetoria) ---
PREENCHER_BANDEIRA = false;    % true = preenche as formas (serpentina); false = só contorno (rápido p/ testar)
PASSO_LEQUE        = 0.015;   % espaçamento entre passadas [m]. MENOR = mais resolução (mais lento); MAIOR = mais rápido/mais ralo
LARGURA_FAIXA      = 0.05;    % largura da faixa branca preenchida [m]
ESPESSURA_CAMADA   = 0.004;   % deslocamento em -X por camada de cor [m], p/ as cores de cima não sumirem atrás do verde

NUM_CONTROLADORES = 1;   % 1 = só PD+gravidade (rápido); 2 = também torque computado (comparação p/ relatório)

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
ferramenta = struct('comprimentoTcp', COMPRIMENTO_TCP, 'massa', MASSA_FERRAMENTA, ...
    'centroMassa', CENTRO_MASSA_FERRAMENTA, 'inercia', INERCIA_FERRAMENTA);
[robo, numJuntas] = robotica.carregarRobo('tx90.urdf', ferramenta);

%% Trajetória cartesiana
geometria = struct('centroBandeira', CENTRO_BANDEIRA, 'pontoEstacao', PONTO_ESTACAO, ...
    'larguraRetangulo', LARGURA_RETANGULO, 'alturaRetangulo', ALTURA_RETANGULO, ...
    'larguraLosango', LARGURA_LOSANGO, 'alturaLosango', ALTURA_LOSANGO, ...
    'raioCirculo', RAIO_CIRCULO, 'centroArco', CENTRO_ARCO, 'raioArco', RAIO_ARCO, ...
    'larguraFaixa', LARGURA_FAIXA);
perfis = struct('dt', DT, 'velPintura', VEL_PINTURA, 'acelPintura', ACEL_PINTURA, ...
    'velTransicao', VEL_TRANSICAO, 'acelTransicao', ACEL_TRANSICAO, ...
    'preencher', PREENCHER_BANDEIRA, 'passoLeque', PASSO_LEQUE, ...
    'espessuraCamada', ESPESSURA_CAMADA);
codigosCor = struct('verde', COR_VERDE, 'transicao', COR_TRANSICAO, ...
    'amarelo', COR_AMARELO, 'azul', COR_AZUL, 'branco', COR_BRANCO);

traj = trajetoria.gerarTrajetoria(geometria, perfis, codigosCor);
numPassos = numel(traj.tempo);

fprintf('Trajetória: %d pontos, duração %.2f s.\n', numPassos, traj.tempo(end));
fprintf('Velocidade cartesiana comandada máxima: %.4f m/s.\n', max(traj.vel));
fprintf('Aceleração cartesiana comandada máxima: %.4f m/s^2.\n', max(abs(traj.acel)));

%% Verificação da área de trabalho (workspace)
fprintf('Verificando área de trabalho do manipulador...\n');
areaTrabalho.verificarAreaTrabalho(robo, traj.pontosDesejados, NUM_AMOSTRAS_WORKSPACE);

%% Cinemática inversa e perfis articulares
fprintf('Calculando cinemática inversa. Aguarde...\n');
[qTraj, caminhoEfetuador, erroPosIK, erroOriIK, sigmaMin] = ...
    robotica.resolverCinematicaInversa(robo, traj.posCart, R_FERRAMENTA, PESOS_IK);

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
fprintf('Calculando torques via dinâmica inversa (modelo URDF)...\n');
tauTraj = robotica.calcularTorques(robo, qTraj, qdTraj, qddTraj, traj.tempo);

torqueMaximo = max(abs(tauTraj), [], 2);
if any(torqueMaximo > LIMITE_TORQUE)
    error('A trajetória ultrapassa o limite de torque de uma junta.');
end

%% Controle em malha fechada
% Os ganhos são calculados a partir da matriz de massa M(q0) do próprio
% robô, e não fixados arbitrariamente: juntas com pouca inércia (punho)
% ficam instáveis com ganhos altos demais, enquanto juntas de base toleram
% e precisam de ganhos maiores.
M0 = massMatrix(robo, qTraj(:,1)');
Kp = (WN^2) * diag(M0);
Kd = (2*ZETA*WN) * diag(M0);

% NUM_CONTROLADORES: 1 = só PD+gravidade (rápido); 2 = também torque computado.
% A simulação roda na grade FINA (passo DT) — necessário para a estabilidade
% numérica da integração em malha fechada.
modos = {'pdGravidade', 'torqueComputado'};
modos = modos(1:NUM_CONTROLADORES);
qSim   = cell(1, numel(modos));
tauSim = cell(1, numel(modos));
for m = 1:numel(modos)
    fprintf('Simulando controlador %d/%d (%s)...\n', m, numel(modos), modos{m});
    [qSim{m}, ~, tauSim{m}] = controle.simularControlador(robo, traj.tempo, qTraj, qdTraj, qddTraj, ...
        Kp, Kd, WN, ZETA, LIMITE_TORQUE, modos{m});
end

% O 1º controlador (PD+gravidade) é a referência para a análise e os gráficos
qSimPD   = qSim{1};
tauSimPD = tauSim{1};
controle.analisarControle(qTraj, qSimPD, tauTraj, tauSimPD, numJuntas);
erroRmsPD = sqrt(mean((qTraj - qSimPD).^2, 2));

fprintf('\n--- RESULTADOS VALIDADOS ---\n');
fprintf('Duração: %.2f s | pontos: %d\n', traj.tempo(end), numPassos);
fprintf('Erro IK máximo: %.3f mm | orientação: %.3e rad\n', 1e3*max(erroPosIK), max(erroOriIK));
fprintf('Sigma mínimo do Jacobiano: %.3e\n', min(sigmaMin));
fprintf('Torque máximo por junta [Nm]: %s\n', mat2str(torqueMaximo', 4));
fprintf('RMS erro PD+G [rad]: %s\n', mat2str(erroRmsPD', 4));
if numel(modos) >= 2
    erroRmsTC = sqrt(mean((qTraj - qSim{2}).^2, 2));
    fprintf('RMS erro torque computado [rad]: %s\n', mat2str(erroRmsTC', 4));
end

%% Animação e gráficos
visualizacao.animarRobo(robo, qTraj, caminhoEfetuador, traj.cor, traj.tempo, ...
    CORES_RASTRO, COR_TRANSICAO, PONTO_ESTACAO);

visualizacao.plotarResultados(traj.tempo, qTraj, qdTraj, qddTraj, tauTraj, qSimPD, tauSimPD, numJuntas);
