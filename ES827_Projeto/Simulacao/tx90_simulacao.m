%  Staubli TX90: Troca de Ferramentas (Cores), Trajetoria Segmentada,
%  Dinamica (Torques) e Controle PD + Compensacao de Gravidade (Feedforward)
clearvars; close all; clc;

%% Carregar modelo 
robot = importrobot('tx90.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];              % garante gravidade padrao (m/s^2)
numDoF = numel(homeConfiguration(robot));

% Aplicador considerado nos calculos (15 cm e 3 kg)
paintTCP = rigidBody('paint_tcp');
paintJoint = rigidBodyJoint('paint_tcp_fixed','fixed');
setFixedTransform(paintJoint, trvec2tform([0 0 0.15]));
paintTCP.Joint = paintJoint;
paintTCP.Mass = 3.0;
paintTCP.CenterOfMass = [0 0 -0.075];
paintTCP.Inertia = [0.006825 0.006825 0.0024 0 0 0];
addBody(robot, paintTCP, 'tool0');

limiteVelocidade = deg2rad([400; 400; 430; 540; 475; 760]);
limiteTorque = [318; 166; 76; 34; 29; 11];

%% Waypoints Cartesianos e Esta��o de Troca 
Xc = 0.65; Yc = 0.0; Zc = 0.45;

% Esta��o de Troca de Ferramenta (Afastada e lateral)
pTool = [0.5; 0.6; 0.2];

% A. Ret�ngulo
wR = 0.70; hR = 0.5;
p1 = [Xc; Yc-wR/2; Zc+hR/2]; 
p2 = [Xc; Yc+wR/2; Zc+hR/2]; 
p3 = [Xc; Yc+wR/2; Zc-hR/2]; 
p4 = [Xc; Yc-wR/2; Zc-hR/2]; 
pRetangulo = [p1, p2, p3, p4, p1];

% B. Losango
wL = 0.581; hL = 0.381;
p5 = [Xc; Yc; Zc+hL/2]; 
p6 = [Xc; Yc+wL/2; Zc]; 
p7 = [Xc; Yc; Zc-hL/2]; 
p8 = [Xc; Yc-wL/2; Zc]; 
pLosango = [p5, p6, p7, p8, p5];

% C. Ci�rculo
rC = 0.1225;
theta = linspace(pi/2, -3*pi/2, 40);
pCirc = [Xc*ones(1,40); Yc + rC*cos(theta); Zc + rC*sin(theta)];

% D. Arco

% Centro do arco
Xc = 0.65;
Yc = -0.07;
Zc = 0.2;

rA = 0.28875;

% �ngulos inicial e final do arco
theta0 = atan2(0.28483, -0.11745 + 0.07);
thetaF = atan2(0.21877, 0.11845 + 0.07);

% Vetor de �ngulos
theta = linspace(theta0, thetaF, 40);

% Pontos do arco
pArc = [ ...
    Xc*ones(1,40);
    Yc + rA*cos(theta);
    Zc + rA*sin(theta)
];

%%  3. Constru��o da Trajet�ria (Idas e Vindas � Esta��o) 
dt = 0.04;           % Passo de tempo (s)
v_draw = 0.12;       % Velocidade maxima durante o desenho (m/s)
a_draw = 0.30;       % Aceleracao maxima durante o desenho (m/s2)
v_trans = 0.35;      % Velocidade maxima nas transicoes (m/s)
a_trans = 0.60;      % Aceleracao maxima nas transicoes (m/s2)

% 1=Verde, 2=Cinza(Transi��o/Movimento Livre), 3=Amarelo, 4=Azul

% Inicio: Esta��o -> Ret�ngulo -> Esta��o
[p_T1, c_T1, v_T1, a_T1] = interpTransicao(pTool, pRetangulo(:,1), v_trans, a_trans, dt, 2);
[p_R,  c_R,  v_R,  a_R]  = interpCartesianaConstante(pRetangulo, v_draw, a_draw, dt, 1);
[p_T2, c_T2, v_T2, a_T2] = interpTransicao(pRetangulo(:,end), pTool, v_trans, a_trans, dt, 2);

% Esta��o -> Losango -> Esta��o
[p_T3, c_T3, v_T3, a_T3] = interpTransicao(pTool, pLosango(:,1), v_trans, a_trans, dt, 2);
[p_L,  c_L,  v_L,  a_L]  = interpCartesianaConstante(pLosango, v_draw, a_draw, dt, 3);
[p_T4, c_T4, v_T4, a_T4] = interpTransicao(pLosango(:,end), pTool, v_trans, a_trans, dt, 2);

% Esta��o -> C�rculo -> Esta��o
[p_T5, c_T5, v_T5, a_T5] = interpTransicao(pTool, pCirc(:,1), v_trans, a_trans, dt, 2);
[p_C,  c_C,  v_C,  a_C]  = interpArco([0.65;0;0.45], rC, pi/2, -3*pi/2, v_draw, a_draw, dt, 4);
[p_T6, c_T6, v_T6, a_T6] = interpTransicao(pCirc(:,1), pTool, v_trans, a_trans, dt, 2);

% Esta��o -> arco -> Guarda Ferramenta
[p_T7, c_T7, v_T7, a_T7] = interpTransicao(pTool, pArc(:,1), v_trans, a_trans, dt, 2);
[p_A,  c_A,  v_A,  a_A]  = interpArco([0.65;-0.07;0.2], rA, theta0, thetaF, v_draw, a_draw, dt, 5);
[p_T8, c_T8, v_T8, a_T8] = interpTransicao(pArc(:,end), pTool, v_trans, a_trans, dt, 2);

% Concatena tudo (removendo o 1� ponto dos blocos subsequentes)
pos_cart = [p_T1, p_R(:,2:end), p_T2(:,2:end), p_T3(:,2:end), p_L(:,2:end), ...
            p_T4(:,2:end), p_T5(:,2:end), p_C(:,2:end), p_T6(:,2:end), p_T7(:,2:end), p_A(:,2:end), p_T8(:,2:end)];
        
color_idx = [c_T1, c_R(2:end), c_T2(2:end), c_T3(2:end), c_L(2:end), ...
             c_T4(2:end), c_T5(2:end), c_C(2:end), c_T6(2:end), c_T7(2:end), c_A(2:end), c_T8(2:end)];

vel_cart = [v_T1, v_R(2:end), v_T2(2:end), v_T3(2:end), v_L(2:end), ...
            v_T4(2:end), v_T5(2:end), v_C(2:end), v_T6(2:end), v_T7(2:end), v_A(2:end), v_T8(2:end)];
acc_cart = [a_T1, a_R(2:end), a_T2(2:end), a_T3(2:end), a_L(2:end), ...
            a_T4(2:end), a_T5(2:end), a_C(2:end), a_T6(2:end), a_T7(2:end), a_A(2:end), a_T8(2:end)];

numSteps = size(pos_cart, 2);
tvec = (0:numSteps-1) * dt;

fprintf('Trajetoria: %d pontos, duracao %.2f s.\n', numSteps, tvec(end));
fprintf('Velocidade cartesiana comandada maxima: %.4f m/s.\n', max(vel_cart));
fprintf('Aceleracao cartesiana comandada maxima: %.4f m/s^2.\n', max(abs(acc_cart)));

%%  3.5 Verifica��o da �rea de Trabalho (Workspace) 
% Amostra o espa�o de juntas (dentro dos limites do URDF) para estimar a
% nuvem de pontos alcan��veis pela ponta do aplicador, e verifica se todos os pontos da
% trajet�ria desejada (incluindo a esta��o de troca) caem dentro dela.
fprintf('Verificando �rea de trabalho do manipulador...\n');
numAmostrasWS = 50000;
nuvemWS = calcularAreaTrabalho(robot, numAmostrasWS);

pontosDesejados = [pRetangulo, pLosango, pCirc, pArc, pTool];
verificarAlcancePontos(pontosDesejados, nuvemWS);
plotAreaTrabalho(nuvemWS, pontosDesejados);

%%  4. Cinem�tica Inversa (IK) 
fprintf('Calculando Cinem�tica Inversa. Aguarde...\n');
ik = inverseKinematics('RigidBodyTree', robot);
ikWeights = [0.5 0.5 0.5 1 1 1]; 

% Orienta��o frontal (+X) mantida em toda a opera��o
R_fixed = [ 0  0  1; 
            0  1  0; 
           -1  0  0 ];
           
initialGuess = homeConfiguration(robot);
q_traj = zeros(numDoF, numSteps);
erroPosIK = zeros(1, numSteps);
erroOriIK = zeros(1, numSteps);
sigmaMin = zeros(1, numSteps);

estadoAvisos = warning('off', 'all');
for i = 1:numSteps
    T_target = trvec2tform(pos_cart(:,i)') * rotm2tform(R_fixed);
    [qSol, ~] = ik('paint_tcp', T_target, ikWeights, initialGuess);
    q_traj(:,i) = qSol';
    initialGuess = qSol; 
end
warning(estadoAvisos); 

% Extrair caminho do End-Effector
eePath = zeros(3, numSteps);
for i = 1:numSteps
    T = getTransform(robot, q_traj(:,i)', 'paint_tcp');
    eePath(:,i) = T(1:3,4);
    erroPosIK(i) = norm(T(1:3,4) - pos_cart(:,i));
    erroRot = rotm2axang(R_fixed * T(1:3,1:3)');
    erroOriIK(i) = abs(erroRot(4));
    J = geometricJacobian(robot, q_traj(:,i)', 'paint_tcp');
    sv = svd(J);
    sigmaMin(i) = min(sv);
end

if max(erroPosIK) > 2e-4 || max(erroOriIK) > 2e-3
    error('A cinematica inversa nao atingiu a tolerancia esperada.');
end

qd_traj = zeros(size(q_traj));
qdd_traj = zeros(size(q_traj));
for j = 1:numDoF
    qd_traj(j,:) = gradient(q_traj(j,:), dt);
    qdd_traj(j,:) = gradient(qd_traj(j,:), dt);
end

if any(max(abs(qd_traj),[],2) > limiteVelocidade)
    error('A trajetoria ultrapassa o limite de velocidade de uma junta.');
end

%%  4b. Din�mica: C�lculo dos Torques Necess�rios (a partir do URDF) 
% Torque "ideal" (feedforward completo) que a din�mica inversa do rob�
% exige para executar exatamente a trajet�ria planejada (q, qd, qdd).
fprintf('Calculando torques via din�mica inversa (modelo URDF)...\n');
tau_traj = calcularTorques(robot, q_traj, qd_traj, qdd_traj);

% An�lise r�pida dos torques (impressa no console)
analisarTorques(tau_traj, tvec, numDoF);

torqueMaximo = max(abs(tau_traj), [], 2);
if any(torqueMaximo > limiteTorque)
    error('A trajetoria ultrapassa o limite de torque de uma junta.');
end

%%  4c. Controle de Torque com Compensa��o de Gravidade (Feedforward) 
% Lei de controle:  tau = Kp*(q_ref - q) + Kd*(qd_ref - qd) + G(q)
% onde G(q) � o torque gravitacional calculado pelo modelo URDF (feedforward),
% e o termo PD realimenta o erro de posi��o/velocidade medido em malha fechada.
fprintf('Simulando controle PD + compensa��o de gravidade (feedforward)...\n');

% Os ganhos s�o calculados a partir da matriz de massa M(q0) do pr�prio
% rob�, e n�o fixados arbitrariamente. Isso � necess�rio porque juntas com
% pouca in�rcia (ex.: punho, J4-J6) ficam inst�veis com ganhos altos demais
% (frequ�ncia natural de malha fechada incompat�vel com o passo de
% integra��o), enquanto juntas de base (J1-J3, mais massa) toleram e
% precisam de ganhos maiores. Crit�rio: wn = frequ�ncia natural desejada,
% zeta = fator de amortecimento (1 = criticamente amortecido).
wn   = 5;      % rad/s
zeta = 1.0;

M0 = massMatrix(robot, q_traj(:,1)');
Kp = (wn^2) * diag(M0);
Kd = (2*zeta*wn) * diag(M0);

% Limite de torque de seguran�a (satura o comando do controlador para
% evitar picos irreais/instabilidade num�rica caso a malha fechada oscile);
% usando os limites de esforco informados para cada junta.
tauLimite = limiteTorque;

[qSim, qdSim, tauSim] = controlePDGravidade(robot, tvec, q_traj, qd_traj, Kp, Kd, tauLimite);
[qSimCT, ~, ~] = controleTorqueComputado(robot, tvec, q_traj, qd_traj, qdd_traj, wn, zeta, tauLimite);

% An�lise do desempenho do controlador (erro de rastreamento, torque aplicado)
analisarControle(tvec, q_traj, qSim, tau_traj, tauSim, numDoF);

erroPD = sqrt(mean((q_traj - qSim).^2, 2));
erroCT = sqrt(mean((q_traj - qSimCT).^2, 2));

fprintf('\n--- RESULTADOS VALIDADOS ---\n');
fprintf('Duracao: %.2f s | pontos: %d\n', tvec(end), numSteps);
fprintf('Erro IK maximo: %.3f mm | orientacao: %.3e rad\n', 1e3*max(erroPosIK), max(erroOriIK));
fprintf('Sigma minimo do Jacobiano: %.3e\n', min(sigmaMin));
fprintf('Torque maximo por junta [Nm]: %s\n', mat2str(torqueMaximo',4));
fprintf('RMS erro PD+G [rad]: %s\n', mat2str(erroPD',4));
fprintf('RMS erro torque computado [rad]: %s\n', mat2str(erroCT',4));

%%  5. Setup da Figura 
hFig = figure('Name','TX90 v9 � Troca de Ferramentas', ...
    'NumberTitle','off', 'Color',[0.07 0.07 0.10], 'Position',[40 40 1200 730]);

ax = axes('Parent',hFig, 'Color',[0.07 0.07 0.10], 'XColor',[0.55 0.65 0.75], ...
    'YColor',[0.55 0.65 0.75], 'ZColor',[0.55 0.65 0.75], ...
    'GridColor',[0.20 0.25 0.30], 'GridAlpha',0.55, 'FontSize',10);

hold(ax,'on'); grid(ax,'on'); view(ax, 140, 22); axis(ax,'equal');
xlim(ax, [-0.10, 1.10]); ylim(ax, [-0.80, 0.80]); zlim(ax, [ 0.00, 1.00]);

camlight(ax, 'headlight'); camlight(ax, 'right'); lighting(ax, 'gouraud');

% Mapeamento de cores
coresBandeira = [
    0.00  0.80  0.30;  % 1: Verde (Ret�ngulo)
    0.40  0.40  0.45;  % 2: Cinza (Transi��o/Livre)
    1.00  0.85  0.00;  % 3: Amarelo (Losango)
    0.00  0.50  1.00   % 4: Azul (C�rculo)
    1.00  1.0  1.0   % 5: Branco (C�rculo)
];
colormap(ax, coresBandeira);
set(ax, 'CLim', [1 5]); 

%%  6. Renderiza��o Inicial 
% Marcador da Esta��o de Troca
plot3(ax, pTool(1), pTool(2), pTool(3), 's', 'MarkerSize', 16, ...
    'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
text(ax, pTool(1), pTool(2), pTool(3)+0.12, 'Esta��o de Cores', ...
    'Color', [0.8 0.8 0.9], 'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');

% Rastro principal
hRastroVivo = scatter3(ax, NaN, NaN, NaN, 18, NaN, 'filled', 'MarkerEdgeColor','none');

hTCP = plot3(ax, NaN,NaN,NaN, 'o', 'MarkerSize',8, ...
    'MarkerFaceColor','w', 'MarkerEdgeColor','k', 'LineWidth',1.2);

xlabel(ax,'X (m)','Color','w'); ylabel(ax,'Y (m)','Color','w'); zlabel(ax,'Z (m)','Color','w');
hTitle = title(ax,'TX90  |  Iniciando...', 'Color',[0.95 0.95 1.0],'FontSize',13,'FontWeight','bold');

%%  7. Loop de anima��o 
show(robot, q_traj(:,1)', 'Parent',ax, 'Visuals','on','Frames','off','PreservePlot',false);

tStart = tic;
for i = 1:numSteps
    if ~isvalid(hFig), break; end
    if toc(tStart) > tvec(i) + dt*1.5, continue; end

    show(robot, q_traj(:,i)', 'Parent',ax, 'Visuals','on','Frames','off','PreservePlot',false);

    set(hTCP, 'XData',eePath(1,i), 'YData',eePath(2,i), 'ZData',eePath(3,i));
    set(hRastroVivo, 'XData',eePath(1,1:i), 'YData',eePath(2,1:i), ...
                     'ZData',eePath(3,1:i), 'CData',color_idx(1:i));
    
    % Atualiza o t�tulo baseado no status atual
    if color_idx(i) == 2
        status = 'Equipando Tinta';
    else
        status = 'Pintando a Bandeira';
    end
                 
    set(hTitle,'String', sprintf('TX90  |  %s  |  t = %.2f s  |  %d%%', status, tvec(i), round(100*i/numSteps)));
    drawnow limitrate;
    
    tRestante = tvec(i) - toc(tStart);
    if tRestante > 0, pause(tRestante); end
end

if isvalid(hFig)
    set(hTitle,'String','TX90  |  Bandeira Conclu�da ','Color',[0.35 1.0 0.55]);
end

%%  8. Gr�ficos: perfis de junta, torques e controle 
plotPerfisJunta(tvec, q_traj, qd_traj, qdd_traj, numDoF);
plotTorques(tvec, tau_traj, numDoF);
plotControleResultados(tvec, q_traj, qSim, tau_traj, tauSim, numDoF);

%% 
%  FUN��ES DE GERA��O DE TRAJET�RIA, DIN�MICA, CONTROLE E PLOT
% 
function [p_out, c_out, v_out, a_out] = interpCartesianaConstante(waypoints, v_draw, a_draw, dt, color_code)
    p_out = [];
    v_out = [];
    a_out = [];
    for i = 1:size(waypoints,2)-1
        delta = waypoints(:,i+1) - waypoints(:,i);
        dist = norm(delta);
        [s, v, a] = perfilLSPB(dist, v_draw, a_draw, dt);
        segmento = waypoints(:,i) + (delta/dist)*s;
        if isempty(p_out)
            p_out = segmento;
            v_out = v;
            a_out = a;
        else
            p_out = [p_out, segmento(:,2:end)];
            v_out = [v_out, v(2:end)];
            a_out = [a_out, a(2:end)];
        end
    end
    c_out = repmat(color_code, 1, size(p_out,2));
end

function [p_out, c_out, v_out, a_out] = interpTransicao(p_start, p_end, v_max, a_max, dt, color_code)
    delta = p_end - p_start;
    dist = norm(delta);
    [s, v_out, a_out] = perfilLSPB(dist, v_max, a_max, dt);
    p_out = p_start + (delta/dist)*s;
    c_out = repmat(color_code, 1, size(p_out,2));
end

function [p_out, c_out, v_out, a_out] = interpArco(centro, raio, theta0, thetaF, v_max, a_max, dt, color_code)
    sentido = sign(thetaF-theta0);
    comprimento = abs(thetaF-theta0)*raio;
    [s, v_out, a_out] = perfilLSPB(comprimento, v_max, a_max, dt);
    theta = theta0 + sentido*s/raio;
    p_out = [centro(1)*ones(size(theta));
             centro(2) + raio*cos(theta);
             centro(3) + raio*sin(theta)];
    c_out = repmat(color_code, 1, size(p_out,2));
end

function [s, v, a] = perfilLSPB(dist, v_max, a_max, dt)
    if dist <= v_max^2/a_max
        tempoMinimo = 2*sqrt(dist/a_max);
    else
        tempoMinimo = dist/v_max + v_max/a_max;
    end

    n = max(2, ceil(tempoMinimo/dt));
    achouPerfil = false;
    while ~achouPerfil
        tempoTotal = n*dt;
        for nAcel = 1:floor(n/2)
            tempoAcel = nAcel*dt;
            vel = dist/(tempoTotal-tempoAcel);
            acel = vel/tempoAcel;
            if vel <= v_max*(1+1e-10) && acel <= a_max*(1+1e-10)
                achouPerfil = true;
                break;
            end
        end
        if ~achouPerfil
            n = n+1;
        end
    end

    tempo = (0:n)*dt;
    s = zeros(size(tempo));
    v = zeros(size(tempo));
    a = zeros(size(tempo));
    for k = 1:numel(tempo)
        if tempo(k) <= tempoAcel+eps
            s(k) = 0.5*acel*tempo(k)^2;
            v(k) = acel*tempo(k);
            a(k) = acel;
        elseif tempo(k) < tempoTotal-tempoAcel-eps
            s(k) = 0.5*acel*tempoAcel^2 + vel*(tempo(k)-tempoAcel);
            v(k) = vel;
        else
            restante = tempoTotal-tempo(k);
            s(k) = dist-0.5*acel*restante^2;
            v(k) = acel*restante;
            a(k) = -acel;
        end
    end
    s(1) = 0;
    s(end) = dist;
    v([1 end]) = 0;
end

%  Din�mica inversa: torque necess�rio para (q, qd, qdd) ao longo do tempo
function tau_traj = calcularTorques(robot, q_traj, qd_traj, qdd_traj)
    numDoF   = size(q_traj,1);
    numSteps = size(q_traj,2);
    tau_traj = zeros(numDoF, numSteps);

    estadoAvisos = warning('off','all');
    for i = 1:numSteps
        tau_traj(:,i) = inverseDynamics(robot, q_traj(:,i)', qd_traj(:,i)', qdd_traj(:,i)');
    end
    warning(estadoAvisos);
end

%  An�lise textual dos torques calculados pela din�mica inversa
function analisarTorques(tau_traj, tvec, numDoF)
    fprintf('\n--- An�lise dos Torques (Din�mica Inversa) ---\n');
    jNames = {'J1','J2','J3','J4','J5','J6'};
    for j = 1:numDoF
        tauMax = max(abs(tau_traj(j,:)));
        tauRMS = rms(tau_traj(j,:));
        [~, idxMax] = max(abs(tau_traj(j,:)));
        fprintf('%s: torque m�x = %6.2f Nm (em t = %5.2f s)  |  RMS = %6.2f Nm\n', ...
            jNames{j}, tauMax, tvec(idxMax), tauRMS);
    end
    fprintf('-----------------------------------------------\n\n');
end

%  Controle PD + compensa��o de gravidade (feedforward)
% Integracao em passo fixo, com dois sub-passos em cada amostra.
function [qSim, qdSim, tauSim] = controlePDGravidade(robot, tvec, q_traj, qd_traj, Kp, Kd, tauLimite)
    numDoF   = size(q_traj,1);
    numSteps = numel(tvec);
    dt       = tvec(2) - tvec(1);

    nSub = 2;
    hSub = dt / nSub;

    qSim   = nan(numDoF, numSteps);
    qdSim  = nan(numDoF, numSteps);
    tauSim = nan(numDoF, numSteps);

    qSim(:,1)  = q_traj(:,1);
    qdSim(:,1) = qd_traj(:,1);

    for i = 1:numSteps-1
        q = qSim(:,i);
        qd = qdSim(:,i);
        qref  = q_traj(:,i);
        qdref = qd_traj(:,i);

        for s = 1:nSub
            G = gravityTorque(robot, q')';
            tau = Kp.*(qref-q) + Kd.*(qdref-qd) + G;
            tau = max(min(tau, tauLimite), -tauLimite);
            qdd = forwardDynamics(robot, q', qd', tau')';
            qd = qd + hSub*qdd;
            q = q + hSub*qd;
        end

        qSim(:,i+1)  = q;
        qdSim(:,i+1) = qd;
        tauSim(:,i) = tau;
    end
    tauSim(:,end) = tauSim(:,end-1);
end

function [qSim, qdSim, tauSim] = controleTorqueComputado(robot, tvec, q_traj, qd_traj, qdd_traj, wn, zeta, tauLimite)
    numDoF = size(q_traj,1);
    numSteps = numel(tvec);
    dt = tvec(2)-tvec(1);
    nSub = 2;
    hSub = dt/nSub;

    qSim = nan(numDoF,numSteps);
    qdSim = nan(numDoF,numSteps);
    tauSim = nan(numDoF,numSteps);
    qSim(:,1) = q_traj(:,1);
    qdSim(:,1) = qd_traj(:,1);

    for i = 1:numSteps-1
        q = qSim(:,i);
        qd = qdSim(:,i);
        for s = 1:nSub
            erro = q_traj(:,i)-q;
            erroVel = qd_traj(:,i)-qd;
            acelDesejada = qdd_traj(:,i) + 2*zeta*wn*erroVel + wn^2*erro;
            tau = inverseDynamics(robot, q', qd', acelDesejada')';
            tau = max(min(tau,tauLimite),-tauLimite);
            qdd = forwardDynamics(robot,q',qd',tau')';
            qd = qd+hSub*qdd;
            q = q+hSub*qd;
        end
        qSim(:,i+1) = q;
        qdSim(:,i+1) = qd;
        tauSim(:,i) = tau;
    end
    tauSim(:,end) = tauSim(:,end-1);
end

%  Um passo de integra��o RK4 da din�mica direta (torque constante no passo)
function [qNext, qdNext] = rk4StepDinamica(robot, q, qd, tau, h)
    f = @(qq, qqd) forwardDynamics(robot, qq', qqd', tau')';

    k1q = qd;                 k1qd = f(q, qd);
    k2q = qd + (h/2)*k1qd;    k2qd = f(q + (h/2)*k1q, qd + (h/2)*k1qd);
    k3q = qd + (h/2)*k2qd;    k3qd = f(q + (h/2)*k2q, qd + (h/2)*k2qd);
    k4q = qd + h*k3qd;        k4qd = f(q + h*k3q,     qd + h*k3qd);

    qNext  = q  + (h/6)*(k1q  + 2*k2q  + 2*k3q  + k4q);
    qdNext = qd + (h/6)*(k1qd + 2*k2qd + 2*k3qd + k4qd);
end

%  Amostra o espa�o de juntas dentro dos limites do URDF e calcula a nuvem
%    de pontos alcan��veis pela ponta do aplicador (estimativa da area de trabalho)
function nuvem = calcularAreaTrabalho(robot, numAmostras)
    numDoF  = numel(homeConfiguration(robot));
    limites = zeros(numDoF, 2);

    jIdx = 0;
    for k = 1:numel(robot.Bodies)
        jt = robot.Bodies{k}.Joint;
        if ~strcmp(jt.Type, 'fixed')
            jIdx = jIdx + 1;
            lim = jt.PositionLimits;
            if any(isinf(lim)) || any(isnan(lim))
                lim = [-pi, pi];   % fallback para juntas sem limite definido no URDF
            end
            limites(jIdx,:) = lim;
        end
    end

    nuvem = zeros(3, numAmostras);
    estadoAvisos = warning('off','all');
    for i = 1:numAmostras
        qRand = limites(:,1) + rand(numDoF,1) .* (limites(:,2) - limites(:,1));
        T = getTransform(robot, qRand', 'paint_tcp');
        nuvem(:,i) = T(1:3,4);
    end
    warning(estadoAvisos);
end

%  Verifica se os pontos desejados da trajet�ria est�o dentro da nuvem
%    de alcance estimada (usando o casco/forma alfa da nuvem amostrada)
function verificarAlcancePontos(pontosDesejados, nuvem)
    basePos = [0;0;0];
    raios       = vecnorm(nuvem - basePos);
    raioMax     = max(raios);
    raioMin     = min(raios);
    raiosPontos = vecnorm(pontosDesejados - basePos);

    try
        shp = alphaShape(nuvem(1,:)', nuvem(2,:)', nuvem(3,:)', Inf); % Inf = casco convexo
        dentro = inShape(shp, pontosDesejados(1,:)', pontosDesejados(2,:)', pontosDesejados(3,:)');
    catch
        % Fallback: verifica��o simplificada por raio (casca esf�rica aproximada)
        dentro = (raiosPontos <= raioMax) & (raiosPontos >= raioMin);
    end

    fprintf('\n--- Verifica��o da �rea de Trabalho ---\n');
    fprintf('Alcance estimado (amostragem): %.3f m (m�n) a %.3f m (m�x) a partir da base\n', raioMin, raioMax);
    nFora = sum(~dentro);
    if nFora > 0
        fprintf(2, 'ATEN��O: %d de %d pontos da trajet�ria est�o FORA da �rea de trabalho estimada!\n', ...
            nFora, numel(dentro));
        idxFora = find(~dentro);
        for k = 1:min(10, numel(idxFora))
            p = pontosDesejados(:, idxFora(k));
            fprintf('  Ponto fora do alcance: [%.3f, %.3f, %.3f] m  (raio = %.3f m)\n', ...
                p(1), p(2), p(3), raiosPontos(idxFora(k)));
        end
        if numel(idxFora) > 10
            fprintf('  ... e mais %d ponto(s).\n', numel(idxFora) - 10);
        end
    else
        fprintf('Todos os %d pontos da trajet�ria est�o dentro da �rea de trabalho estimada.\n', numel(dentro));
    end
    fprintf('----------------------------------------\n\n');
end

%  Plota a nuvem de pontos da �rea de trabalho junto com a trajet�ria desejada
function plotAreaTrabalho(nuvem, pontosDesejados)
    bg = [0.07 0.07 0.10]; axC = [0.65 0.75 0.85];
    hF0 = figure('Name','TX90 � �rea de Trabalho (Workspace)', 'NumberTitle','off', ...
        'Color',bg,'Position',[40 40 700 600]);
    ax0 = axes('Parent',hF0);
    set(ax0,'Color',bg,'XColor',axC,'YColor',axC,'ZColor',axC, ...
        'GridColor',[0.22 0.27 0.32],'GridAlpha',0.6,'FontSize',9);
    hold(ax0,'on'); grid(ax0,'on'); view(ax0, 140, 22); axis(ax0,'equal');

    scatter3(ax0, nuvem(1,:), nuvem(2,:), nuvem(3,:), 4, ...
        [0.30 0.45 0.65], 'filled', 'MarkerFaceAlpha', 0.15, 'DisplayName','Alcance amostrado');
    plot3(ax0, pontosDesejados(1,:), pontosDesejados(2,:), pontosDesejados(3,:), ...
        'o', 'MarkerSize', 5, 'MarkerFaceColor',[1 0.4 0.2], 'MarkerEdgeColor','w', ...
        'DisplayName','Pontos da trajet�ria');

    xlabel(ax0,'X (m)','Color','w'); ylabel(ax0,'Y (m)','Color','w'); zlabel(ax0,'Z (m)','Color','w');
    title(ax0,'�rea de Trabalho Estimada vs. Trajet�ria Desejada', ...
        'Color','w','FontSize',12,'FontWeight','bold');
    legend(ax0,'Location','best','TextColor','w','Color',[0.12 0.12 0.18],'EdgeColor',[0.30 0.30 0.40]);
end

%  An�lise do desempenho do controlador (erro de rastreamento e torque)
function analisarControle(tvec, q_traj, qSim, tau_traj, tauSim, numDoF)
    jNames = {'J1','J2','J3','J4','J5','J6'};
    erro = q_traj - qSim;

    fprintf('\n--- An�lise do Controle PD + Compensa��o de Gravidade ---\n');
    for j = 1:numDoF
        erroMax = max(abs(erro(j,:)), [], 'omitnan');
        erroRMS = rms(erro(j,:), 'omitnan');
        tauMaxSim = max(abs(tauSim(j,:)), [], 'omitnan');
        fprintf('%s: erro m�x = %7.4f rad | erro RMS = %7.4f rad | torque aplicado m�x = %6.2f Nm\n', ...
            jNames{j}, erroMax, erroRMS, tauMaxSim);
    end

    diffTau = tau_traj - tauSim;
    rmsPorJunta = zeros(numDoF,1);
    for j = 1:numDoF
        rmsPorJunta(j) = rms(diffTau(j,:), 'omitnan');
    end
    fprintf('Diferen�a RMS (torque ideal - torque do controlador): %.3f Nm (m�dia entre juntas)\n', ...
        mean(rmsPorJunta, 'omitnan'));
    fprintf('-----------------------------------------------------------\n\n');
end

function plotPerfisJunta(tvec, q_traj, qd_traj, qdd_traj, numDoF)
    bg = [0.07 0.07 0.10]; axC = [0.65 0.75 0.85]; cmap = lines(numDoF);
    jNames = {'J1','J2','J3','J4','J5','J6'};
    hF2 = figure('Name','TX90 � Perfis de Junta', 'NumberTitle','off','Color',bg,'Position',[1260 40 680 760]);
    dados   = {q_traj,   qd_traj,   qdd_traj};
    titulos = {'Posi��o (rad)','Velocidade (rad/s)','Acelera��o (rad/s�)'};

    for s = 1:3
        ax_s = subplot(3,1,s,'Parent',hF2);
        set(ax_s,'Color',bg,'XColor',axC,'YColor',axC, 'GridColor',[0.22 0.27 0.32],'GridAlpha',0.6,'FontSize',9);
        hold(ax_s,'on'); grid(ax_s,'on');
        for j = 1:numDoF
            plot(ax_s, tvec, dados{s}(j,:),'LineWidth',1.8,'Color',cmap(j,:),'DisplayName',jNames{j});
        end
        ylabel(ax_s, titulos{s},'Color','w','FontSize',10);
        title(ax_s,  titulos{s},'Color','w','FontWeight','bold');
        if s == 3, xlabel(ax_s,'Tempo (s)','Color','w','FontSize',10); end
        legend(ax_s, jNames,'Location','best','TextColor','w','Color',[0.12 0.12 0.18],'EdgeColor',[0.30 0.30 0.40]);
    end
    sgtitle(hF2,'TX90 � Perfis de Junta (Troca de Ferramenta)','Color','w','FontSize',12,'FontWeight','bold');
end

%  Gr�fico Torque x Tempo (din�mica inversa) por junta
function plotTorques(tvec, tau_traj, numDoF)
    bg = [0.07 0.07 0.10]; axC = [0.65 0.75 0.85]; cmap = lines(numDoF);
    jNames = {'J1','J2','J3','J4','J5','J6'};

    hF3 = figure('Name','TX90 � Torques (Din�mica Inversa)', 'NumberTitle','off', ...
        'Color',bg,'Position',[40 800 900 480]);
    ax3 = axes('Parent',hF3);
    set(ax3,'Color',bg,'XColor',axC,'YColor',axC,'GridColor',[0.22 0.27 0.32],'GridAlpha',0.6,'FontSize',9);
    hold(ax3,'on'); grid(ax3,'on');
    for j = 1:numDoF
        plot(ax3, tvec, tau_traj(j,:), 'LineWidth', 1.8, 'Color', cmap(j,:), 'DisplayName', jNames{j});
    end
    xlabel(ax3,'Tempo (s)','Color','w','FontSize',10);
    ylabel(ax3,'Torque (Nm)','Color','w','FontSize',10);
    title(ax3,'Torque por Junta ao Longo da Trajet�ria (Din�mica Inversa)', ...
        'Color','w','FontWeight','bold');
    legend(ax3, jNames,'Location','best','TextColor','w','Color',[0.12 0.12 0.18],'EdgeColor',[0.30 0.30 0.40]);
end

%  Gr�fico comparando torque ideal vs torque do controlador, e erro de rastreamento
function plotControleResultados(tvec, q_traj, qSim, tau_traj, tauSim, numDoF)
    bg = [0.07 0.07 0.10]; axC = [0.65 0.75 0.85]; cmap = lines(numDoF);
    jNames = {'J1','J2','J3','J4','J5','J6'};
    erro = q_traj - qSim;

    hF4 = figure('Name','TX90 � Controle PD + Compensa��o de Gravidade', 'NumberTitle','off', ...
        'Color',bg,'Position',[960 800 900 760]);

    % Subplot 1: erro de rastreamento por junta
    ax4a = subplot(3,1,1,'Parent',hF4);
    set(ax4a,'Color',bg,'XColor',axC,'YColor',axC,'GridColor',[0.22 0.27 0.32],'GridAlpha',0.6,'FontSize',9);
    hold(ax4a,'on'); grid(ax4a,'on');
    for j = 1:numDoF
        plot(ax4a, tvec, erro(j,:), 'LineWidth', 1.6, 'Color', cmap(j,:), 'DisplayName', jNames{j});
    end
    ylabel(ax4a,'Erro (rad)','Color','w','FontSize',10);
    title(ax4a,'Erro de Rastreamento (q_{ref} - q_{sim})','Color','w','FontWeight','bold');
    legend(ax4a, jNames,'Location','best','TextColor','w','Color',[0.12 0.12 0.18],'EdgeColor',[0.30 0.30 0.40]);

    % Subplot 2: torque aplicado pelo controlador (PD + gravidade)
    ax4b = subplot(3,1,2,'Parent',hF4);
    set(ax4b,'Color',bg,'XColor',axC,'YColor',axC,'GridColor',[0.22 0.27 0.32],'GridAlpha',0.6,'FontSize',9);
    hold(ax4b,'on'); grid(ax4b,'on');
    for j = 1:numDoF
        plot(ax4b, tvec, tauSim(j,:), 'LineWidth', 1.6, 'Color', cmap(j,:), 'DisplayName', jNames{j});
    end
    ylabel(ax4b,'Torque (Nm)','Color','w','FontSize',10);
    title(ax4b,'Torque Aplicado pelo Controlador (PD + Gravidade)','Color','w','FontWeight','bold');
    legend(ax4b, jNames,'Location','best','TextColor','w','Color',[0.12 0.12 0.18],'EdgeColor',[0.30 0.30 0.40]);

    % Subplot 3: compara��o torque ideal (din�mica inversa) vs torque do controlador, junta a junta (m�dia das diferen�as)
    ax4c = subplot(3,1,3,'Parent',hF4);
    set(ax4c,'Color',bg,'XColor',axC,'YColor',axC,'GridColor',[0.22 0.27 0.32],'GridAlpha',0.6,'FontSize',9);
    hold(ax4c,'on'); grid(ax4c,'on');
    diffTau = tau_traj - tauSim;
    for j = 1:numDoF
        plot(ax4c, tvec, diffTau(j,:), 'LineWidth', 1.6, 'Color', cmap(j,:), 'DisplayName', jNames{j});
    end
    xlabel(ax4c,'Tempo (s)','Color','w','FontSize',10);
    ylabel(ax4c,' Torque (Nm)','Color','w','FontSize',10);
    title(ax4c,'Diferen�a: Torque Ideal (Din. Inversa)  Torque do Controlador','Color','w','FontWeight','bold');
    legend(ax4c, jNames,'Location','best','TextColor','w','Color',[0.12 0.12 0.18],'EdgeColor',[0.30 0.30 0.40]);

    sgtitle(hF4,'TX90 � Desempenho do Controle PD + Compensa��o de Gravidade','Color','w','FontSize',12,'FontWeight','bold');
end
