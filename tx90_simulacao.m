%% Stäubli TX90 - simulação validada do projeto ES827
% Trajetória temporal LSPB, IK validada, dinâmica inversa, limites físicos,
% controle PD + gravidade e controle por torque computado.
%
% Requisitos: Robotics System Toolbox.
% Execute este arquivo a partir da pasta TX90_v3_DEV.

clearvars; close all; clc;

%% Configuração reproduzível
DT = 0.04;                  % passo de amostragem [s]
V_DRAW = 0.12;             % limite de velocidade durante pintura [m/s]
A_DRAW = 0.30;             % limite de aceleração durante pintura [m/s^2]
V_TRANSIT = 0.35;          % limite de velocidade em movimento livre [m/s]
A_TRANSIT = 0.60;          % limite de aceleração em movimento livre [m/s^2]

TCP_LENGTH = 0.15;         % flange até ponta do aplicador [m]
PAYLOAD_MASS = 3.0;        % cenário de projeto; substituir por valor medido [kg]
PAYLOAD_COM_FROM_FLANGE = 0.075; % centro de massa ao longo de +Z da ferramenta [m]
PAYLOAD_RADIUS = 0.04;     % aproximação cilíndrica para tensor de inércia [m]

POSITION_TOL = 2.0e-4;     % tolerância de posição da IK [m]
ORIENTATION_TOL = 2.0e-3;  % tolerância de orientação da IK [rad]
WN = 5.0;                  % frequência natural dos controladores [rad/s]
ZETA = 1.0;                % amortecimento crítico

RUN_WORKSPACE_PLOT = true;
RUN_ANIMATION = false;     % false para execução rápida e reprodutível
WORKSPACE_SAMPLES = 20000;

OUTPUT_DIR = fullfile(pwd, 'validation_outputs_matlab');
if ~exist(OUTPUT_DIR, 'dir'), mkdir(OUTPUT_DIR); end
rng(827, 'twister');

%% Modelo do robô e ferramenta
robot = importrobot('tx90.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

paintTCP = rigidBody('paint_tcp');
paintJoint = rigidBodyJoint('paint_tcp_fixed', 'fixed');
setFixedTransform(paintJoint, trvec2tform([0 0 TCP_LENGTH]));
paintTCP.Joint = paintJoint;
paintTCP.Mass = PAYLOAD_MASS;
paintTCP.CenterOfMass = [0 0 -(TCP_LENGTH - PAYLOAD_COM_FROM_FLANGE)];
Itrans = PAYLOAD_MASS * (3*PAYLOAD_RADIUS^2 + TCP_LENGTH^2) / 12;
Iaxial = 0.5 * PAYLOAD_MASS * PAYLOAD_RADIUS^2;
paintTCP.Inertia = [Itrans Itrans Iaxial 0 0 0];
addBody(robot, paintTCP, 'tool0');

numDoF = numel(homeConfiguration(robot));
if numDoF ~= 6, error('Esperados 6 GDL; encontrados %d.', numDoF); end

jointLower = [-pi; deg2rad(-130); deg2rad(-145); deg2rad(-270); deg2rad(-115); deg2rad(-270)];
jointUpper = [ pi; deg2rad(147.5); deg2rad(145); deg2rad(270); deg2rad(140); deg2rad(270)];
velocityLimit = deg2rad([400; 400; 430; 540; 475; 760]);
effortLimit = [318; 166; 76; 34; 29; 11];

%% Trajetória cartesiana com perfil temporal coerente
traj = buildValidatedPath(DT, V_DRAW, A_DRAW, V_TRANSIT, A_TRANSIT);
posCart = traj.position;
tvec = traj.time;
numSteps = numel(tvec);

fprintf('Trajetória: %d pontos, duração %.2f s.\n', numSteps, tvec(end));
fprintf('Velocidade cartesiana comandada máxima: %.4f m/s.\n', max(traj.speed));
fprintf('Aceleração cartesiana comandada máxima: %.4f m/s^2.\n', max(abs(traj.accel)));

%% Cinemática inversa com verificação ponto a ponto
ik = inverseKinematics('RigidBodyTree', robot);
ikWeights = [0.5 0.5 0.5 1 1 1];
Rfixed = [0 0 1; 0 1 0; -1 0 0];

qTraj = zeros(numDoF, numSteps);
eePath = zeros(3, numSteps);
ikPosError = zeros(1, numSteps);
ikOriError = zeros(1, numSteps);
sigmaMin = zeros(1, numSteps);
jacCondition = zeros(1, numSteps);
ikStatus = strings(1, numSteps);

initialGuess = homeConfiguration(robot);
for i = 1:numSteps
    target = trvec2tform(posCart(:,i)') * rotm2tform(Rfixed);
    [qSol, info] = ik('paint_tcp', target, ikWeights, initialGuess);
    actual = getTransform(robot, qSol, 'paint_tcp');
    eePath(:,i) = actual(1:3,4);
    ikPosError(i) = norm(actual(1:3,4) - posCart(:,i));
    axang = rotm2axang(Rfixed * actual(1:3,1:3)');
    ikOriError(i) = abs(axang(4));

    J = geometricJacobian(robot, qSol, 'paint_tcp');
    sv = svd(J);
    sigmaMin(i) = min(sv);
    jacCondition(i) = max(sv) / max(min(sv), eps);
    if isfield(info, 'Status'), ikStatus(i) = string(info.Status); else, ikStatus(i) = "não informado"; end

    if ikPosError(i) > POSITION_TOL || ikOriError(i) > ORIENTATION_TOL
        error('Falha de IK no ponto %d: erro pos=%.3g m, ori=%.3g rad.', ...
            i, ikPosError(i), ikOriError(i));
    end
    if any(qSol' < jointLower - 1e-8) || any(qSol' > jointUpper + 1e-8)
        error('Solução de IK fora dos limites articulares no ponto %d.', i);
    end
    qTraj(:,i) = qSol';
    initialGuess = qSol;
end

%% Perfis articulares consistentes com q(t)
qdTraj = zeros(size(qTraj));
qddTraj = zeros(size(qTraj));
for j = 1:numDoF
    qdTraj(j,:) = gradient(qTraj(j,:), DT);
    qddTraj(j,:) = gradient(qdTraj(j,:), DT);
end

maxJointSpeed = max(abs(qdTraj), [], 2);
if any(maxJointSpeed > velocityLimit + 1e-8)
    error('A trajetória viola limites de velocidade articular.');
end

%% Dinâmica inversa e limites de esforço
tauTraj = zeros(numDoF, numSteps);
for i = 1:numSteps
    tauTraj(:,i) = inverseDynamics(robot, qTraj(:,i)', qdTraj(:,i)', qddTraj(:,i)')';
end
maxTorque = max(abs(tauTraj), [], 2);
rmsTorque = sqrt(mean(tauTraj.^2, 2));
if any(maxTorque > effortLimit + 1e-8)
    error('A trajetória viola limites de esforço do URDF.');
end

%% Controles com os mesmos limites físicos
M0 = massMatrix(robot, qTraj(:,1)');
KpPD = WN^2 * diag(M0);
KdPD = 2*ZETA*WN * diag(M0);

[qPD, qdPD, tauPD, satPD] = simulateController(robot, tvec, qTraj, qdTraj, qddTraj, ...
    KpPD, KdPD, effortLimit, 'pd_gravity', WN, ZETA);
[qCT, qdCT, tauCT, satCT] = simulateController(robot, tvec, qTraj, qdTraj, qddTraj, ...
    KpPD, KdPD, effortLimit, 'computed_torque', WN, ZETA);

metricsPD = controlMetrics(qTraj, qPD, tauPD, satPD);
metricsCT = controlMetrics(qTraj, qCT, tauCT, satCT);
if ~metricsPD.completed || ~metricsCT.completed
    error('Um dos controladores não completou a trajetória.');
end

%% Workspace: visualização; executabilidade é comprovada pela IK completa
if RUN_WORKSPACE_PLOT
    cloud = sampleWorkspace(robot, WORKSPACE_SAMPLES, jointLower, jointUpper);
    fWS = figure('Color','w','Name','Workspace TX90');
    scatter3(cloud(1,:), cloud(2,:), cloud(3,:), 2, [0.3 0.5 0.8], 'filled', ...
        'MarkerFaceAlpha', 0.08); hold on;
    plot3(posCart(1,:), posCart(2,:), posCart(3,:), 'r.', 'MarkerSize', 5);
    axis equal; grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Workspace amostrado e trajetória validada por IK');
    legend('Amostras do workspace', 'Trajetória', 'Location','best');
    exportgraphics(fWS, fullfile(OUTPUT_DIR, 'workspace_validado.png'), 'Resolution', 180);
end

%% Figuras e exportação de dados
plotValidatedResults(OUTPUT_DIR, tvec, posCart, traj, qTraj, qdTraj, qddTraj, ...
    tauTraj, effortLimit, qPD, qCT);

T = table(tvec', posCart(1,:)', posCart(2,:)', posCart(3,:)', traj.speed', ...
    traj.accel', traj.color', traj.spray', ikPosError', ikOriError', sigmaMin', jacCondition', ...
    'VariableNames', {'time_s','x_m','y_m','z_m','cart_speed_m_s','cart_accel_m_s2', ...
    'color','spray_on','ik_position_error_m','ik_orientation_error_rad','sigma_min','jacobian_condition'});
for j = 1:numDoF
    T.(sprintf('q_J%d_rad',j)) = qTraj(j,:)';
    T.(sprintf('qd_J%d_rad_s',j)) = qdTraj(j,:)';
    T.(sprintf('qdd_J%d_rad_s2',j)) = qddTraj(j,:)';
    T.(sprintf('tau_J%d_Nm',j)) = tauTraj(j,:)';
end
writetable(T, fullfile(OUTPUT_DIR, 'tx90_resultados_validados_matlab.csv'));

summary = struct;
summary.configuration = struct('dt_s',DT,'draw_speed_m_s',V_DRAW,'draw_accel_m_s2',A_DRAW, ...
    'transit_speed_m_s',V_TRANSIT,'transit_accel_m_s2',A_TRANSIT, ...
    'tcp_length_m',TCP_LENGTH,'payload_mass_kg',PAYLOAD_MASS);
summary.trajectory = struct('points',numSteps,'duration_s',tvec(end), ...
    'max_cart_speed_m_s',max(traj.speed),'max_cart_accel_m_s2',max(abs(traj.accel)), ...
    'spray_time_s',sum(traj.spray)*DT,'transit_time_s',sum(~traj.spray)*DT);
summary.kinematics = struct('ik_failures',0,'max_position_error_m',max(ikPosError), ...
    'rms_position_error_m',sqrt(mean(ikPosError.^2)), ...
    'max_orientation_error_rad',max(ikOriError),'min_sigma',min(sigmaMin), ...
    'max_condition',max(jacCondition),'max_joint_speed_rad_s',maxJointSpeed', ...
    'joint_speed_limit_rad_s',velocityLimit');
summary.dynamics = struct('max_torque_Nm',maxTorque','rms_torque_Nm',rmsTorque', ...
    'effort_limit_Nm',effortLimit','violations',(maxTorque > effortLimit)');
summary.control = struct('pd_gravity',metricsPD,'computed_torque',metricsCT);

fid = fopen(fullfile(OUTPUT_DIR,'validation_summary_matlab.json'),'w');
fprintf(fid, '%s', jsonencode(summary, 'PrettyPrint', true)); fclose(fid);

fprintf('\n--- RESULTADOS VALIDADOS ---\n');
fprintf('Duração: %.2f s | pontos: %d\n', tvec(end), numSteps);
fprintf('Erro IK máximo: %.3f mm | orientação: %.3e rad\n', 1e3*max(ikPosError), max(ikOriError));
fprintf('Sigma mínimo do Jacobiano: %.3e\n', min(sigmaMin));
fprintf('Torque máximo por junta [Nm]: %s\n', mat2str(maxTorque',4));
fprintf('RMS erro PD+G [rad]: %s\n', mat2str(metricsPD.errorRMS,4));
fprintf('RMS erro torque computado [rad]: %s\n', mat2str(metricsCT.errorRMS,4));

if RUN_ANIMATION
    animateRobot(robot, qTraj, eePath, traj, tvec);
end

%% Funções locais
function traj = buildValidatedPath(dt, vDraw, aDraw, vTransit, aTransit)
    X = 0.65; Y = 0; Z = 0.45;
    pTool = [0.50; 0.60; 0.20];
    rect = [X X X X; -0.35 0.35 0.35 -0.35; 0.70 0.70 0.20 0.20];
    diamond = [X X X X; 0 0.581/2 0 -0.581/2; Z+0.381/2 Z Z-0.381/2 Z];
    rC = 0.1225;
    centerC = [X;Y;Z];
    centerA = [0.65;-0.07;0.20]; rA = 0.28875;
    th0 = atan2(0.28483, -0.11745+0.07);
    thF = atan2(0.21877, 0.11845+0.07);
    pArc0 = centerA + [0; rA*cos(th0); rA*sin(th0)];
    pArcF = centerA + [0; rA*cos(thF); rA*sin(thF)];

    b = emptyBuilder(dt);
    b = appendLine(b,pTool,rect(:,1),vTransit,aTransit,0,false);
    for k=1:4, b=appendLine(b,rect(:,k),rect(:,mod(k,4)+1),vDraw,aDraw,1,true); end
    b = appendLine(b,rect(:,1),pTool,vTransit,aTransit,0,false);
    b = appendLine(b,pTool,diamond(:,1),vTransit,aTransit,0,false);
    for k=1:4, b=appendLine(b,diamond(:,k),diamond(:,mod(k,4)+1),vDraw,aDraw,2,true); end
    b = appendLine(b,diamond(:,1),pTool,vTransit,aTransit,0,false);
    pCircle0 = centerC + [0;0;rC];
    b = appendLine(b,pTool,pCircle0,vTransit,aTransit,0,false);
    b = appendArc(b,centerC,rC,pi/2,-3*pi/2,vDraw,aDraw,3,true);
    b = appendLine(b,pCircle0,pTool,vTransit,aTransit,0,false);
    b = appendLine(b,pTool,pArc0,vTransit,aTransit,0,false);
    b = appendArc(b,centerA,rA,th0,thF,vDraw,aDraw,4,true);
    b = appendLine(b,pArcF,pTool,vTransit,aTransit,0,false);
    traj = struct('time',(0:size(b.position,2)-1)*dt,'position',b.position, ...
        'speed',b.speed,'accel',b.accel,'color',b.color,'spray',logical(b.spray));
end

function b = emptyBuilder(dt)
    b=struct('dt',dt,'position',zeros(3,0),'speed',zeros(1,0), ...
        'accel',zeros(1,0),'color',zeros(1,0),'spray',zeros(1,0));
end

function b = appendLine(b,p0,p1,vmax,amax,color,spray)
    delta=p1-p0; L=norm(delta); dir=delta/L;
    [s,sd,sdd]=lspbProfile(L,vmax,amax,b.dt);
    pts=p0+dir*s;
    b=appendSamples(b,pts,sd,sdd,color,spray);
end

function b = appendArc(b,c,r,th0,th1,vmax,amax,color,spray)
    dth=th1-th0; L=abs(dth)*r; sg=sign(dth);
    [s,sd,sdd]=lspbProfile(L,vmax,amax,b.dt);
    th=th0+sg*s/r;
    pts=[c(1)*ones(size(th)); c(2)+r*cos(th); c(3)+r*sin(th)];
    b=appendSamples(b,pts,sd,sdd,color,spray);
end

function b = appendSamples(b,pts,sd,sdd,color,spray)
    first=1+~isempty(b.position);
    idx=first:numel(sd);
    b.position=[b.position pts(:,idx)]; b.speed=[b.speed sd(idx)];
    b.accel=[b.accel sdd(idx)]; b.color=[b.color color*ones(1,numel(idx))];
    b.spray=[b.spray spray*ones(1,numel(idx))];
end

function [s,sd,sdd] = lspbProfile(L,vmax,amax,dt)
    if L <= vmax^2/amax, Tmin=2*sqrt(L/amax); else, Tmin=L/vmax+vmax/amax; end
    n=max(2,ceil(Tmin/dt)); found=false;
    while ~found
        T=n*dt;
        for na=1:floor(n/2)
            ta=na*dt; v=L/(T-ta); a=v/ta;
            if v<=vmax*(1+1e-10) && a<=amax*(1+1e-10), found=true; break; end
        end
        if ~found, n=n+1; end
    end
    t=(0:n)*dt; s=zeros(size(t)); sd=s; sdd=s;
    for k=1:numel(t)
        if t(k)<=ta+eps, s(k)=0.5*a*t(k)^2; sd(k)=a*t(k); sdd(k)=a;
        elseif t(k)<T-ta-eps, s(k)=0.5*a*ta^2+v*(t(k)-ta); sd(k)=v;
        else, rem=T-t(k); s(k)=L-0.5*a*rem^2; sd(k)=a*rem; sdd(k)=-a; end
    end
    s(1)=0; s(end)=L; sd([1 end])=0;
end

function [qSim,qdSim,tauSim,saturated] = simulateController(robot,tvec,qRef,qdRef,qddRef,Kp,Kd,limit,mode,wn,zeta)
    n=numel(tvec); dof=size(qRef,1); dt=tvec(2)-tvec(1); nSub=2; h=dt/nSub;
    qSim=nan(dof,n); qdSim=nan(dof,n); tauSim=nan(dof,n); saturated=false(dof,n);
    qSim(:,1)=qRef(:,1); qdSim(:,1)=qdRef(:,1);
    for i=1:n-1
        q=qSim(:,i); qd=qdSim(:,i);
        for s=1:nSub
            e=qRef(:,i)-q; ed=qdRef(:,i)-qd;
            if strcmp(mode,'pd_gravity')
                G=gravityTorque(robot,q')'; tauCmd=Kp.*e+Kd.*ed+G;
            else
                accCmd=qddRef(:,i)+2*zeta*wn*ed+wn^2*e;
                tauCmd=inverseDynamics(robot,q',qd',accCmd')';
            end
            tau=max(min(tauCmd,limit),-limit);
            saturated(:,i)=saturated(:,i) | abs(tauCmd)>limit+1e-9;
            qdd=forwardDynamics(robot,q',qd',tau')';
            qd=qd+h*qdd; q=q+h*qd;
        end
        qSim(:,i+1)=q; qdSim(:,i+1)=qd; tauSim(:,i)=tau;
        if any(~isfinite(q)), return; end
    end
    tauSim(:,end)=tauSim(:,end-1); saturated(:,end)=saturated(:,end-1);
end

function m = controlMetrics(qRef,qSim,tau,sat)
    err=qRef-qSim;
    m=struct('errorMax',max(abs(err),[],2)','errorRMS',sqrt(mean(err.^2,2))', ...
        'torqueMax',max(abs(tau),[],2)','saturationPercent',100*mean(sat,2)', ...
        'completed',all(isfinite(qSim(:,end))));
end

function cloud = sampleWorkspace(robot,n,lower,upper)
    cloud=zeros(3,n);
    for i=1:n
        q=(lower+rand(6,1).*(upper-lower))';
        T=getTransform(robot,q,'paint_tcp'); cloud(:,i)=T(1:3,4);
    end
end

function plotValidatedResults(out,t,pos,traj,q,qd,qdd,tau,effort,qPD,qCT)
    c=[.45 .45 .5; 0 .65 .25; 1 .78 0; 0 .35 .9; .85 .85 .85];
    f1=figure('Color','w'); hold on;
    for k=0:4, mask=traj.color==k; scatter(pos(2,mask),pos(3,mask),8,c(k+1,:),'filled'); end
    axis equal; grid on; xlabel('Y (m)'); ylabel('Z (m)'); title('Trajetória cartesiana validada');
    legend('Transição','Verde','Amarelo','Azul','Branco','Location','best');
    exportgraphics(f1,fullfile(out,'trajetoria_validada_matlab.png'),'Resolution',180);

    f2=figure('Color','w');
    subplot(3,1,1); plot(t,q'); grid on; ylabel('q (rad)'); legend('J1','J2','J3','J4','J5','J6');
    subplot(3,1,2); plot(t,qd'); grid on; ylabel('dq/dt (rad/s)');
    subplot(3,1,3); plot(t,qdd'); grid on; ylabel('d2q/dt2'); xlabel('Tempo (s)');
    exportgraphics(f2,fullfile(out,'perfis_articulares_matlab.png'),'Resolution',180);

    f3=figure('Color','w');
    for j=1:6
        subplot(3,2,j); plot(t,tau(j,:)); hold on; yline(effort(j),'r--'); yline(-effort(j),'r--');
        grid on; title(sprintf('J%d',j)); ylabel('Nm');
    end
    exportgraphics(f3,fullfile(out,'torques_limites_matlab.png'),'Resolution',180);

    f4=figure('Color','w'); ePD=q-qPD; eCT=q-qCT;
    for j=1:6
        subplot(3,2,j); plot(t,ePD(j,:),t,eCT(j,:)); grid on; title(sprintf('J%d',j)); ylabel('Erro (rad)');
    end
    legend('PD + gravidade','Torque computado');
    exportgraphics(f4,fullfile(out,'comparacao_controle_matlab.png'),'Resolution',180);
end

function animateRobot(robot,q,ee,traj,t)
    f=figure('Color',[.07 .07 .1]); ax=axes(f); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,140,22);
    trail=scatter3(ax,nan,nan,nan,18,nan,'filled'); cmap=[.45 .45 .5;0 .8 .3;1 .85 0;0 .5 1;1 1 1]; colormap(ax,cmap); clim(ax,[0 4]);
    tic;
    for i=1:numel(t)
        show(robot,q(:,i)','Parent',ax,'Visuals','on','Frames','off','PreservePlot',false);
        set(trail,'XData',ee(1,1:i),'YData',ee(2,1:i),'ZData',ee(3,1:i),'CData',traj.color(1:i));
        drawnow limitrate; rem=t(i)-toc; if rem>0, pause(rem); end
    end
end
