% Gera a trajetória cartesiana completa da pintura da bandeira: contornos
% das formas (retângulo, losango, círculo e arco) intercalados com idas e
% vindas à estação de troca de cores, todos com perfil temporal LSPB.
%
% geometria:  centroBandeira, pontoEstacao, larguraRetangulo, alturaRetangulo,
%             larguraLosango, alturaLosango, raioCirculo, centroArco, raioArco
% perfis:     dt, velPintura, acelPintura, velTransicao, acelTransicao
% codigosCor: verde, transicao, amarelo, azul, branco
%
% Retorna traj com: posCart (3xN), cor (1xN), vel (1xN), acel (1xN),
% tempo (1xN) e pontosDesejados (waypoints para verificação de alcance).
function traj = gerarTrajetoria(geometria, perfis, codigosCor)
    arguments
        geometria (1,1) struct
        perfis (1,1) struct
        codigosCor (1,1) struct
    end

    %% Waypoints cartesianos das formas da bandeira
    % Retângulo (contorno externo, verde)
    metadeLargRet = geometria.larguraRetangulo/2;
    metadeAltRet  = geometria.alturaRetangulo/2;
    pontosRetangulo = geometria.centroBandeira + [ ...
        0              0             0              0             0;
       -metadeLargRet  metadeLargRet metadeLargRet -metadeLargRet -metadeLargRet;
        metadeAltRet   metadeAltRet -metadeAltRet  -metadeAltRet   metadeAltRet ];

    % Losango (amarelo)
    metadeLargLos = geometria.larguraLosango/2;
    metadeAltLos  = geometria.alturaLosango/2;
    pontosLosango = geometria.centroBandeira + [ ...
        0             0              0             0              0;
        0             metadeLargLos  0            -metadeLargLos  0;
        metadeAltLos  0             -metadeAltLos  0              metadeAltLos ];

    % Círculo (azul)
    anguloCirculo = linspace(pi/2, -3*pi/2, 40);
    pontosCirculo = geometria.centroBandeira + [ zeros(1,40);
        geometria.raioCirculo*cos(anguloCirculo);
        geometria.raioCirculo*sin(anguloCirculo) ];

    % Arco (faixa branca) — ângulos inicial e final derivados da geometria
    anguloInicialArco = atan2(0.28483, -0.11745 + 0.07);
    anguloFinalArco   = atan2(0.21877,  0.11845 + 0.07);
    anguloArco = linspace(anguloInicialArco, anguloFinalArco, 40);
    pontosArco = geometria.centroArco + [ zeros(1,40);
        geometria.raioArco*cos(anguloArco);
        geometria.raioArco*sin(anguloArco) ];

    %% Construção da trajetória (idas e vindas à estação)
    estacao = geometria.pontoEstacao;
    dt = perfis.dt;

    % Estação -> retângulo -> estação
    [pT1, cT1, vT1, aT1] = interpolarTransicao(estacao, pontosRetangulo(:,1), perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    [pR,  cR,  vR,  aR ] = interpolarCartesianaConstante(pontosRetangulo, perfis.velPintura, perfis.acelPintura, dt, codigosCor.verde);
    [pT2, cT2, vT2, aT2] = interpolarTransicao(pontosRetangulo(:,end), estacao, perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);

    % Estação -> losango -> estação
    [pT3, cT3, vT3, aT3] = interpolarTransicao(estacao, pontosLosango(:,1), perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    [pL,  cL,  vL,  aL ] = interpolarCartesianaConstante(pontosLosango, perfis.velPintura, perfis.acelPintura, dt, codigosCor.amarelo);
    [pT4, cT4, vT4, aT4] = interpolarTransicao(pontosLosango(:,end), estacao, perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);

    % Estação -> círculo -> estação
    [pT5, cT5, vT5, aT5] = interpolarTransicao(estacao, pontosCirculo(:,1), perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    [pC,  cC,  vC,  aC ] = interpolarArco(geometria.centroBandeira, geometria.raioCirculo, pi/2, -3*pi/2, perfis.velPintura, perfis.acelPintura, dt, codigosCor.azul);
    [pT6, cT6, vT6, aT6] = interpolarTransicao(pontosCirculo(:,1), estacao, perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);

    % Estação -> arco -> estação (guarda a ferramenta)
    [pT7, cT7, vT7, aT7] = interpolarTransicao(estacao, pontosArco(:,1), perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    [pA,  cA,  vA,  aA ] = interpolarArco(geometria.centroArco, geometria.raioArco, anguloInicialArco, anguloFinalArco, perfis.velPintura, perfis.acelPintura, dt, codigosCor.branco);
    [pT8, cT8, vT8, aT8] = interpolarTransicao(pontosArco(:,end), estacao, perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);

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

    tempo = (0:size(posCart,2)-1) * dt;
    pontosDesejados = [pontosRetangulo, pontosLosango, pontosCirculo, pontosArco, estacao];

    traj = struct('posCart', posCart, 'cor', corTraj, 'vel', velCart, ...
        'acel', acelCart, 'tempo', tempo, 'pontosDesejados', pontosDesejados);
end

%% Funções locais de interpolação
function [pontos, cores, vel, acel] = interpolarCartesianaConstante(pontosApoio, velMax, acelMax, dt, codigoCor)
    numSegmentos = size(pontosApoio, 2) - 1;
    segPontos = cell(1, numSegmentos);
    segVel    = cell(1, numSegmentos);
    segAcel   = cell(1, numSegmentos);
    for seg = 1:numSegmentos
        delta = pontosApoio(:,seg+1) - pontosApoio(:,seg);
        distancia = norm(delta);
        [s, v, a] = perfilLSPB(distancia, velMax, acelMax, dt);
        pontosSeg = pontosApoio(:,seg) + (delta/distancia)*s;
        if seg > 1   % remove o 1º ponto (repetido do fim do segmento anterior)
            pontosSeg = pontosSeg(:,2:end);
            v = v(2:end);
            a = a(2:end);
        end
        segPontos{seg} = pontosSeg;
        segVel{seg}    = v;
        segAcel{seg}   = a;
    end
    pontos = [segPontos{:}];
    vel    = [segVel{:}];
    acel   = [segAcel{:}];
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
    arguments
        distancia (1,1) double {mustBePositive}
        velMax (1,1) double {mustBePositive}
        acelMax (1,1) double {mustBePositive}
        dt (1,1) double {mustBePositive}
    end

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
