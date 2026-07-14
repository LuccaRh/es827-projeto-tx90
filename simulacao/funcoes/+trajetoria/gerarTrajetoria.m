% Gera a trajetória cartesiana completa da pintura da bandeira calculando
% os pontos de cada forma (retângulo, losango, círculo e faixa branca).
% Caso a opção de preenchimento esteja ativada, cada forma é preenchida em serpentina
% com espaçamento definido pelo passoLeque. Caso contrário, apenas o contorno
% de cada forma é percorrido. A faixa branca é sempre preenchida, mas pode ser
function traj = gerarTrajetoria(geometria, perfis, codigosCor)
    arguments
        geometria (1,1) struct
        perfis (1,1) struct
        codigosCor (1,1) struct
    end

    %% Waypoints cartesianos dos contornos das formas da bandeira
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

    %% Pinceladas de cada forma (contorno OU preenchimento em serpentina)
    dt = perfis.dt;
    estacao = geometria.pontoEstacao;
    preencher = isfield(perfis, 'preencher') && perfis.preencher;

    if preencher
        passo = perfis.passoLeque;
        % Meia-largura de cada forma em função da altura z (offset do centro):
        limiteRet = @(z) metadeLargRet;                                   % retângulo: largura constante
        limiteLos = @(z) metadeLargLos * max(0, 1 - abs(z)/metadeAltLos); % losango: estreita linearmente
        limiteCir = @(z) sqrt(max(0, geometria.raioCirculo^2 - z^2));     % círculo: semicorda

        wRet = rasterizarForma(geometria.centroBandeira, limiteRet, -metadeAltRet, metadeAltRet, passo);
        wLos = rasterizarForma(geometria.centroBandeira, limiteLos, -metadeAltLos, metadeAltLos, passo);
        wCir = rasterizarForma(geometria.centroBandeira, limiteCir, -geometria.raioCirculo, geometria.raioCirculo, passo);

        [pR, cR, vR, aR] = interpolarCartesianaConstante(wRet, perfis.velPintura, perfis.acelPintura, dt, codigosCor.verde);
        [pL, cL, vL, aL] = interpolarCartesianaConstante(wLos, perfis.velPintura, perfis.acelPintura, dt, codigosCor.amarelo);
        [pC, cC, vC, aC] = interpolarCartesianaConstante(wCir, perfis.velPintura, perfis.acelPintura, dt, codigosCor.azul);
        [pA, cA, vA, aA] = preencherFaixa(geometria, perfis, anguloInicialArco, anguloFinalArco, codigosCor.branco);
    else
        [pR, cR, vR, aR] = interpolarCartesianaConstante(pontosRetangulo, perfis.velPintura, perfis.acelPintura, dt, codigosCor.verde);
        [pL, cL, vL, aL] = interpolarCartesianaConstante(pontosLosango, perfis.velPintura, perfis.acelPintura, dt, codigosCor.amarelo);
        [pC, cC, vC, aC] = interpolarArco(geometria.centroBandeira, geometria.raioCirculo, pi/2, -3*pi/2, perfis.velPintura, perfis.acelPintura, dt, codigosCor.azul);
        [pA, cA, vA, aA] = interpolarArco(geometria.centroArco, geometria.raioArco, anguloInicialArco, anguloFinalArco, perfis.velPintura, perfis.acelPintura, dt, codigosCor.branco);
    end

    %% Empilhamento das camadas de tinta (evita conflito de profundidade)
    espessura = 0;
    if isfield(perfis, 'espessuraCamada'), espessura = perfis.espessuraCamada; end
    pL(1,:) = pL(1,:) - 1*espessura;   % losango sobre o retângulo
    pC(1,:) = pC(1,:) - 2*espessura;   % círculo sobre o losango
    pA(1,:) = pA(1,:) - 3*espessura;   % faixa branca sobre tudo

    % Pontos representativos (já com o deslocamento) p/ verificação de alcance
    amostrar = @(P) P(:, unique(round(linspace(1, size(P,2), min(size(P,2), 60)))));
    pontosDesejados = [amostrar(pR), amostrar(pL), amostrar(pC), amostrar(pA), estacao];

    %% Construção da trajetória (idas e vindas à estação, entre cada forma)
    % Estação -> retângulo -> estação
    [pT1, cT1, vT1, aT1] = interpolarTransicao(estacao, pR(:,1),   perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    [pT2, cT2, vT2, aT2] = interpolarTransicao(pR(:,end), estacao, perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    % Estação -> losango -> estação
    [pT3, cT3, vT3, aT3] = interpolarTransicao(estacao, pL(:,1),   perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    [pT4, cT4, vT4, aT4] = interpolarTransicao(pL(:,end), estacao, perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    % Estação -> círculo -> estação
    [pT5, cT5, vT5, aT5] = interpolarTransicao(estacao, pC(:,1),   perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    [pT6, cT6, vT6, aT6] = interpolarTransicao(pC(:,end), estacao, perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    % Estação -> faixa branca -> estação
    [pT7, cT7, vT7, aT7] = interpolarTransicao(estacao, pA(:,1),   perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);
    [pT8, cT8, vT8, aT8] = interpolarTransicao(pA(:,end), estacao, perfis.velTransicao, perfis.acelTransicao, dt, codigosCor.transicao);

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

    traj = struct('posCart', posCart, 'cor', corTraj, 'vel', velCart, ...
        'acel', acelCart, 'tempo', tempo, 'pontosDesejados', pontosDesejados);
end
