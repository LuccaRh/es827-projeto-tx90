% Estima a área de trabalho do manipulador por amostragem de Monte Carlo do
% espaço de juntas (dentro dos limites do URDF), verifica se os pontos
% desejados da trajetória são alcançáveis (casco convexo da nuvem) e plota
% a nuvem junto com a trajetória.
function nuvem = verificarAreaTrabalho(robo, pontosDesejados, numAmostras)
    arguments
        robo (1,1) rigidBodyTree
        pontosDesejados (3,:) double
        numAmostras (1,1) double {mustBePositive}
    end

    nuvem = amostrarAreaTrabalho(robo, numAmostras);
    verificarAlcancePontos(pontosDesejados, nuvem);
    plotarAreaTrabalho(nuvem, pontosDesejados);
end

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
    for amostra = 1:numAmostras
        qAleatorio = limites(:,1) + rand(numJuntas,1) .* (limites(:,2) - limites(:,1));
        pose = getTransform(robo, qAleatorio', 'paint_tcp');
        nuvem(:,amostra) = pose(1:3,4);
    end
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
    figWorkspace = visualizacao.criarFiguraEscura('TX90 – Área de Trabalho (Workspace)');
    eixo = visualizacao.estilizarEixoEscuro(axes('Parent', figWorkspace));
    view(eixo, 140, 22); axis(eixo, 'equal');

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
