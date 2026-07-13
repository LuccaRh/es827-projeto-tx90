% Anima o TX90 executando a trajetória em tempo (aproximadamente) real, com
% rastro colorido por cor de tinta e marcador da estação de troca. Frames
% atrasados são pulados para a animação não ficar em câmera lenta, e o
% 'FastUpdate' atualiza apenas as transformações dos corpos já desenhados.
function animarRobo(robo, qTraj, caminhoEfetuador, corTraj, tempo, coresRastro, corTransicao, pontoEstacao)
    arguments
        robo (1,1) rigidBodyTree
        qTraj double
        caminhoEfetuador (3,:) double
        corTraj (1,:) double
        tempo (1,:) double
        coresRastro (:,3) double
        corTransicao (1,1) double
        pontoEstacao (3,1) double
    end

    numPassos = numel(tempo);
    dt = tempo(2) - tempo(1);

    %% Setup da figura
    figAnim = figure('Name', 'TX90 – Troca de Ferramentas', ...
        'NumberTitle', 'off', 'Color', [0.07 0.07 0.10], ...
        'Units', 'normalized', 'Position', [0.05 0.08 0.62 0.80]);

    eixoAnim = axes('Parent', figAnim, 'Color', [0.07 0.07 0.10], 'XColor', [0.55 0.65 0.75], ...
        'YColor', [0.55 0.65 0.75], 'ZColor', [0.55 0.65 0.75], ...
        'GridColor', [0.20 0.25 0.30], 'GridAlpha', 0.55, 'FontSize', 10);

    hold(eixoAnim, 'on'); grid(eixoAnim, 'on'); view(eixoAnim, 140, 22); axis(eixoAnim, 'equal');
    xlim(eixoAnim, [-0.10, 1.10]); ylim(eixoAnim, [-0.80, 0.80]); zlim(eixoAnim, [0.00, 1.00]);

    camlight(eixoAnim, 'headlight'); camlight(eixoAnim, 'right'); lighting(eixoAnim, 'gouraud');

    colormap(eixoAnim, coresRastro);
    set(eixoAnim, 'CLim', [1 size(coresRastro,1)]);

    % Marcador da estação de troca
    plot3(eixoAnim, pontoEstacao(1), pontoEstacao(2), pontoEstacao(3), 's', 'MarkerSize', 16, ...
        'MarkerFaceColor', [0.3 0.3 0.3], 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);
    text(eixoAnim, pontoEstacao(1), pontoEstacao(2), pontoEstacao(3)+0.12, 'Estação de Cores', ...
        'Color', [0.8 0.8 0.9], 'HorizontalAlignment', 'center', 'FontSize', 9, 'FontWeight', 'bold');

    % Rastro da pintura e marcador do TCP
    rastro = scatter3(eixoAnim, NaN, NaN, NaN, 18, NaN, 'filled', 'MarkerEdgeColor', 'none');
    marcadorTCP = plot3(eixoAnim, NaN, NaN, NaN, 'o', 'MarkerSize', 8, ...
        'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'k', 'LineWidth', 1.2);

    xlabel(eixoAnim, 'X (m)', 'Color', 'w'); ylabel(eixoAnim, 'Y (m)', 'Color', 'w'); zlabel(eixoAnim, 'Z (m)', 'Color', 'w');
    tituloAnim = title(eixoAnim, 'TX90  |  Iniciando...', 'Color', [0.95 0.95 1.0], 'FontSize', 13, 'FontWeight', 'bold');

    %% Loop principal
    show(robo, qTraj(:,1)', 'Parent', eixoAnim, 'Visuals', 'on', 'Frames', 'off', ...
        'PreservePlot', false, 'FastUpdate', true);

    tInicio = tic;
    for passo = 1:numPassos
        if ~isvalid(figAnim), break; end
        if toc(tInicio) > tempo(passo) + dt*1.5, continue; end   % pula frames atrasados

        show(robo, qTraj(:,passo)', 'Parent', eixoAnim, 'Visuals', 'on', 'Frames', 'off', ...
            'PreservePlot', false, 'FastUpdate', true);

        set(marcadorTCP, 'XData', caminhoEfetuador(1,passo), 'YData', caminhoEfetuador(2,passo), ...
            'ZData', caminhoEfetuador(3,passo));
        set(rastro, 'XData', caminhoEfetuador(1,1:passo), 'YData', caminhoEfetuador(2,1:passo), ...
            'ZData', caminhoEfetuador(3,1:passo), 'CData', corTraj(1:passo));

        if corTraj(passo) == corTransicao
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
end
