% Gera os gráficos de resultados da simulação: perfis articulares (posição,
% velocidade e aceleração), torques da dinâmica inversa e desempenho do
% controlador (erro de rastreamento e torques aplicados).
function plotarResultados(tempo, qTraj, qdTraj, qddTraj, tauTraj, qSim, tauSim, numJuntas)
    arguments
        tempo (1,:) double
        qTraj double
        qdTraj double
        qddTraj double
        tauTraj double
        qSim double
        tauSim double
        numJuntas (1,1) double {mustBePositive}
    end

    plotarPerfisJuntas(tempo, qTraj, qdTraj, qddTraj, numJuntas);
    plotarTorques(tempo, tauTraj, numJuntas);
    plotarControle(tempo, qTraj, qSim, tauTraj, tauSim, numJuntas);
end

function plotarPerfisJuntas(tempo, qTraj, qdTraj, qddTraj, numJuntas)
    mapaCores = lines(numJuntas);
    nomesJuntas = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6'};
    figPerfis = visualizacao.criarFiguraEscura('TX90 – Perfis de Junta');
    dados   = {qTraj, qdTraj, qddTraj};
    titulos = {'Posição (rad)', 'Velocidade (rad/s)', 'Aceleração (rad/s²)'};

    for grafico = 1:3
        eixoSub = visualizacao.estilizarEixoEscuro(subplot(3, 1, grafico, 'Parent', figPerfis));
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
    mapaCores = lines(numJuntas);
    nomesJuntas = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6'};

    figTorques = visualizacao.criarFiguraEscura('TX90 – Torques (Dinâmica Inversa)');
    eixo = visualizacao.estilizarEixoEscuro(axes('Parent', figTorques));
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
    mapaCores = lines(numJuntas);
    nomesJuntas = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6'};
    erro = qTraj - qSim;

    figControle = visualizacao.criarFiguraEscura('TX90 – Controle PD + Compensação de Gravidade');

    % Subplot 1: erro de rastreamento por junta
    eixoErro = visualizacao.estilizarEixoEscuro(subplot(3, 1, 1, 'Parent', figControle));
    for junta = 1:numJuntas
        plot(eixoErro, tempo, erro(junta,:), 'LineWidth', 1.6, ...
            'Color', mapaCores(junta,:), 'DisplayName', nomesJuntas{junta});
    end
    ylabel(eixoErro, 'Erro (rad)', 'Color', 'w', 'FontSize', 10);
    title(eixoErro, 'Erro de Rastreamento (q_{ref} - q_{sim})', 'Color', 'w', 'FontWeight', 'bold');
    legend(eixoErro, nomesJuntas, 'Location', 'best', 'TextColor', 'w', ...
        'Color', [0.12 0.12 0.18], 'EdgeColor', [0.30 0.30 0.40]);

    % Subplot 2: torque aplicado pelo controlador (PD + gravidade)
    eixoTorque = visualizacao.estilizarEixoEscuro(subplot(3, 1, 2, 'Parent', figControle));
    for junta = 1:numJuntas
        plot(eixoTorque, tempo, tauSim(junta,:), 'LineWidth', 1.6, ...
            'Color', mapaCores(junta,:), 'DisplayName', nomesJuntas{junta});
    end
    ylabel(eixoTorque, 'Torque (Nm)', 'Color', 'w', 'FontSize', 10);
    title(eixoTorque, 'Torque Aplicado pelo Controlador (PD + Gravidade)', 'Color', 'w', 'FontWeight', 'bold');
    legend(eixoTorque, nomesJuntas, 'Location', 'best', 'TextColor', 'w', ...
        'Color', [0.12 0.12 0.18], 'EdgeColor', [0.30 0.30 0.40]);

    % Subplot 3: diferença entre torque ideal (dinâmica inversa) e o do controlador
    eixoDif = visualizacao.estilizarEixoEscuro(subplot(3, 1, 3, 'Parent', figControle));
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
