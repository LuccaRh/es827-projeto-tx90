% Aplica o tema escuro padrão do projeto a um eixo existente e o devolve
function eixo = estilizarEixoEscuro(eixo)
    set(eixo, 'Color', [0.07 0.07 0.10], 'XColor', [0.65 0.75 0.85], ...
        'YColor', [0.65 0.75 0.85], 'ZColor', [0.65 0.75 0.85], ...
        'GridColor', [0.22 0.27 0.32], 'GridAlpha', 0.6, 'FontSize', 9);
    hold(eixo, 'on'); grid(eixo, 'on');
end
