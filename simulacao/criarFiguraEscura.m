% Cria uma figura com o tema escuro padrão do projeto
function fig = criarFiguraEscura(nome)
    arguments
        nome (1,:) char
    end
    fig = figure('Name', nome, 'NumberTitle', 'off', 'Color', [0.07 0.07 0.10]);
end
