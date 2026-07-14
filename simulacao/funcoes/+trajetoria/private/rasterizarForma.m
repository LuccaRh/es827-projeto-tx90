% Gera os waypoints de PREENCHIMENTO de uma forma plana (plano X = centro(1))
% em serpentina (vai-e-volta).
% A quantidade de waypoints depende da resolução que foi passada, ou seja, do leque
% A saida será uma matriz 3xN, onde cada coluna é um waypoint [x; y; z].
% Entradas: 
%   centro   : [3x1] centro da forma no espaço cartesiano
%   limiteY  : handle @(z) -> meia-largura da forma na altura z
%   zMin,zMax: faixa de alturas a preencher (offsets do centro) [m]
%   passo    : espaçamento entre passadas (passo do leque) [m]
function pontos = rasterizarForma(centro, limiteY, zMin, zMax, passo)
    niveis = zMin:passo:zMax;
    if niveis(end) < zMax - 1e-9
        niveis(end+1) = zMax;   % garante que a última passada toca a borda
    end

    colunas = {};
    sentido = 1;                % alterna o sentido da passada a cada nível
    for i = 1:numel(niveis)
        z = niveis(i);
        w = limiteY(z);
        if w <= 1e-6            % altura sem largura útil (ponta da forma)
            ys = 0;
        elseif sentido > 0
            ys = [-w, w];
        else
            ys = [w, -w];
        end
        for y = ys
            candidato = centro + [0; y; z];
            % evita waypoints repetidos (LSPB exige distância > 0 entre eles)
            if isempty(colunas) || norm(candidato - colunas{end}) > 1e-9
                colunas{end+1} = candidato; %#ok<AGROW>
            end
        end
        sentido = -sentido;
    end
    pontos = [colunas{:}];
end
