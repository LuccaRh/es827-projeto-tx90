% Faz o calculo da velocidade e aceleração de um waypoint a outro, com base na LSPB
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
