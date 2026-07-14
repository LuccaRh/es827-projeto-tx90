% Interpola um movimento livre (linha reta) entre dois pontos com perfil LSPB.
% Usada nas idas e vindas à estação de troca de cor e nos conectores radiais
% entre arcos da faixa branca.
function [pontos, cores, vel, acel] = interpolarTransicao(pInicio, pFim, velMax, acelMax, dt, codigoCor)
    delta = pFim - pInicio;
    distancia = norm(delta);
    [s, vel, acel] = perfilLSPB(distancia, velMax, acelMax, dt);
    pontos = pInicio + (delta/distancia)*s;
    cores = repmat(codigoCor, 1, size(pontos,2));
end
