% Faz o calculo da velocidade e aceleração de um waypoint a outro, com base na LSPB para uma faixa
function [pontos, cores, vel, acel, waypoints] = preencherFaixa(geometria, perfis, angIni, angFim, codigoCor)
    dt = perfis.dt;
    raioInt = geometria.raioArco - geometria.larguraFaixa/2;
    raioExt = geometria.raioArco + geometria.larguraFaixa/2;
    raios = raioInt:perfis.passoLeque:raioExt;
    if raios(end) < raioExt - 1e-9
        raios(end+1) = raioExt;
    end

    pontos = []; cores = []; vel = []; acel = []; waypoints = [];
    for i = 1:numel(raios)
        if mod(i,2) == 1        % alterna o sentido angular (serpentina)
            a0 = angIni; a1 = angFim;
        else
            a0 = angFim; a1 = angIni;
        end
        [pArc, cArc, vArc, aArc] = interpolarArco(geometria.centroArco, raios(i), a0, a1, ...
            perfis.velPintura, perfis.acelPintura, dt, codigoCor);

        if isempty(pontos)
            pontos = pArc; cores = cArc; vel = vArc; acel = aArc;
        else
            % conector radial curto entre o fim do arco anterior e o início deste
            [pCon, cCon, vCon, aCon] = interpolarTransicao(pontos(:,end), pArc(:,1), ...
                perfis.velPintura, perfis.acelPintura, dt, codigoCor);
            pontos = [pontos, pCon(:,2:end), pArc(:,2:end)];
            cores  = [cores,  cCon(2:end),   cArc(2:end)];
            vel    = [vel,    vCon(2:end),   vArc(2:end)];
            acel   = [acel,   aCon(2:end),   aArc(2:end)];
        end
        waypoints = [waypoints, pArc(:,1), pArc(:,end)]; %#ok<AGROW>
    end
end
