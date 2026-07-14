% Perfil LSPB (segmento linear com concordância parabólica) ajustado à grade
% de amostragem: procura o menor número de passos que respeita velMax/acelMax.
% Retorna a coordenada de caminho s (0..distancia) e os perfis de velocidade v
% e aceleração a amostrados em passos de dt.
function [s, v, a] = perfilLSPB(distancia, velMax, acelMax, dt)
    arguments
        distancia (1,1) double {mustBePositive}
        velMax (1,1) double {mustBePositive}
        acelMax (1,1) double {mustBePositive}
        dt (1,1) double {mustBePositive}
    end

    if distancia <= velMax^2/acelMax
        tempoMinimo = 2*sqrt(distancia/acelMax);
    else
        tempoMinimo = distancia/velMax + velMax/acelMax;
    end

    numIntervalos = max(2, ceil(tempoMinimo/dt));
    achouPerfil = false;
    while ~achouPerfil
        tempoTotal = numIntervalos*dt;
        for numPassosAcel = 1:floor(numIntervalos/2)
            tempoAcel = numPassosAcel*dt;
            vel = distancia/(tempoTotal - tempoAcel);
            acel = vel/tempoAcel;
            if vel <= velMax*(1+1e-10) && acel <= acelMax*(1+1e-10)
                achouPerfil = true;
                break;
            end
        end
        if ~achouPerfil
            numIntervalos = numIntervalos + 1;
        end
    end

    instantes = (0:numIntervalos)*dt;
    s = zeros(size(instantes));
    v = zeros(size(instantes));
    a = zeros(size(instantes));
    for k = 1:numel(instantes)
        if instantes(k) <= tempoAcel + eps
            s(k) = 0.5*acel*instantes(k)^2;
            v(k) = acel*instantes(k);
            a(k) = acel;
        elseif instantes(k) < tempoTotal - tempoAcel - eps
            s(k) = 0.5*acel*tempoAcel^2 + vel*(instantes(k) - tempoAcel);
            v(k) = vel;
        else
            restante = tempoTotal - instantes(k);
            s(k) = distancia - 0.5*acel*restante^2;
            v(k) = acel*restante;
            a(k) = -acel;
        end
    end
    s(1) = 0;
    s(end) = distancia;
    v([1 end]) = 0;
end
