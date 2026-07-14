% Faz o calculo da velocidade e aceleração de um waypoint a outro, com base na LSPB
% Percorrendo um arco.
function [pontos, cores, vel, acel] = interpolarArco(centro, raio, anguloInicial, anguloFinal, velMax, acelMax, dt, codigoCor)
    sentido = sign(anguloFinal - anguloInicial);
    comprimento = abs(anguloFinal - anguloInicial)*raio;
    [s, vel, acel] = perfilLSPB(comprimento, velMax, acelMax, dt);
    angulo = anguloInicial + sentido*s/raio;
    pontos = [centro(1)*ones(size(angulo));
              centro(2) + raio*cos(angulo);
              centro(3) + raio*sin(angulo)];
    cores = repmat(codigoCor, 1, size(pontos,2));
end
