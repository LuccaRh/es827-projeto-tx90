% Carrega o modelo URDF do TX90 e acopla o aplicador de tinta ao flange.
% A ferramenta entra na cadeia como corpo rígido com massa, centro de massa
% e inércia, de modo que cinemática e dinâmica passam a considerá-la.
%
% ferramenta: struct com os campos comprimentoTcp [m], massa [kg],
%             centroMassa [1x3, m] e inercia [1x6, kg*m^2].
function [robo, numJuntas] = carregarRobo(arquivoUrdf, ferramenta)
    arguments
        arquivoUrdf (1,:) char
        ferramenta (1,1) struct
    end

    robo = importrobot(arquivoUrdf);
    robo.DataFormat = 'row';
    robo.Gravity = [0 0 -9.81];   % gravidade padrão [m/s^2]
    numJuntas = numel(homeConfiguration(robo));

    aplicador = rigidBody('paint_tcp');
    juntaAplicador = rigidBodyJoint('paint_tcp_fixed', 'fixed');
    setFixedTransform(juntaAplicador, trvec2tform([0 0 ferramenta.comprimentoTcp]));
    aplicador.Joint = juntaAplicador;
    aplicador.Mass = ferramenta.massa;
    aplicador.CenterOfMass = ferramenta.centroMassa;
    aplicador.Inertia = ferramenta.inercia;
    addBody(robo, aplicador, 'tool0');
end
