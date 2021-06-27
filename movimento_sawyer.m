% Este projecto mostra como traçar uma forma 3D predefinida no espaço. 
%Seguir um caminho suave e distinto é útil em muitas aplicações de robótica. 
%O robo segue uma trajetória 3-D
%MATLAB® e é executada usando o robô Sawyer da Rethink Robotics®. 
%O objetivo é gerar um caminho suave para o efeito final do robô seguir com base em pontos de referência 

%nota meti algumas pausas de 10 segundospara dar tempo de visualização de
%janelas e assim ser de fácil compreensão 




sawyer = importrobot ( 'sawyer.urdf' );
sawyer.DataFormat = 'column';
taskSpaceLimits = [0.25 0.5; -0.125 0.125; -0.15 0.1];
numJoints = 8; 

numSamples = 7;
[pathSegments, surface] = generateMembranePaths(numSamples, taskSpaceLimits);

% output Figura 1 grafico de ponto de refência, o robo vai se mover com
% estes pontos
figure 
surf(surface{:},'FaceAlpha',0.3,'EdgeColor','none');
hold all
for i=1:numel(pathSegments)
    segment = pathSegments{i};
    plot3(segment(:,1),segment(:,2),segment(:,3),'x-','LineWidth', 2);
end
hold off


pause(10)
% figura 2 o sawyer
figure 
show(sawyer);


hold all

for i=1:numel(pathSegments)
    segment = pathSegments{i};
    plot3(segment(:,1),segment(:,2),segment(:,3),'x-','LineWidth',2);
end

view(135,20)
axis([-1 1 -.5 .5 -1 .75])
hold off


ik = inverseKinematics('RigidBodyTree', sawyer);
initialGuess = sawyer.homeConfiguration;
weights = [1 1 1 1 1 1];
eeName = 'right_hand';



%inicializar a matriz
jointPathSegmentMatrix = zeros(length(pathSegments),numJoints,numSamples);

%definir as cordenadas
sawyerOrientation = axang2rotm([0 1 0 pi]);

for i = 1:length(pathSegments)
    currentTaskSpaceSegment = pathSegments{i};
    currentJointSegment = zeros(numJoints, length(currentTaskSpaceSegment));
    for j = 1:length(currentTaskSpaceSegment)
        pose = [sawyerOrientation currentTaskSpaceSegment(j,:)'; 0 0 0 1];
        currentJointSegment(:,j) = ik(eeName,pose,weights,initialGuess);
        initialGuess = currentJointSegment(:,j);
    end
    
    jointPathSegmentMatrix(i, :, :) = (currentJointSegment);
end



sim("shapeTracingSawyer.slx")
pause(10)

% posição final
xPositionsEE = reshape(eePosData.Data(1,:,:),1,size(eePosData.Data,3));
yPositionsEE = reshape(eePosData.Data(2,:,:),1,size(eePosData.Data,3));
zPositionsEE = reshape(eePosData.Data(3,:,:),1,size(eePosData.Data,3));

%Extrair os espaços de junta 
jointConfigurationData = reshape(jointPosData.Data, numJoints, size(eePosData.Data,3));

figure 
surf(surface{:},'FaceAlpha',0.3,'EdgeColor','none');
hold all
plot3(xPositionsEE,yPositionsEE,zPositionsEE)
grid on
hold off

% vamos visualizar a movimentação do robo
% para ser mais rapido ver vamos acelarar os passos
vizStep = 5;
pause(10)
% figura final, o robo mexendo
figure 
set(gcf,'Visible','on');

% Iterate through all joint configurations and end-effectort positions
for i = 1:vizStep:length(xPositionsEE)
    show(sawyer, jointConfigurationData(:,i),'Frames','off','PreservePlot',false);
    hold on
    plot3(xPositionsEE(1:i),yPositionsEE(1:i),zPositionsEE(1:i),'b','LineWidth',3)
    
    view(135,20)
    axis([-1 1 -.5 .5 -1 .75])
    
    drawnow
end
hold off


