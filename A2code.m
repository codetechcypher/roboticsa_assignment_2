%A2
clear all;
close all;
clc;

%Kinova spherical 7dof DH Parameters
L1 = Link('d', 0.2755 ,'a', 0       ,'alpha', pi/2  ,'qlim', deg2rad([-350 350]));
L2 = Link('d', 0       ,'a', 0       ,'alpha', pi/2  ,'qlim', deg2rad([47 313]));
L3 = Link('d', 0.41   ,'a', 0       ,'alpha', pi/2  ,'qlim', deg2rad([-350 350]));
L4 = Link('d', 0.0098 ,'a', 0       ,'alpha', pi/2  ,'qlim', deg2rad([30 330]));
L5 = Link('d', 0.3111  ,'a', 0       ,'alpha', pi/2  ,'qlim', deg2rad([-350 350]));
L6 = Link('d', 0       ,'a', 0       ,'alpha', pi/2  ,'qlim', deg2rad([65 295]));
L7 = Link('d', 0.2638 ,'a', 0       ,'alpha', pi    ,'qlim', deg2rad([-350 350]));

Kinova= SerialLink([L1, L2, L3, L4, L5, L6, L7],'name','Kinova robot spherical 7dof');
Kinova.base = transl( 0, 0, 1);

% q0 is initial pose of Kinova 7dof
q0 = deg2rad([ 180 180 180 180 180 180 180]);
Kinova.plot(q0, 'workspace',[ -2, 2, -2, 2, -0.01, 4], 'scale', 0.3);
hold on
camlight;

% % Load Table % %
[f,v,data] = plyread('workbench.ply','tri');
tableVertexCount = size(v,1);
% Move center point to origin
midPoint = sum(v)/tableVertexCount;
tableVerts = v - repmat(midPoint,tableVertexCount,1);
% Create a transform to describe the location (at the origin, since it's centered
tablePose = eye(4);
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
TableMesh_h = trisurf(f,v(:,1),v(:,2),v(:,3),'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','Flat');
% Move the pose forward and a slight and random rotation
tablePose = transl(1,0.25,0);
updatedPoints = [tablePose * [v,ones(tableVertexCount,1)]']';  
% Now update the Vertices
TableMesh_h.Vertices = updatedPoints(:,1:3);
drawnow();

% % Load Pencil % %
[pf,pv,pdata] = plyread('pencil.ply','tri');
pencilVertexCount = size(pv,1);
% Move center point to origin
midPoint = sum(pv)/pencilVertexCount;
pencilVerts = pv - repmat(midPoint,pencilVertexCount,1);
% Create a transform to describe the location (at the origin, since it's centered
pencilPose = eye(4);
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [pdata.vertex.red, pdata.vertex.green, pdata.vertex.blue] / 255;
pencilMesh_h = trisurf(pf,pv(:,1),pv(:,2),pv(:,3),'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','Flat');
% Move the pose forward and at slight and random rotation
pencilPose = transl(0.25,0,0.9);
updatedPoints = [pencilPose * [pv,ones(pencilVertexCount,1)]']';  
% Now update the Vertices
pencilMesh_h.Vertices = updatedPoints(:,1:3);
drawnow();

% % Load Hammer % %
[f,v,data] = plyread('hammer.ply','tri');
hammerVertexCount = size(v,1);
% Move center point to origin
midPoint = sum(v)/hammerVertexCount;
hammerVerts = v - repmat(midPoint,hammerVertexCount,1);
% Create a transform to describe the location (at the origin, since it's centered
hammerPose = eye(4);
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
hammerMesh_h = trisurf(f,v(:,1),v(:,2),v(:,3),'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','Flat');
% Move the pose forward and at slight and random rotation
hammerPose = transl(0.5,0,0.9);
updatedPoints = [hammerPose * [v,ones(hammerVertexCount,1)]']';  
% Now update the Vertices
hammerMesh_h.Vertices = updatedPoints(:,1:3);
drawnow();

% % % Move robot to hammer
% p1 = hammerPose;
% p2 = transl(0.5,0.5,1.25);
% p3 = transl(0.5,1.25,1.25)*trotx(deg2rad(90));
% 
% q1 = Kinova.ikcon(p1);
% q2 = Kinova.ikcon(p2);
% q3 = Kinova.ikcon(p3);
% 
% steps = 30;
% s = lspb(0,1,steps); 
% qMatrix = nan(steps,6); 
% 
% for i = 1:steps; 
%     Qmatrix(i,:)= (1-s(i))*q0 + s(i)*q1;
%     Kinova.plot(Qmatrix(i,:))
% end
% 
% for i = 1:steps; 
%     Qmatrix(i,:)= (1-s(i))*q1 + s(i)*q2;
%     Kinova.plot(Qmatrix(i,:))
%     
%     qmove = Kinova.getpos();
%     hammerPose = Kinova.fkine(qmove);
%     updatedPoints = [hammerPose * [v,ones(hammerVertexCount,1)]']';  
%     % Now update the Vertices
%     hammerMesh_h.Vertices = updatedPoints(:,1:3);
%     drawnow();
% end
% 
% for i = 1:steps;
%      Qmatrix(i,:)= (1-s(i))*q2 + s(i)*q3;
%      Kinova.plot(Qmatrix(i,:))
%      
%      qmove = Kinova.getpos();
%      hammerPose = Kinova.fkine(qmove);
%      updatedPoints = [hammerPose * [v,ones(hammerVertexCount,1)]']';  
%      % Now update the Vertices
%      hammerMesh_h.Vertices = updatedPoints(:,1:3);
%      drawnow();
%      
% end

% % Move robot to pencil
p1 = pencilPose;
p2 = transl(0.25,0,1.3);
p3 = transl(0.25,2,1.3)*trotx(deg2rad(15));

q0 = Kinova.getpos();
q1 = Kinova.ikcon(p1);
q2 = Kinova.ikcon(p2);
q3 = Kinova.ikcon(p3);

steps = 30;
s = lspb(0,1,steps); 
qMatrix = nan(steps,6); 

for i = 1:steps; 
    Qmatrix(i,:)= (1-s(i))*q0 + s(i)*q1;
    Kinova.plot(Qmatrix(i,:))
end

for i = 1:steps; 
    Qmatrix(i,:)= (1-s(i))*q1 + s(i)*q2;
    Kinova.plot(Qmatrix(i,:))
    
    qmove = Kinova.getpos();
    pencilPose = Kinova.fkine(qmove);
    updatedPoints = [pencilPose * [pv,ones(pencilVertexCount,1)]']';
    % Now update the Vertices
    pencilMesh_h.Vertices = updatedPoints(:,1:3);
    drawnow();
end

for i = 1:steps
     Qmatrix(i,:)= (1-s(i))*q2 + s(i)*q3;
     Kinova.plot(Qmatrix(i,:))
     
     qmove = Kinova.getpos();
     pencilPose = Kinova.fkine(qmove);
     updatedPoints = [pencilPose * [pv,ones(pencilVertexCount,1)]']';  
     % Now update the Vertices
     pencilMesh_h.Vertices = updatedPoints(:,1:3);
     drawnow();
     
end
 
% % Move robot to hammer
p1 = hammerPose;
p2 = transl(0.5,0.5,1.25);
p3 = transl(0.5,1.25,1.25)*trotx(deg2rad(90));

q0 = Kinova.getpos();
q1 = Kinova.ikcon(p1);
q2 = Kinova.ikcon(p2);
q3 = Kinova.ikcon(p3);

steps = 30;
s = lspb(0,1,steps); 
qMatrix = nan(steps,6); 

for i = 1:steps; 
    Qmatrix(i,:)= (1-s(i))*q0 + s(i)*q1;
    Kinova.plot(Qmatrix(i,:))
end

for i = 1:steps; 
    Qmatrix(i,:)= (1-s(i))*q1 + s(i)*q2;
    Kinova.plot(Qmatrix(i,:))
    
    qmove = Kinova.getpos();
    hammerPose = Kinova.fkine(qmove);
    updatedPoints = [hammerPose * [v,ones(hammerVertexCount,1)]']';  
    % Now update the Vertices
    hammerMesh_h.Vertices = updatedPoints(:,1:3);
    drawnow();
end

for i = 1:steps;
     Qmatrix(i,:)= (1-s(i))*q2 + s(i)*q3;
     Kinova.plot(Qmatrix(i,:))
     
     qmove = Kinova.getpos();
     hammerPose = Kinova.fkine(qmove);
     updatedPoints = [hammerPose * [v,ones(hammerVertexCount,1)]']';  
     % Now update the Vertices
     hammerMesh_h.Vertices = updatedPoints(:,1:3);
     drawnow();
     
end



    
    