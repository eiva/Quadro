%% ��������� ��������� ������ � �������������
TMax = 20*pi;
dt=0.01;
T=(0:dt:TMax);

%������� ������
deviation = @(t) normrnd(0, 1, size(t,1), size(t,2));
ideal = @(t) 0;

% ������� ���������
tFuncA = @(t, div) [(log(T+exp(1))-1) + div(t);...
                    zeros(size(t,1), size(t,2)) + div(t);...
                    zeros(size(t,1), size(t,2)) + div(t)];
                
% ������� ������� ��������
tFuncRot = @(t, div) [ sin(t*0.1) + div(t);...
                       sin(t*0.6) + div(t);...
                       sin(t*0.3) + div(t)];

%% ���������� � ������� �������
p0 = [0;0;0];
r0 = p0;
v0 = p0;
Ideal = struct('Trj', zeros(3, length(T)),... ���������� 
               'Pos', p0,... ������� Frame
               'Rot', r0,... ���������� Body
               'V', v0,... ������� �������� Body
               'Acc', tFuncA(T,ideal),... ��������� Body
               'RotV', tFuncRot(T,ideal)); % ������� �������� Body
Estimated = struct('Trj', zeros(3, length(T)),...
                   'Pos', p0,...
                   'Rot', r0,...
                   'V', v0,...
                   'Acc', tFuncA(T,deviation),...
                   'RotV', tFuncRot(T,deviation));
Kalman = struct('Trj', zeros(3, length(T)),...
                   'Pos', p0,...
                   'Rot', r0,...
                   'V', v0,...
                   'Acc', tFuncA(T,deviation),...
                   'RotV', tFuncRot(T,deviation));
%% ������ ��������� �������

for i=1:length(T)
    t = T(i);
    Ideal.dRot = Ideal.RotV(:,i);
    Ideal.dAcc = Ideal.Acc(:,i);
    Ideal.Rot = Ideal.Rot + Ideal.dRot * dt;
    Ideal.V = Ideal.V + dt*Ideal.dAcc;
    Tr=rotz(Ideal.Rot(3)) * rotx(Ideal.Rot(1)) * roty(Ideal.Rot(2));
    Ideal.Vf = Tr*Ideal.V;
    Ideal.Pos = Ideal.Pos + dt* Ideal.Vf;
    p = Ideal.Pos;
    Ideal.Trj(:,i) = Ideal.Pos;
end

%% ������� �������������� �������

for i=1:length(T)
    t = T(i);
    Estimated.dRot = Estimated.RotV(:,i);
    Estimated.dAcc = Estimated.Acc(:,i);
    Estimated.Rot = Estimated.Rot + Estimated.dRot * dt;
    Estimated.V = Estimated.V + dt*Estimated.dAcc;
    Tr=rotz(Estimated.Rot(3)) * rotx(Estimated.Rot(1)) * roty(Estimated.Rot(2));
    Estimated.Vf = Tr*Estimated.V;
    Estimated.Pos = Estimated.Pos + dt* Estimated.Vf;
    p = Estimated.Pos;
    Estimated.Trj(:,i) = Estimated.Pos;
end

%% ������� �������������� ������� � ����������� ������� �������


%% �����������
plotTrj = @(trj, p) plot3(trj(1,:)', trj(2,:)', trj(3,:)', p);
figure
hold on
plotTrj(Ideal.Trj, 'r-');
plotTrj(Estimated.Trj, 'b-');