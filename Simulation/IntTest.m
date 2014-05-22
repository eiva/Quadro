%% Генерация начальных данных с погрешностями
TMax = 20*pi;
dt=0.01;
T=(0:dt:TMax);

%Функция ошибки
deviation = @(t) normrnd(0, 1, size(t,1), size(t,2));
ideal = @(t) 0;

% % Функция Ускорения
% tFuncA = @(t, div) [(log(T+exp(1))-1) + div(t);...
%                     zeros(size(t,1), size(t,2)) + div(t);...
%                     zeros(size(t,1), size(t,2)) + div(t)];
%                 
% % Функция угловой скорости
% tFuncRot = @(t, div) [ sin(t*0.1) + div(t);...
%                        sin(t*0.6) + div(t);...
%                        sin(t*0.3) + div(t)];

% Функция Ускорения
tFuncA = @(t, div) [zeros(size(t,1), size(t,2)) + div(t);...
                    zeros(size(t,1), size(t,2)) + div(t);...
                    zeros(size(t,1), size(t,2)) + div(t)];
                
% Функция угловой скорости
tFuncRot = @(t, div) [ zeros(size(t,1), size(t,2)) + div(t);...
                       zeros(size(t,1), size(t,2)) + div(t);...
                       zeros(size(t,1), size(t,2)) + div(t)];

                   
%% Подготовка к расчету позиции
p0 = [0;0;0];
r0 = p0;
v0 = p0;
Ideal = struct('Trj', zeros(3, length(T)),... Траектория 
               'Pos', p0,... Позиция Frame
               'Rot', r0,... Ориентация Body
               'V', v0,... Текущая скорость Body
               'Acc', tFuncA(T,ideal),... Ускорения Body
               'RotV', tFuncRot(T,ideal)); % Угловые скорости Body
           
Estimated = struct('Trj', zeros(3, length(T)),...
                   'Pos', p0,...
                   'Rot', r0,...
                   'V', v0,...
                   'Acc', tFuncA(T,deviation),...
                   'RotV', tFuncRot(T,deviation));
               
ABFilter = struct('Trj', zeros(3, length(T)),...
                'Pos', p0,...
                'Rot', r0,...
                'V', v0,...
                'Acc', tFuncA(T,deviation),...
                'RotV', tFuncRot(T,deviation));
%% Расчет Идеальной позиции

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

%% Рассчет Предполагаемой позиции

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

%% Рассчет Предполагаемой позиции с упрощенным фильтром Калмана
ABFilter.Aopt = [0;0;0];
ABFilter.Ropt = [0;0;0];
ABFilter.Kstab = 0.8; % параметр стабилизации.
for i=1:length(T)
    t = T(i);
    ABFilter.dRot = ABFilter.RotV(:,i);
    ABFilter.dAcc = ABFilter.Acc(:,i);
    ABFilter.Ropt = ABFilter.dRot*ABFilter.Kstab + ABFilter.Ropt*(1-ABFilter.Kstab);
    ABFilter.Aopt = ABFilter.dAcc*ABFilter.Kstab + ABFilter.Aopt*(1-ABFilter.Kstab);
    ABFilter.Rot = ABFilter.Rot + ABFilter.Ropt * dt;
    ABFilter.V = ABFilter.V + dt*ABFilter.Aopt;
    Tr=rotz(ABFilter.Rot(3)) * rotx(ABFilter.Rot(1)) * roty(ABFilter.Rot(2));
    ABFilter.Vf = Tr*ABFilter.V;
    ABFilter.Pos = ABFilter.Pos + dt* ABFilter.Vf;
    p = ABFilter.Pos;
    ABFilter.Trj(:,i) = ABFilter.Pos;
end

%% Отображение
plotTrj = @(trj, p) plot3(trj(1,:)', trj(2,:)', trj(3,:)', p);
figure
hold on
plotTrj(Ideal.Trj, 'r*');
plotTrj(Estimated.Trj, 'g-');
plotTrj(ABFilter.Trj, 'b-');
display(norm(Ideal.Trj-Estimated.Trj))