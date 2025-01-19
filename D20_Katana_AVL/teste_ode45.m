clear; clc;

% Definir a variável de controle: Deflexão da superfície de comando
deflexao = [2 5]; % Deflexão máxima (em graus)
delta_max = 2;
tempo_controle = [30 50 70];
% Define o sistema como uma função anônima
odefun = @(t, y) sistema(t, y, delta_max, tempo_controle);

% Função separada para o sistema
function dydt = sistema(t, y, delta_max, tempo_controle)
    % Associação explícita
    u = y(1); % w(t) = y(1)
    v = y(2); % u(t) = y(2)
    w = y(3);
    
    % Controle atua apenas no intervalo [T_start_control, T_end_control]
    if t >= tempo_controle(2) && t <= tempo_controle(3)
        delta = delta_max*0.5; % Aplicar a deflexão máxima
    elseif t>= tempo_controle(1) && t < tempo_controle(2)
        delta = delta_max;
    else
        delta = 0; % Fora do intervalo, a deflexão é zero
    end
    
    % Define as equações diferenciais com a deflexão
    du_dt = -0.2*w + 2*u + 2*v + delta;   % dw/dt = -w + 2u + 2p + delta
    dv_dt = -2*w - 3*u - 0.5*v + 2*delta; % du/dt = -2w - 3u - 0.5p + delta
    dw_dt = -3*w - 4*u - 7*v + 4*delta;   % dp/dt = -3w - 4u - 7p + delta
    
    % Retorna o sistema como um vetor coluna
    dydt = [du_dt; dv_dt; dw_dt];
end

% Intervalo de integração
tspan = [0 100];

% Condições iniciais
y0 = [1; -1; 2];  % w(0) = 1, u(0) = -1, p(0) = 2

% Resolve o sistema
[t, y] = ode45(odefun, tspan, y0);

% Extração das soluções
u = y(:, 1); % w(t)
v = y(:, 2); % u(t)
w = y(:, 3); % p(t)

% Plota as soluções
figure;
plot(t, u, '-r', 'LineWidth', 1.5); hold on;
plot(t, v, '-g', 'LineWidth', 1.5); hold on;
plot(t, w, '-b', 'LineWidth', 1.5);
xlabel('Tempo (t)');
ylabel('Soluções (w ,u e p)');
legend('w(t)', 'u(t)', 'p(t)');
title('Solução do sistema de ODEs com controle em intervalo específico');
grid on;