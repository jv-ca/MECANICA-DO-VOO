function dydt = sistema_aerodinamico(t, y, aeronave, tempo_controle, controle)
    % Extração de variáveis de estado
    u = y(1); v = y(2); w = y(3);  % Velocidades no corpo
    p = y(4); q = y(5); r = y(6);  % Taxas angulares
    phi = y(7); theta = y(8); psi = y(9); % Ângulos de Euler
    x_E = y(10); y_E = y(11); z_E = y(12); % Posições no espaço inercial
    
    % Gravidade 
    g = 9.8;
    % Velocidade total
    V = sqrt(u^2 + v^2 + w^2); % Velocidade total
    
    % Linearização das variáveis de controle (p, q, r)
    p_barra_atual = p * aeronave.b / (2 * V);
    q_barra_atual = q * aeronave.b / (2 * V);
    r_barra_atual = r * aeronave.b / (2 * V);
    
    % Diferenças entre as iterações para as variáveis linearizadas
    delta_p = p_barra_atual - aeronave.p_barra;
    delta_q = q_barra_atual - aeronave.q_barra;
    delta_r = r_barra_atual - aeronave.r_barra;
    
    % Cálculo dos ângulos de ataque (alfa) e derrapagem (beta)
    alfa_atual = atan(w / u); % Ângulo de ataque
    beta_atual = asin(v / V);  % Ângulo de derrapagem
    
    % Cálculo de delta para os ângulos de ataque (alfa) e derrapagem (beta)
    delta_alpha = alfa_atual - aeronave.alfa_ref;
    delta_beta = beta_atual;
    % Controle atua apenas no intervalo [T_start_control, T_end_control]
    if t >= tempo_controle(2) && t <= tempo_controle(3)
        delta_elevator = 0; % Aplicar a deflexão máxima
    elseif t >= tempo_controle(1) && t < tempo_controle(2)
        delta_elevator = controle.delta_elevator - aeronave.elevator_ref; % Aplicar deflexão parcial
    else
        delta_elevator = 0; % Fora do intervalo, a deflexão é zero
    end

    % Deflexões de controle
    delta_aileron = 0;
    delta_rudder = 0;
    
    % Cálculo dos coeficientes aerodinâmicos
    CD = aeronave.CD_ref;
    CY = aeronave.CY_ref + aeronave.CY_beta * delta_beta + aeronave.CY_p * delta_p + aeronave.CY_r * delta_r + aeronave.CY_rudder * delta_rudder;
    CL = aeronave.CL_ref + aeronave.CL_alfa * delta_alpha + aeronave.CL_elevator * delta_elevator + aeronave.CL_q * delta_q;
    Cl = aeronave.Cl_ref + aeronave.Cl_beta * delta_beta + aeronave.Cl_p * delta_p + aeronave.Cl_r * delta_r + aeronave.Cl_aileron * delta_aileron + aeronave.Cl_rudder * delta_rudder;
    Cm = aeronave.Cm_ref + aeronave.Cm_alfa * delta_alpha + aeronave.Cm_q * delta_q + aeronave.Cm_elevator * delta_elevator;
    Cn = aeronave.Cn_ref + aeronave.Cn_beta * delta_beta + aeronave.Cn_p * delta_p + aeronave.Cn_r * delta_r + aeronave.Cn_aileron * delta_aileron + aeronave.Cn_rudder * delta_rudder;
    
    % Pressão dinâmica
    q_bar = 0.5 * aeronave.rho * V^2;
    aeronave.T = aeronave.CD_ref*q_bar*aeronave.S;
    % Forças aerodinâmicas
    C_x = -cos(alfa_atual) * sec(beta_atual) * CD - cos(alfa_atual) * tan(beta_atual) * CY + sin(alfa_atual) * CL;
    C_y = CY;
    C_z = -sin(alfa_atual) * sec(beta_atual) * CD - sin(alfa_atual) * tan(beta_atual) * CY - cos(alfa_atual) * CL;
    
    X_A = C_x * q_bar * aeronave.S;
    Y_A = C_y * q_bar * aeronave.S;
    Z_A = C_z * q_bar * aeronave.S;
    
    % Momentos aerodinâmicos
    L = Cl * q_bar * aeronave.S * aeronave.b;
    M = Cm * q_bar * aeronave.S * aeronave.c;
    N = Cn * q_bar * aeronave.S * aeronave.b;
    
    % Equações de movimento linear
    du_dt = ((aeronave.T + X_A) / aeronave.m) - g*sin(theta) + r * v - q * w;
    dv_dt = ((Y_A) / aeronave.m) + g*sin(phi)*cos(theta) + p * w - r * u;
    dw_dt = ((Z_A) / aeronave.m) + g*cos(phi)*cos(theta) + q * u - p * v;
    
    % Equações de movimento angular
    dp_dt = (L - (aeronave.I_zz - aeronave.I_yy)*q*r)/aeronave.I_xx;
    dq_dt = (M -(aeronave.I_xx - aeronave.I_zz)*p*r)/aeronave.I_yy ;
    dr_dt = (N - (aeronave.I_yy - aeronave.I_xx)*p*q)/aeronave.I_zz;
    
    % Derivadas dos ângulos de Euler
    dphi_dt = p + (q * sin(phi) * tan(theta) + r * cos(phi)) * tan(theta);
    dtheta_dt = q * cos(phi) - r * sin(phi);
    dpsi_dt = (q * sin(phi) + cos(theta)* r) * sec(theta);
    
    % Conversão de velocidades para o sistema inercial
    dx_E_dt = u * cos(theta) * cos(psi) ...
              + v * (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) ...
              + w * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));
    dy_E_dt = u * cos(theta) * sin(psi) ...
              + v * (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) ...
              + w * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi));
    dz_E_dt = u * sin(theta) ...
              - v * sin(phi) * cos(theta) ...
              - w * cos(phi) * cos(theta);
    
    % Retorna o sistema como vetor coluna
    dydt = [du_dt; dv_dt; dw_dt; dp_dt; dq_dt; dr_dt; dphi_dt; dtheta_dt; dpsi_dt; dx_E_dt; dy_E_dt; dz_E_dt];
    
  
end

clear; clc;
% Parâmetros da aeronave
aeronave.m = 697.424;          % Massa (kg)
aeronave.rho = 1.007;       % Densidade do ar (kg/m³)
aeronave.S = 15.428;            % Área da asa (m²)
aeronave.b = 10.84;            % Envergadura (m)
aeronave.c = 1.09;             % corda media (m)
aeronave.I_xx = 1280;       % Momento de inércia em torno do eixo x (kg·m²)
aeronave.I_yy = 1294;       % Momento de inércia em torno do eixo y (kg·m²)
aeronave.I_zz = 2150;       % Momento de inércia em torno do eixo z (kg·m²)
aeronave.I_xy = 0;          % Momento de inércia em torno dos eixos x-y (kg·m²)
aeronave.I_xz = 0;          % Momento de inércia em torno dos eixos x-z (kg·m²)
aeronave.I_yz = 0;          % Momento de inércia em torno dos eixos y-z (kg·m²)

% Coeficientes de referência
aeronave.CL_ref = 0.21236;   % Coeficiente de sustentação (sem deflexão)
aeronave.CD_ref = 0.00189;   % Coeficiente de arrasto (sem deflexão)
aeronave.CY_ref = 0.00000;   % Coeficiente de lado (sem deflexão)
aeronave.Cl_ref = 0.00000;   % Coeficiente de momento roll (sem deflexão)
aeronave.Cm_ref = 0.00000;   % Coeficiente de momento pitch (sem deflexão)
aeronave.Cn_ref = 0.00000;   % Coeficiente de momento yaw (sem deflexão)

% Derivadas em relação ao ângulo de ataque (alfa) e ângulo de derrapagem (beta)  
aeronave.CL_alfa = 4.170401;   aeronave.CL_beta = 0.000000;
aeronave.CY_alfa = -0.000000;  aeronave.CY_beta = -0.288538;
aeronave.Cl_alfa = -0.000000;  aeronave.Cl_beta = -0.122742;
aeronave.Cm_alfa = -1.502410;  aeronave.Cm_beta = -0.000000;
aeronave.Cn_alfa = 0.000000;   aeronave.Cn_beta = 0.084891;

% Derivadas em relação às taxas angulares p, q, r
aeronave.CL_p = -0.000000;    aeronave.CL_q = 8.142290;    aeronave.CL_r = -0.000000;
aeronave.CY_p = -0.150675;    aeronave.CY_q = 0.000000;     aeronave.CY_r = 0.237460;
aeronave.Cl_p = -0.436045;    aeronave.Cl_q = -0.000000;    aeronave.Cl_r = 0.078641;
aeronave.Cm_p = 0.000000;     aeronave.Cm_q = -15.889362;   aeronave.Cm_r = 0.000000;
aeronave.Cn_p = -0.016782;    aeronave.Cn_q = -0.000000;    aeronave.Cn_r = -0.079937;

% Derivadas de estabilidade para aileron, elevator e rudder
% Multiplicar todas 180/pi
conversor_deg_rad = 180/pi;
aeronave.CL_aileron = 0.000000 * conversor_deg_rad;    aeronave.CL_rudder = -0.000000 * conversor_deg_rad;    aeronave.CL_elevator = 0.005507* conversor_deg_rad;
aeronave.CY_aileron = -0.001034 * conversor_deg_rad;    aeronave.CY_rudder = -0.003119* conversor_deg_rad;    aeronave.CY_elevator = 0.000000* conversor_deg_rad;
aeronave.Cl_aileron = -0.007066 * conversor_deg_rad;    aeronave.Cl_rudder = -0.000201* conversor_deg_rad;    aeronave.Cl_elevator = 0.000000* conversor_deg_rad;
aeronave.Cm_aileron = 0.000000 * conversor_deg_rad;     aeronave.Cm_rudder = 0.000000* conversor_deg_rad;     aeronave.Cm_elevator = -0.021509* conversor_deg_rad;
aeronave.Cn_aileron = -0.000284 * conversor_deg_rad;    aeronave.Cn_rudder = 0.001274* conversor_deg_rad;     aeronave.Cn_elevator = -0.000000* conversor_deg_rad;

% Parâmetros de controle (exemplo simples)
t_span = [0 4000];  % Intervalo de tempo para a simulação
tempo_controle = [1500 1505 1510];  % Intervalos de controle
controle.delta_elevator = deg2rad(0.35); % Deflexão do elevator (rad)
controle.delta_aileron = deg2rad(0);  % Deflexão do aileron (rad)
controle.delta_rudder = deg2rad(0);  % Deflexão do leme (rad)

% Inicialização do estado da aeronave
V_0 = 48.34; % m/s
alfa_ref = deg2rad(0.147); %°
betha_ref = 0; %°
h = 2000; % m

u_0 = V_0/(sqrt(1+tan(alfa_ref)^2));
v_0 = 0;
w_0 = u_0*tan(alfa_ref);
p_0 = 0;
q_0 = 0;
r_0 = 0;
phi_0 = 0;
theta_0 = alfa_ref;
psi_0 = 0;
x_E_0 = 0;
y_E_0 = 0;
z_E_0 = -h;

y0 = [u_0 v_0 w_0 p_0 q_0 r_0 phi_0 theta_0 psi_0 x_E_0 y_E_0 z_E_0]; % Vetor dos estados na condição inicial

aeronave.alfa_ref = alfa_ref; %rad
aeronave.elevator_ref = deg2rad(-2.35); %deg -> rad
aeronave.V = sqrt(u_0^2 + v_0^2 + w_0^2);
aeronave.p_barra = p_0 * aeronave.b / (2 * aeronave.V);
aeronave.q_barra = q_0 * aeronave.b / (2 * aeronave.V);
aeronave.r_barra = r_0 * aeronave.b / (2 * aeronave.V);

% Inicialização do controle anterior (valores iniciais)
q_bar = 0.5 * aeronave.rho * aeronave.V^2;
aeronave.T = aeronave.CD_ref*q_bar*aeronave.S;          % Empuxo (N)
% Simulação usando ODE45
[t, y] = ode45(@(t, y) sistema_aerodinamico(t, y, aeronave, tempo_controle, controle), t_span, y0);

% Resultados
figure;

% Gráfico 1: Velocidades no corpo (u, v, w)
subplot(4,3,1);
plot(t, y(:,1), 'r', 'LineWidth', 2); % u
title('Velocidade u');
xlabel('Tempo (s)');
ylabel('u (m/s)');

subplot(4,3,2);
plot(t, y(:,2), 'g', 'LineWidth', 2); % v
title('Velocidade v');
xlabel('Tempo (s)');
ylabel('v (m/s)');

subplot(4,3,3);
plot(t, y(:,3), 'b', 'LineWidth', 2); % w
title('Velocidade w');
xlabel('Tempo (s)');
ylabel('w (m/s)');

% Gráfico 2: Taxas angulares (p, q, r)
subplot(4,3,4);
plot(t, y(:,4), 'r', 'LineWidth', 2); % p
title('Taxa angular p');
xlabel('Tempo (s)');
ylabel('p (rad/s)');

subplot(4,3,5);
plot(t, y(:,5), 'g', 'LineWidth', 2); % q
title('Taxa angular q');
xlabel('Tempo (s)');
ylabel('q (rad/s)');
ylim([-1 1])

subplot(4,3,6);
plot(t, y(:,6), 'b', 'LineWidth', 2); % r
title('Taxa angular r');
xlabel('Tempo (s)');
ylabel('r (rad/s)');

% Gráfico 3: Ângulos de Euler (phi, theta, psi)
subplot(4,3,7);
plot(t, y(:,7), 'r', 'LineWidth', 2); % phi
title('Ângulo de Euler \phi');
xlabel('Tempo (s)');
ylabel('\phi (rad)');

subplot(4,3,8);
plot(t, y(:,8), 'g', 'LineWidth', 2); % theta
title('Ângulo de Euler \theta');
xlabel('Tempo (s)');
ylabel('\theta (rad)');
ylim([-pi pi]); % Define a escala vertical fixa para \theta

subplot(4,3,9);
plot(t, y(:,9), 'b', 'LineWidth', 2); % psi
title('Ângulo de Euler \psi');
xlabel('Tempo (s)');
ylabel('\psi (rad)');

% Gráfico 4: Posições espaciais (x_E, y_E, z_E)
subplot(4,3,10);
plot(t, y(:,10), 'r', 'LineWidth', 2); % x_E
title('Posição x_E');
xlabel('Tempo (s)');
ylabel('x_E (m)');

subplot(4,3,11);
plot(t, y(:,11), 'g', 'LineWidth', 2); % y_E
title('Posição y_E');
xlabel('Tempo (s)');
ylabel('y_E (m)');

subplot(4,3,12);
plot(t, y(:,12), 'b', 'LineWidth', 2); % z_E
title('Posição z_E');
xlabel('Tempo (s)');
ylabel('z_E (m)');