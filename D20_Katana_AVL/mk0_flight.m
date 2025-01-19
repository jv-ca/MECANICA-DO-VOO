clear; clc;

%% CONSTANTES =============================================================================================
g = 9.8; % m^2/s
aeronave.S = 11.6; % m^2
aeronave.b = 10.84; % m
MAC = 1.09; % m
V_0 = 48.34; % m/s
h = 4000; % m

% DADOS de ATMOSFERA usando o modelo ISA
c1 = -6.5*10^-3; % K/m
densidade_0 = 1.225; % kg/m
T_0 = 288.16; % K
p_0 = 101325; % Pa
R = 287; % J/kg
constante_potencia = g/(c1*R);

T = T_0 + c1*(h); % K
aeronave.densidade = densidade_0*(T/T_0)^(-(constante_potencia + 1));
pressao = p_0*(T/T_0)^(-constante_potencia);

% Controle
tempo_simu = [0 100];
tempo_manobras = [10 20 30];
deflexao = [10 20];
%% CONDIÇÃO DA AERONAVE ========================================================================================
V_som = sqrt(1.4*R*T);
Ma = V_0/V_som;
pressao_dinamica_0 = (1/2)*(V_0^2)*aeronave.densidade;
alfa_ref = 0.19; %°
betha_ref = 0; %°

aeronave.massa = 697.424; %kg
aeronave.I_xx = 1280;
aeronave.I_yy = 1294;
aeronave.I_zz = 2150;
aeronave.M_T = 0;
%% ESTADOS INICIAIS
u_0 = V_0/sqrt(1+tan(alfa_ref)^2);
v_0 = 0;
w_0 = u_0*tan(alfa_ref);
p_0 = 0;
q_0 = 0;
r_0 = 0;
phi_0 = 0;
theta_0 = 0;
psi_0 = 0;
x_E_0 = 0;
y_E_0 = 0;
z_E_0 = -h;

y_0 = [u_0 v_0 w_0 p_0 q_0 r_0 phi_0 theta_0 psi_0 x_E_0 y_E_0 z_E_0]; % Vetor dos estados na condição inicial
%% DERIVADAS DE ESTABILIDADE ====================================================================================
% Condições de Referência
CL_ref = 0.28302; CD_ref = 0.00246; CY_ref = 0.00000;
Cl_ref = 0.00000; Cm_ref = 0.00005; Cn_ref = 0.00000;

% Derivadas em relação a alpha e beta
CL_alfa = 5.546633;   CL_beta = 0.000000;
CY_alfa = -0.000000;  CY_beta = -0.383755;
Cl_alfa = -0.000000;  Cl_beta = -0.163247;
Cm_alfa = -1.998206;  Cm_beta = -0.000000;
Cn_alfa = 0.000000;   Cn_beta = 0.112905;

% Derivadas em relação às taxas angulares p, q, r
CL_p = -0.000000;    CL_q = 10.829246;    CL_r = -0.000000;
CY_p = -0.200398;    CY_q = 0.000000;     CY_r = 0.315822;
Cl_p = -0.579940;    Cl_q = -0.000000;    Cl_r = 0.104592;
Cm_p = 0.000000;     Cm_q = -21.132851;   Cm_r = 0.000000;
Cn_p = -0.022319;    Cn_q = -0.000000;    Cn_r = -0.106316;

% Derivadas de estabilidade para aileron, elevator e rudder
CL_aileron = 0.000000;    CL_rudder = -0.000000;    CL_elevator = 0.007324;
CY_aileron = -0.001375;    CY_rudder = -0.004148;    CY_elevator = 0.000000;
Cl_aileron = -0.009397;    Cl_rudder = -0.000268;    Cl_elevator = 0.000000;
Cm_aileron = 0.000000;     Cm_rudder = 0.000000;     Cm_elevator = -0.028607;
Cn_aileron = -0.000378;    Cn_rudder = 0.001694;     Cn_elevator = -0.000000;

%% DISPLAY RESULTADOS =======================================================================================
checagem = [T, aeronave.densidade, pressao, V_0, V_som, Ma];
fprintf('%.3f %.3f\n', checagem.');
