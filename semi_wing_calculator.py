import math

# DADOS GEOMÉTRICOS DA ASA
envergadura = 10.84 #m
area = 11.6 #m^2
semi_envergadura = envergadura/2
semi_area = area/2

# DADOS ASA
afilamento_a = 0.8
afilamento_b = 0.3

corda_root = area/(((0.868*semi_envergadura)*(1+afilamento_a))+((0.132*semi_envergadura)*afilamento_a*(1+afilamento_b)))
corda_switch = corda_root*afilamento_a
corda_tip = corda_switch*afilamento_b
print(f'Cord_root_wing: {corda_root}')
print(f'Cord_switch_wing: {corda_switch}')
print(f'Cord_tip_wing: {corda_tip}')
# MAIN_WING
diedro = math.radians(4) #°
elevation = math.radians(1) #°
root_points = [0,0,0]
switch_points = [(0.868*semi_envergadura*math.tan(elevation)),
                (0.868*semi_envergadura),(0.868*semi_envergadura*math.tan(diedro))]
tip_points = [switch_points[0]+(corda_switch-corda_tip),semi_envergadura,switch_points[2]+(0.132*semi_envergadura*math.tan(math.radians(28)))]
print(f'Switch points wing: {switch_points}')
print(f'Tip points wing: {tip_points}')
print('---------------------')
#VERTICAL STABILIZER
area_v = 0.15 *area
envergadura_v = 0.15*envergadura

semi_envergadura_v = envergadura_v/2
semi_area_v = area_v/2
afilamento_v = 0.6
corda_root_v = (area_v)/(semi_envergadura_v*(1+afilamento_v))
corda_tip_v = afilamento_v*corda_root_v
root_points_vertical = [(7.17-corda_root_v)-1.972,0,-0.2]
tip_points_vertical = [root_points_vertical[0]+(corda_root_v-corda_tip_v),0,envergadura_v-0.2]
print(f'Area vertical: {area_v}')
print(f'Envergadura vertical: {envergadura_v}')
print(f'Cord_root_vertical: {corda_root_v}')
print(f'Cord_tip_vertical: {corda_tip_v}')
print(f'Root points vertical: {root_points_vertical}')
print(f'Tip points vertical: {tip_points_vertical}')
print('---------------------------')
# HORIZONTAL STABILIZER
area_h = 0.18*area
envergadura_h = 2.65 #m

semi_area_h = area_h/2
semi_envergadura_h = envergadura_h/2
afilamento_a_h = 0.8
afilamento_b_h = 0.3
corda_root_h = area_h/(((0.868*semi_envergadura_h)*(1+afilamento_a_h))+((0.132*semi_envergadura_h)*afilamento_a_h*(1+afilamento_b_h)))
corda_switch_h = afilamento_a_h*corda_root_h
corda_tip_h = corda_switch_h*afilamento_b_h

root_points_h = [tip_points_vertical[0]-(corda_root_h-corda_tip_v),tip_points_vertical[1],tip_points_vertical[2]]
switch_points_h = [root_points_h[0]+(corda_root_h-corda_switch_h),0.868*semi_envergadura_h,root_points_h[2]]
tip_points_h = [switch_points_h[0]+(corda_switch_h-corda_tip_h),semi_envergadura_h,root_points_h[2]]
print(f'Area horizontal: {area_h}')
print(f'Envergadura horizontal: {envergadura_h}')
print(f'Cord_root_horizontal: {corda_root_h}')
print(f'Cord_switch_horizontal: {corda_switch_h}')
print(f'Cord_tip_horizontal: {corda_tip_h}')
print(f'Root points horizontal: {root_points_h}')
print(f'Switch points horizontal: {switch_points_h}')
print(f'Tip points horizontal: {tip_points_h}')
print('---------------------------')

# CALCULO DOS CG das ASAS

area_sec_root_switch = ((0.868*semi_envergadura)*(corda_root+corda_switch))/2
x_sec_root_switch = (2/3)*((corda_root+2*corda_switch)/(corda_root+corda_switch))*corda_root
y_sec_root_switch = 0 + (0.868*semi_envergadura)/3
z_sec_root_switch = y_sec_root_switch*math.tan(diedro)

area_sec_switch_tip = ((0.132*semi_envergadura)*(corda_switch+corda_tip))/2
x_sec_switch_tip = (2/3)*((corda_switch+2*corda_tip)/(corda_switch+corda_tip))*corda_switch
y_sec_switch_tip = switch_points[1] + (0.868*semi_envergadura)/3
z_sec_switch_tip = y_sec_root_switch*math.tan(diedro + math.radians(24))

x_cg_wing = (area_sec_root_switch*x_sec_root_switch+area_sec_switch_tip*x_sec_switch_tip)/(area_sec_root_switch+area_sec_switch_tip)
y_cg_wing = (area_sec_root_switch*y_sec_root_switch+area_sec_switch_tip*y_sec_switch_tip)/(area_sec_root_switch+area_sec_switch_tip)
z_cg_wing = (area_sec_root_switch*z_sec_root_switch+area_sec_switch_tip*z_sec_switch_tip)/(area_sec_root_switch+area_sec_switch_tip)
print(f'Posição X do CG da asa: {x_cg_wing}')
print(f'Posição Y do CG da asa: {y_cg_wing}')
print(f'Posição Z do CG da asa: {z_cg_wing}')