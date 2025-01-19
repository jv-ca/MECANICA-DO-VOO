comprimento_fuselagem = 7.17 #m
b = 10.84 #m
peso = 697.424 #kg
Rx = 0.25
Ry = 0.38
Rz = 0.39

Ixx = ((b**2)*peso*(Rx**2))/4
Iyy = ((comprimento_fuselagem**2)*peso*(Ry**2))/4
Izz = (((b+comprimento_fuselagem)/2)**2) * (peso*(Rz**2)/4)

print(f'Ixx: {Ixx}')
print(f'Iyy: {Iyy}')
print(f'Izz: {Izz}')