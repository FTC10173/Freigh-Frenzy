import math as m

fieldSize = 705
robotSize = (64,90)
IMUPos = (0,0,0)
DSPos = ((0,45,0),(32,0,90),(0,-45,180),(-32,0,270))
RPos = [100, 100, 45*0.0174533]
DS = 0



DSAbs = ((DSPos[DS][0]*m.cos(m.pi*2-RPos[2]))-(DSPos[DS][1]*m.sin(m.pi*2-RPos[2]))+RPos[0],(DSPos[DS][1]*m.cos(m.pi*2-RPos[2]))+(DSPos[DS][0]*m.sin(m.pi*2-RPos[2]))+RPos[1],(RPos[2]*57.2958)+DSPos[DS][2])
q = m.floor(DSAbs[2]/90)%4
d1 = abs((m.floor(((q+2)%4)/2)*fieldSize)-DSAbs[((q+1)%4)%2])
d2 = abs((m.floor(((q+3)%4)/2)*fieldSize)-DSAbs[q%2])
distance = 0
if DSAbs[2]%90 < (90-m.atan(d1 / d2)*57.2958):
    distance = d1 / m.cos((DSAbs[2]%90)*0.0174533)
else:
    distance = d2 / m.cos((90-((DSAbs[2]%90)))*0.0174533)
print(distance)  