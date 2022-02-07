import pygame as pg
import pygame.gfxdraw
import math as m

pg.init()

fieldSize = 705
RSize = (64,90)
IMUPos = (0,0,0)
DSPos = ((0,45,0),(32,0,90),(0,-45,180),(-32,0,270))
RPos = [100,100,0]

screen = pg.display.set_mode((fieldSize, fieldSize))

class Sensor:
    def __init__(self, DSPos, rand, IMUPos):
        self.DSPos = DSPos
        self.IMUPos = IMUPos
        self.rand = rand
    def getDistance(self, RPos, DS):
        DSAbs = ((DSPos[DS][0]*m.cos(m.pi*2-RPos[2]))-(DSPos[DS][1]*m.sin(m.pi*2-RPos[2]))+RPos[0],(DSPos[DS][1]*m.cos(m.pi*2-RPos[2]))+(DSPos[DS][0]*m.sin(m.pi*2-RPos[2]))+RPos[1],(RPos[2]*57.2958)+DSPos[DS][2])
        q = m.floor(DSAbs[2]/90)%4
        d1 = abs((m.floor(((q+2)%4)/2)*fieldSize)-DSAbs[((q+1)%4)%2])
        d2 = abs((m.floor(((q+3)%4)/2)*fieldSize)-DSAbs[q%2])
        if DSAbs[2]%90 < (90-m.atan(d1 / d2)*57.2958):
            return d1 / m.cos((DSAbs[2]%90)*0.0174533)
        else:
            return d2 / m.cos((90-((DSAbs[2]%90)))*0.0174533)
    def getRotation(self, RPos):
        return RPos[2] 

class Robot:
    def __init__(self, RSize, RPos):
        self.RSize = RSize
        self.RPos = RPos
        self.touching = False
    def drawRobot(self):
        points = []
        points.append(((self.RSize[1]/2)*m.cos(self.RPos[2])-(self.RSize[0]/2)*m.sin(self.RPos[2])+self.RPos[0],(self.RSize[1]/2)*m.sin(self.RPos[2])+(self.RSize[0]/2)*m.cos(self.RPos[2])+self.RPos[1]))
        points.append(((-self.RSize[1]/2)*m.cos(self.RPos[2])-(self.RSize[0]/2)*m.sin(self.RPos[2])+self.RPos[0],(-self.RSize[1]/2)*m.sin(self.RPos[2])+(self.RSize[0]/2)*m.cos(self.RPos[2])+self.RPos[1]))
        points.append(((-self.RSize[1]/2)*m.cos(self.RPos[2])-(-self.RSize[0]/2)*m.sin(self.RPos[2])+self.RPos[0],(-self.RSize[1]/2)*m.sin(self.RPos[2])+(-self.RSize[0]/2)*m.cos(self.RPos[2])+self.RPos[1]))
        points.append(((self.RSize[1]/2)*m.cos(self.RPos[2])-(-self.RSize[0]/2)*m.sin(self.RPos[2])+self.RPos[0],(self.RSize[1]/2)*m.sin(self.RPos[2])+(-self.RSize[0]/2)*m.cos(self.RPos[2])+self.RPos[1]))
        pg.draw.polygon(screen, (0,0,0), points, 5)
    def move(self, pos):
        newPos = [self.RPos[0]+ pos[0]*m.cos(self.RPos[2]) + pos[1]*m.cos(m.pi/2 + self.RPos[2]), self.RPos[1] + pos[0]*m.sin(self.RPos[2]) + pos[1]*m.sin(m.pi/2 - self.RPos[2]), (self.RPos[2] + pos[2]) % (2*m.pi)]
        if not self.collisionDetection(newPos):
            self.RPos = newPos
    def collisionDetection(self, pos):
        diag = m.sqrt((RSize[0]/2)**2 + (RSize[1]/2)**2)
        angle = m.atan(RSize[0]/RSize[1])
        size = (max(diag*m.cos(pos[2]+angle),diag*m.cos(m.pi-pos[2]+angle), diag*m.cos((m.pi/2)-pos[2]+angle), diag*m.cos((m.pi*2)-pos[2]+angle)),32)
        return pos[0]-size[0] < 0 or pos[0]+size[0] > fieldSize or pos[1]-size[1] < 0 or pos[1]+size[1] > fieldSize



robot1 = Robot(RSize, RPos)
sensors = Sensor(DSPos, 0, IMUPos)

while True:
    keys = pygame.key.get_pressed()
    if keys[pygame.K_a]:
        robot1.move((0,0,-.001))
    if keys[pygame.K_d]:
        robot1.move((0,0,.001))
    if keys[pygame.K_w]:
        robot1.move((.1,0,0))
    if keys[pygame.K_s]:
        robot1.move((-.1,0,0))
    if keys[pygame.K_q]:
        robot1.move((0,-.1,0))
    if keys[pygame.K_e]:
        robot1.move((0,.1,0))
    for event in pg.event.get():
        if event.type == pg.QUIT:
            pg.quit()
            quit()
    screen.fill((255,255,255))
    robot1.drawRobot()
    pg.display.update()