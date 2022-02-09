import pygame as pg
import pygame.gfxdraw
import math as m
import random

pg.init()
fieldSize = 705
RSize = (64,90)
IMUPos = (0,0,0)
DSPos = ((0,45,0),(32,0,90),(0,-45,180),(-32,0,270))
assets = ((2,0,0,fieldSize,fieldSize),(1,240,300,45),(1,465,300,45),(1,10,10,37.5),(1,695,10,37.5))
acc = ((6,90),(-3,-45))
maxVel = (3,180)
rand = (-1,1)
maxRange = 250
screen = pg.display.set_mode((fieldSize, fieldSize))

class Sensor:
    def __init__(self, DSPos, rand, IMUPos):
        self.DSPos = DSPos
        self.IMUPos = IMUPos
        self.rand = rand

    def dist(self, assets, p):
        dist = 1000
        for asset in assets:
            if asset[0] == 0:
                dist = min(dist, m.sqrt(max(asset[1] - p[0], 0, p[0] - asset[3])**2 + max(asset[2] - p[1], 0, p[1] - asset[4])**2))
                pygame.draw.rect(screen, (0,0,0), (asset[1],asset[2],asset[3]-asset[1],asset[4]-asset[2]), 4)
            elif asset[0] == 1:
                dist = min(dist, m.sqrt((p[0] - asset[1])**2 + (p[1] - asset[2])**2)-asset[3])
                pygame.draw.circle(screen, (0,0,0), (asset[1],asset[2]), asset[3], 4)
            elif asset[0] == 2:
                dist = min(dist,min(p[0]-asset[1], asset[3]-p[0], p[1]-asset[2], asset[4]-p[1]))
        return dist

    def getDistance(self, RPos, DS):
        DSAbs = ((DSPos[DS][1]*m.cos(m.pi*2-RPos[2]))+(DSPos[DS][0]*m.sin(m.pi*2-RPos[2]))+RPos[0],(DSPos[DS][0]*m.cos(m.pi*2-RPos[2]))-(DSPos[DS][1]*m.sin(m.pi*2-RPos[2]))+RPos[1],RPos[2]+(DSPos[DS][2]*0.0174533))
        distance = self.dist(assets, DSAbs)
        DSVal = 0
        while distance > 1:
            distance = self.dist(assets, DSAbs)
            DSVal += distance
            #pygame.draw.circle(screen, (0,0,0), (DSAbs[0],DSAbs[1]), 4, 4)
            pygame.draw.line(screen, (255,0,0), (DSAbs[0],DSAbs[1]), (DSAbs[0]+(distance*m.cos(DSAbs[2])),DSAbs[1]+(distance*m.sin(DSAbs[2]))), 3)
            #pygame.draw.circle(screen, (210,210,210), (DSAbs[0],DSAbs[1]), distance, 1)
            DSAbs = [DSAbs[0]+(distance*m.cos(DSAbs[2])),DSAbs[1]+(distance*m.sin(DSAbs[2])), DSAbs[2]]
        return DSVal + self.getRand()

    def getRotation(self, RPos):
        return RPos[2]+self.IMUPos[2] + self.getRand()

    def getRand(self):
        return self.rand[0]+(random.random()*(self.rand[1]-self.rand[0]))

class Robot:
    def __init__(self, RSize, RPos, acc):
        self.RSize = RSize
        self.RPos = RPos
        self.acc = acc
        self.vel = [0,0,0]

    def drawRobot(self):
        points = []
        points.append(((self.RSize[1]/2)*m.cos(self.RPos[2])-(self.RSize[0]/2)*m.sin(self.RPos[2])+self.RPos[0],(self.RSize[1]/2)*m.sin(self.RPos[2])+(self.RSize[0]/2)*m.cos(self.RPos[2])+self.RPos[1]))
        points.append(((-self.RSize[1]/2)*m.cos(self.RPos[2])-(self.RSize[0]/2)*m.sin(self.RPos[2])+self.RPos[0],(-self.RSize[1]/2)*m.sin(self.RPos[2])+(self.RSize[0]/2)*m.cos(self.RPos[2])+self.RPos[1]))
        points.append(((-self.RSize[1]/2)*m.cos(self.RPos[2])-(-self.RSize[0]/2)*m.sin(self.RPos[2])+self.RPos[0],(-self.RSize[1]/2)*m.sin(self.RPos[2])+(-self.RSize[0]/2)*m.cos(self.RPos[2])+self.RPos[1]))
        points.append(((self.RSize[1]/2)*m.cos(self.RPos[2])-(-self.RSize[0]/2)*m.sin(self.RPos[2])+self.RPos[0],(self.RSize[1]/2)*m.sin(self.RPos[2])+(-self.RSize[0]/2)*m.cos(self.RPos[2])+self.RPos[1]))
        pg.draw.polygon(screen, (0,0,0), points, 5)

    def move(self, vectors):
        vectors.append(self.vel)
        self.vel = self.vectorAdd(vectors)
        self.vel[0] = min(max(self.vel[0]-.1,0),maxVel[0])
        vel = (m.cos(self.vel[1])*self.vel[0] + m.cos(self.vel[1])*self.vel[0], m.sin(self.vel[1])*self.vel[0] + m.sin(self.vel[1])*self.vel[0])
        newPos = [self.RPos[0]+ vel[0]*m.cos(self.RPos[2]) + vel[1]*m.cos(m.pi/2 + self.RPos[2]), self.RPos[1] + vel[0]*m.sin(self.RPos[2]) + vel[1]*m.sin(m.pi/2 - self.RPos[2]), 3*m.pi/2]
        if not self.collisionDetection(newPos)[0]:
            self.RPos[0] = newPos[0]
            self.RPos[2] = newPos[2]
        if not self.collisionDetection(newPos)[1]:
            self.RPos[1] = newPos[1]
            self.RPos[2] = newPos[2]

    def vectorAdd(self, vectors):
        x = 0
        y = 0
        for v in vectors:
            x += m.cos(v[1])*v[0] 
            y += m.sin(v[1])*v[0]
        return [m.sqrt(x**2+y**2), m.atan2(y, x)]

    def collisionDetection(self, pos):
        diag = m.sqrt((RSize[0]/2)**2 + (RSize[1]/2)**2)
        angle = m.atan(RSize[0]/RSize[1])
        size = (max(diag*m.cos(pos[2]+angle),diag*m.cos(m.pi-pos[2]+angle),diag*m.cos((m.pi)+pos[2]+angle),diag*m.cos((m.pi*2)-pos[2]+angle)),max(diag*m.sin(pos[2]+angle),diag*m.sin(m.pi-pos[2]+angle),diag*m.sin((m.pi)+pos[2]+angle),diag*m.sin((m.pi*2)-pos[2]+angle)))
        return (pos[0]-size[0] < 0 or pos[0]+size[0] > fieldSize, pos[1]-size[1] < 0 or pos[1]+size[1] > fieldSize)

RPos = [100,100,0]
robot1 = Robot(RSize, RPos, acc)
sensors = Sensor(DSPos, rand, IMUPos)
speed = 30
FPS = 30 # frames per second setting
fpsClock = pygame.time.Clock()


while True:
    screen.fill((255,255,255))
    accVectors = []
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        accVectors.append((1,0))
    if keys[pygame.K_d]:
        accVectors.append((1,m.pi/2))
    if keys[pygame.K_s]:
        accVectors.append((1,m.pi))
    if keys[pygame.K_a]:
        accVectors.append((1,3*m.pi/2))
        
    robot1.move(accVectors)
    for event in pg.event.get():
        if event.type == pg.QUIT:
            pg.quit()
            quit()
    print("{}, {}, {}, {}".format(sensors.getDistance(robot1.RPos, 0),sensors.getDistance(robot1.RPos, 1) ,sensors.getDistance(robot1.RPos, 2) ,sensors.getDistance(robot1.RPos, 3)))
    robot1.drawRobot()
    pg.display.update()
    fpsClock.tick(FPS)
