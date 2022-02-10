import pygame as pg
import pygame.gfxdraw
import math as m
import random

pg.init()
fieldSize = 705
PPI = 5
RAD = 0.0174533
DEG = 57.2958
RSize = (64,90)
IMUPos = (0,0,0)
DSPos = ((0,45,0),(32,0,90),(0,-45,180),(-32,0,270))
assets = [[[100,100,0],[(0,(0,0,64,90,0),(0,0,0),4),(0,(32,0,10,10,0),(0,0,0),4)],(3,0),True,(-2,-20)]]
assets.append([[240,300,0],[(1,(0,0,45),(255,0,0),4),(1,(0,0,37.5),(255,0,0),4),(1,(0,0,30),(255,0,0),4),(1,(0,0,3.25),(255,0,0),0)],(2,1,0,0),True,(-2,-20)])
assets.append([[465,300,0],[(1,(0,0,45),(0,0,255),4),(1,(0,0,37.5),(0,0,255),4),(1,(0,0,30),(0,0,255),4),(1,(0,0,3.25),(0,0,255),0)],(2,1,0,0),True,(-2,-20)])
assets.append([[10,10,0],[(1,(0,0,37.5),(0,0,0),4)],(2),False])
assets.append([[695,10,0],[(1,(0,0,37.5),(0,0,0),4)],(2),False])
acc = (6,90)
maxVel = (20,150)
rand = (-1,1)
maxRange = 250
screen = pg.display.set_mode((fieldSize, fieldSize))


class Assets:
    def __init__(self, assets):
        self.assets = assets
    def addAsset (self, asset):
        self.assets.append(asset)
    def dist(self, p):
        dist = 1000
        for asset in assets:
            if(asset[0][0]):
                for s in asset[1]:
                    d = s[1]
                    if s[0] == 0:
                        dist = min(dist, m.sqrt(max(d[0] - p[0], 0, p[0] - d[2])**2 + max(d[1] - p[1], 0, p[1] - d[3])**2))
                    elif s[0] == 1:
                        dist = min(dist, m.sqrt((p[0] - d[0])**2 + (p[1] - d[1])**2)-d[2])
                    elif s[0] == 2:
                        dist = min(dist,min(p[0]-d[0], d[2]-p[0], p[1]-d[1], d[3]-p[1]))
        return dist
    def drawAssets(self):
        for asset in self.assets:
            for s in asset[1]:
                d = s[1]
                if s[0] == 0:
                    p = lambda x,y : (((y*d[3]/2)+d[1])*m.cos(d[4]+asset[0][2])-((x*d[2]/2)+d[0])*m.sin(d[4]+asset[0][2])+asset[0][0]-d[0],((y*d[3]/2)+d[1])*m.sin(d[4]+asset[0][2])+((x*d[2]/2)+d[0])*m.cos(d[4]+asset[0][2])+asset[0][1]-d[1])
                    pygame.draw.polygon(screen, s[2], (p(1,1),p(1,-1),p(-1,-1),p(-1,1)), s[3])
                elif s[0] == 1:
                    pygame.draw.circle(screen, s[2], (d[0]+asset[0][0],d[1]+asset[0][1]), d[2], s[3])
    def moveAsset(self, num, v):
        d = self.assets[num][0]
        self.assets[num][0] = [d[0]+v[0]*m.cos(v[1]),d[1]+v[0]*m.sin(v[1]),d[2]+v[1]]
    def checkCollision(self, num):
        pass
        


class Sensor:
    def __init__(self, DSPos, rand, IMUPos):
        self.DSPos = DSPos
        self.IMUPos = IMUPos
        self.rand = rand

    def getDistance(self, RPos, DS):
        DSAbs = ((DSPos[DS][1]*m.cos(m.pi*2-RPos[2]))+(DSPos[DS][0]*m.sin(m.pi*2-RPos[2]))+RPos[0],(DSPos[DS][0]*m.cos(m.pi*2-RPos[2]))-(DSPos[DS][1]*m.sin(m.pi*2-RPos[2]))+RPos[1],RPos[2]+(DSPos[DS][2]*0.0174533))
        distance = field1.dist(assets, DSAbs)
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
    def __init__(self, asset, DSPos, IMUPos, rand):
        sensors = Sensor(DSPos, IMUPos, rand)
        self.asset = asset
        self.RSize = (asset[0][0][3],asset[0][0][4])
        self.RPos = RPos
        self.acc = acc
        self.vel = [0,0,0]

    def move(self, vectors):
        vectors.append(self.vel)
        self.vel = self.vectorAdd(vectors)
        self.vel = [min(max(self.vel[0]+acc[1][0]/FPS*PPI,0),maxVel[0]/FPS*PPI),self.vel[1], min(max(0,self.vel[2]+acc[1][1]/FPS*RAD),maxVel[1]/FPS*RAD)if self.vel[2] > 0 else max(min(0,self.vel[2]-acc[1][1]/FPS*RAD),-1*maxVel[1]/FPS*RAD)]
        vel = (m.cos(self.vel[1])*self.vel[0] + m.cos(self.vel[1])*self.vel[0], m.sin(self.vel[1])*self.vel[0] + m.sin(self.vel[1])*self.vel[0])
        newPos = [self.RPos[0]+ vel[0]*m.cos(self.RPos[2]) + vel[1]*m.cos(m.pi/2 + self.RPos[2]), self.RPos[1] + vel[0]*m.sin(self.RPos[2]) + vel[1]*m.sin(m.pi/2 - self.RPos[2]), self.RPos[2]+self.vel[2]]
        if not self.collisionDetection(newPos)[0]:
            self.RPos[0] = newPos[0]
            self.RPos[2] = newPos[2]
        if not self.collisionDetection(newPos)[1]:
            self.RPos[1] = newPos[1]
            self.RPos[2] = newPos[2]

    def vectorAdd(self, vectors):
        x = 0
        y = 0
        a = 0
        for v in vectors:
            x += m.cos(v[1])*v[0] 
            y += m.sin(v[1])*v[0]
            a += v[2]
        return [m.sqrt(x**2+y**2), m.atan2(y, x), a]

    def collisionDetection(self, pos):
        diag = m.sqrt((RSize[0]/2)**2 + (RSize[1]/2)**2)
        angle = m.atan(RSize[0]/RSize[1])
        size = (max(diag*m.cos(pos[2]+angle),diag*m.cos(m.pi-pos[2]+angle),diag*m.cos((m.pi)+pos[2]+angle),diag*m.cos((m.pi*2)-pos[2]+angle)),max(diag*m.sin(pos[2]+angle),diag*m.sin(m.pi-pos[2]+angle),diag*m.sin((m.pi)+pos[2]+angle),diag*m.sin((m.pi*2)-pos[2]+angle)))
        return (pos[0]-size[0] < 0 or pos[0]+size[0] > fieldSize, pos[1]-size[1] < 0 or pos[1]+size[1] > fieldSize)

RPos = [100,100,0]
field1 = Assets(assets)
#robot1 = Robot(assets[1], DSPos, IMUPos, rand)
FPS = 30
fpsClock = pygame.time.Clock()


while True:
    screen.fill((255,255,255))
    accVectors = []
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        accVectors.append((acc[0][0]/FPS*PPI,0,0))
    if keys[pygame.K_e]:
        accVectors.append((acc[0][0]/FPS*PPI,m.pi/2,0))
    if keys[pygame.K_s]:
        accVectors.append((acc[0][0]/FPS*PPI,m.pi,0))
    if keys[pygame.K_q]:
        accVectors.append((acc[0][0]/FPS*PPI,3*m.pi/2,0))
    if keys[pygame.K_a]:
        accVectors.append((0,0,-acc[0][1]/FPS*RAD))
    if keys[pygame.K_d]:
        accVectors.append((0,0,acc[0][1]/FPS*RAD))
    
        
    #robot1.move(accVectors)
    for event in pg.event.get():
        if event.type == pg.QUIT:
            pg.quit()
            quit()
    #print("{}, {}, {}, {}".format(sensors.getDistance(robot1.RPos, 0),sensors.getDistance(robot1.RPos, 1) ,sensors.getDistance(robot1.RPos, 2) ,sensors.getDistance(robot1.RPos, 3)))
    field1.drawAssets()
    field1.moveAsset(0, (0,.01))
    pg.display.update()
    fpsClock.tick(FPS)
