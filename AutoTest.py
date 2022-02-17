from turtle import circle
import pygame as pg
import pygame.gfxdraw
import math as m
import random

PPI = 5
RAD = 0.0174533
DEG = 57.2958
FPS = 45
FSize = 705

class Field:
    def __init__(self, robots, assets):
        self.assets = assets
        self.robots = robots
    def addAsset (self, asset):
        self.assets.append(asset)
    def addRobot (self, robot):
        self.robots.append(robot)
    def stepSim (self):
        for a in self.assets:
            if a.movable and sum(a.vel) != 0:
                a.move(self.assets)
    def readSensors(self):
        for robot in self.robots:
            for sensor in robot.sensors:
                sensor.read(self.assets, robot.asset.pos)
    def drawField(self):
        for asset in self.assets:
            asset.drawAsset()

class Asset:
    def __init__(self, asset):
        self.pos = asset[0]
        self.shapes = asset[1]
        self.behavior = asset[2]
        self.movable = asset[3]
        if self.movable:
            self.vel = asset[4]
            self.speed = asset[5]
    def getPos(self):
        return self.pos
    def drawAsset(self):
        for s in self.shapes:
            d = s[1]
            if s[0]%2 == 0:
                p = lambda x,y : (((x*d[2]/2+d[0])*m.cos(d[4]+self.pos[2]))-((y*d[3]/2+d[1])*m.sin(d[4]+self.pos[2]))+self.pos[0],((x*d[2]/2+d[0])*m.sin(d[4]+self.pos[2]))+((y*d[3]/2+d[1])*m.cos(d[4]+self.pos[2]))+self.pos[1])
                pygame.draw.polygon(screen, s[2], (p(1,1),p(1,-1),p(-1,-1),p(-1,1)), s[3])
            elif s[0] == 1:
                pygame.draw.circle(screen, s[2], (d[0]+self.pos[0],d[1]+self.pos[1]), d[2], s[3])
    def move(self, assets):
        if self.movable:
            newPos = [self.pos[0]+self.vel[0]*m.cos(self.vel[1]+m.pi/2),self.pos[1]+self.vel[0]*m.sin(self.vel[1]+m.pi/2),(self.pos[2]+self.vel[2])%(m.pi*2)]
            if (not self.collisionDetection(assets, newPos)):
                self.pos = newPos
                self.vel = [max(self.vel[0]+(self.speed[2]/FPS*PPI),0),self.vel[1],max(0,self.vel[2]+(self.speed[3]*RAD/FPS)) if self.vel[2] > 0 else min(0,self.vel[2]-(self.speed[3]*RAD/FPS))] 
            else:
                self.vel = [0,0,0]
    def drive(self, v):
        if self.movable:
            x = m.cos(v[1])*v[0]+m.cos(self.vel[1])*self.vel[0]
            y = m.sin(v[1])*v[0]+m.sin(self.vel[1])*self.vel[0]
            a = self.vel[2]+v[2]
            self.vel = [min(m.sqrt(x**2+y**2),self.speed[0]/FPS*PPI),m.atan2(y,x),min(self.speed[1]*RAD/FPS,a) if a > 0 else max(-self.speed[1]*RAD/FPS,a)]
    def moveTo(self, pos):
        self.pos = pos
    def collisionDetection(self, assets, newPos):
        #dist = lambda x1,x2,y1,y2: m.sqrt((x2-x1)**2+(y1-y2)**2)
        rd = self.shapes[0][1]
        for asset in assets:
            for i, s in enumerate(asset.shapes):
                if asset.behavior[i] > 1 and self.shapes[0][0] == 0:
                    d = s[1]
                    normPos = [(d[0]+asset.pos[0])-(newPos[0]+rd[0]),(d[1]+asset.pos[1])-(newPos[1]+rd[1])]
                    if s[0] == 1:
                        newCenter = [normPos[0]*m.cos(2*m.pi-newPos[2])-normPos[1]*m.sin(2*m.pi-newPos[2])+newPos[0]+rd[0],normPos[0]*m.sin(2*m.pi-newPos[2])+normPos[1]*m.cos(2*m.pi-newPos[2])+newPos[1]+rd[1]]
                        pg.draw.circle(screen, (0,255,50), newCenter, d[2], 4)
                        pg.draw.rect(screen, (0,255,50), (newPos[0]-rd[2]/2,newPos[1]-rd[3]/2,rd[2],rd[3]), 4)
                        circleDist = [abs(newCenter[0]-newPos[0]), abs(newCenter[1]-newPos[1])]
                        cornerDist = (circleDist[0] - rd[2]/2)**2 + (circleDist[1] - rd[3]/2)**2 
                        if not(circleDist[0] > (rd[2]/2 + d[2]) or circleDist[1] > (rd[3]/2 + d[2])) and((circleDist[0] <= rd[2]/2) or (circleDist[1] <= rd[3]/2) or (cornerDist <= d[2]**2)):
                            direction = m.pi/2-(max(0,min(m.pi/2,m.atan2(circleDist[1] - rd[3]/2,circleDist[0] - rd[2]/2)))+ ((self.pos[2])%(m.pi/2))-m.pi/2)
                            asset.drive((10,direction,0))
                            return True
                    if s[0] == 3:
                        diag = m.sqrt((rd[2]/2)**2 + (rd[3]/2)**2)
                        angle = m.atan(rd[3]/rd[2])
                        size = (max(diag*m.cos(newPos[2]+angle),diag*m.cos(m.pi-newPos[2]+angle),diag*m.cos((m.pi)+newPos[2]+angle),diag*m.cos((m.pi*2)-newPos[2]+angle)),max(diag*m.sin(newPos[2]+angle),diag*m.sin(m.pi-newPos[2]+angle),diag*m.sin((m.pi)+newPos[2]+angle),diag*m.sin((m.pi*2)-newPos[2]+angle)))
                        if (newPos[0]-size[0] < s[0]+s[2]+asset[0][0] and newPos[1]+size[1] < FSize or newPos[1]-size[1] < 0 or newPos[1]+size[1] > FSize):
                            return True
                    if s[0] == 2:
                        diag = m.sqrt((rd[2]/2)**2 + (rd[3]/2)**2)
                        angle = m.atan(rd[3]/rd[2])
                        size = (max(diag*m.cos(newPos[2]+angle),diag*m.cos(m.pi-newPos[2]+angle),diag*m.cos((m.pi)+newPos[2]+angle),diag*m.cos((m.pi*2)-newPos[2]+angle)),max(diag*m.sin(newPos[2]+angle),diag*m.sin(m.pi-newPos[2]+angle),diag*m.sin((m.pi)+newPos[2]+angle),diag*m.sin((m.pi*2)-newPos[2]+angle)))
                        if (newPos[0]-size[0] < 0 or newPos[0]+size[0] > d[2] or newPos[1]-size[1] < 0 or newPos[1]+size[1] > d[3]):
                            return True
        return False

        
class Sensor:
    reading = 0
    def __init__(self, pos, type, rand):
        self.pos = pos
        self.type = type
        self.rand = rand
    def read(self, assets, RPos):
        if self.type == 0:
            p = [(self.pos[0]*m.cos(RPos[2]))-(self.pos[1]*m.sin(RPos[2]))+RPos[0],(self.pos[0]*m.sin(RPos[2]))+(self.pos[1]*m.cos(RPos[2]))+RPos[1],RPos[2]+(self.pos[2]*RAD)]
            DSVal = 0
            dist = 1000
            while dist > 1:
                dist = 1000
                for asset in assets:
                    for i, s in enumerate(asset.shapes):
                        if asset.behavior[i]%2 == 1:
                            d = s[1]
                            if s[0] == 0:
                                dist = min(dist, m.sqrt(max(d[0] - p[0], 0, p[0] - d[2])**2 + max(d[1] - p[1], 0, p[1] - d[3])**2))
                            elif s[0] == 1:
                                dist = min(dist, m.sqrt((p[0] - (d[0]+asset.pos[0]))**2 + (p[1] - (d[1]+asset.pos[1]))**2)-d[2])
                            elif s[0] == 2:
                                dist = min(dist,min(p[0]-d[0], d[2]-p[0], p[1]-d[1], d[3]-p[1]))
                DSVal += dist
                pygame.draw.line(screen, (255,100,100), (p[0],p[1]), (p[0]+(dist*m.cos(p[2])),p[1]+(dist*m.sin(p[2]))), 2)
                pygame.draw.circle(screen, (240,240,240), (p[0],p[1]), dist, 1)
                p = [p[0]+(dist*m.cos(p[2])),p[1]+(dist*m.sin(p[2])), p[2]]
            self.reading = DSVal + self.getRand()
        elif self.type == 1:
            self.reading = RPos[2]+self.pos[2] + self.getRand()
    def getRand(self):
        return self.rand[0]+(random.random()*(self.rand[1]-self.rand[0]))

class Robot:
    def __init__(self, asset, sensors):
        self.sensors = sensors
        self.asset = asset
        self.RSize = (asset.shapes[0][1][2],asset.shapes[0][1][3])

    def drive(self, vectors):
        x = 0
        y = 0
        a = 0
        for v in vectors:
            x += m.cos(v[1]+self.asset.pos[2])*v[0] 
            y += m.sin(v[1]+self.asset.pos[2])*v[0]
            a += v[2]
        self.asset.drive([m.sqrt(x**2+y**2), m.atan2(y, x), a])

    # def collisionDetection(self, pos):
    #     diag = m.sqrt((self.RSize[0]/2)**2 + (self.RSize[1]/2)**2)
    #     angle = m.atan(self.RSize[0]/self.RSize[1])
    #     size = (max(diag*m.cos(pos[2]+angle),diag*m.cos(m.pi-pos[2]+angle),diag*m.cos((m.pi)+pos[2]+angle),diag*m.cos((m.pi*2)-pos[2]+angle)),max(diag*m.sin(pos[2]+angle),diag*m.sin(m.pi-pos[2]+angle),diag*m.sin((m.pi)+pos[2]+angle),diag*m.sin((m.pi*2)-pos[2]+angle)))
    #     return (pos[0]-size[0] < 0 or pos[0]+size[0] > fieldSize, pos[1]-size[1] < 0 or pos[1]+size[1] > fieldSize)



pg.init()

screen = pg.display.set_mode((FSize, FSize))


r = (-1,1)
assets = [Asset([[100,100,0],[(0,(0,0,64,90,0),(0,0,0),4),(0,(0,32,10,10,0),(0,0,0),4)],(2,0),True,[0,0,0],(45,90,-1.5,-20)]),Asset([[FSize/2,FSize/2,0],[(2,(0,0,FSize,FSize,0),(0,0,0),4)],[3],False]),Asset([[240,300,0],[(1,(0,0,45),(255,0,0),4),(1,(0,0,37.5),(255,0,0),4),(1,(0,0,30),(255,0,0),4),(1,(0,0,3.25),(255,0,0),0)],(2,1,0,0),True,[0,0,0],(20,150,-2,-20)]),Asset([[465,300,0],[(1,(0,0,45),(0,0,255),4),(1,(0,0,37.5),(0,0,255),4),(1,(0,0,30),(0,0,255),4),(1,(0,0,3.25),(0,0,255),0)],(2,1,0,0),True,[0,0,0],(20,150,-2,-20)]),Asset([[10,10,0],[(1,(0,0,37.5),(0,0,0),4)],[2],False]),Asset([[695,10,0],[(1,(0,0,37.5),(0,0,0),4)],[2],False]),Asset([[176.25,57.5,0],[(0,(52.5,0,10,115,0),(255,0,0),0),(0,(-52.5,0,10,115,0),(255,0,0),0),(0,(0,52.5,115,10,0),(255,0,0),0)],[0,0,0],False]),Asset([[528.75,57.5,0],[(0,(52.5,0,10,115,0),(0,0,255),0),(0,(-52.5,0,10,115,0),(0,0,255),0),(0,(0,52.5,115,10,0),(0,0,255),0)],[0,0,0],False]),Asset([[231.4,561.875,0],[(0,(0,0,27.8,148.75,0),(0,0,0),5)],[2],False]),Asset([[142.875,473.5,0],[(0,(0,0,148.75,27.8,0),(0,0,0),5)],[2],False]),Asset([[284.875,473.5,0],[(0,(0,0,135.25,27.8,0),(0,0,0),5)],[2],False]),Asset([[420.125,473.5,0],[(0,(0,0,135.25,27.8,0),(0,0,0),5)],[2],False]),Asset([[473.6,561.875,0],[(0,(0,0,27.8,148.75,0),(0,0,0),5)],[2],False]),Asset([[562.125,473.5,0],[(0,(0,0,148.75,27.8,0),(0,0,0),5)],[2],False])]

sensors = [Sensor((0,45,90),0,r),Sensor((32,0,0),0,r),Sensor((0,-45,270),0,r),Sensor((-32,0,180),0,r),Sensor((0,0,0),1,r)]
robots = [Robot(assets[0],sensors)]
field1 = Field(robots,assets)
acc = (13,80)


fpsClock = pygame.time.Clock()

while True:
    screen.fill((255,255,255))
    accVectors = []
    keys = pygame.key.get_pressed()
    if keys[pygame.K_w]:
        accVectors.append((acc[0]/FPS*PPI,0,0))
    if keys[pygame.K_e]:
        accVectors.append((acc[0]/FPS*PPI,m.pi/2,0))
    if keys[pygame.K_s]:
        accVectors.append((acc[0]/FPS*PPI,m.pi,0))
    if keys[pygame.K_q]:
        accVectors.append((acc[0]/FPS*PPI,3*m.pi/2,0))
    if keys[pygame.K_a]:
        accVectors.append((0,0,-acc[1]*RAD/FPS))
    if keys[pygame.K_d]:
        accVectors.append((0,0,acc[1]*RAD/FPS))
    
        
    for event in pg.event.get():
        if event.type == pg.QUIT:
            pg.quit()
            quit()
    robots[0].drive(accVectors)
    field1.stepSim()
    field1.drawField()
    field1.readSensors()
    pg.display.update()
    fpsClock.tick(FPS)
