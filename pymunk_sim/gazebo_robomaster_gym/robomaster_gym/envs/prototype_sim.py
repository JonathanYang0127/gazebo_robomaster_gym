"""Port of the Chipmunk tank demo. Showcase a topdown tank driving towards the
mouse, and hitting obstacles on the way.
"""

import random
from math import *
from copy import deepcopy as copy

import pygame
from pygame.locals import *

import pymunk
from pymunk.vec2d import Vec2d
import pymunk.pygame_util

from robomaster_env import RobomasterEnv
from robomaster_gym.misc import *

RATIO = .10
SCREEN = (int(8100*RATIO), int(5100*RATIO))
MAX_VEL = 200

class PymunkEnv(RobomasterEnv):
    def __init__(self, dt):
        super().__init__()
        
        self._dt = dt
        space = pymunk.Space()
        space.iterations = 10
        space.sleep_time_threshold = 0.5

        static_body = space.static_body
        
        # Create segments around the edge of the screen.
        shape = pymunk.Segment(static_body, (0,0), (0,SCREEN[1]), 1.0)
        space.add(shape)
        shape.elasticity = 0
        shape.friction = 10

        shape = pymunk.Segment(static_body, (SCREEN[0],0), SCREEN, 1.0)
        space.add(shape)
        shape.elasticity = 0
        shape.friction = 10
        
        shape = pymunk.Segment(static_body, (0,0), (SCREEN[0],0), 1.0)
        space.add(shape)
        shape.elasticity = 0
        shape.friction = 10
        
        shape = pymunk.Segment(static_body, (0,SCREEN[1]), SCREEN, 1.0)
        space.add(shape)
        shape.elasticity = 0
        shape.friction = 10

        for i in range(0,len(self.segments),4):
            vertices = [(int(s[0]*1000*RATIO), int(s[1]*1000*RATIO)) for s in self.segments[i:i+4]]
            shape = pymunk.Poly(static_body, vertices)
            shape.friction = 10
            shape.color = (0,255,255,255)
            space.add(shape)

        generate_tank = PymunkEnv.generate_tank
        self._tank_bodies = [generate_tank(space,(500,500),0), generate_tank(space,(500,4600),0), generate_tank(space,(7600,500),pi), generate_tank(space,(7600,4600),pi)]
        self.space = space
        self._robot_coords = [self.get_coords(i) for i in range(4)]
        self._robot_plates_coords = [self.get_armor(i) for i in range(4)]
        self._odom_info = [(500*RATIO,500*RATIO,0), (500*RATIO,4600*RATIO,0), (7600*RATIO,500*RATIO,pi), (7600*RATIO,4600*RATIO,pi)]
        self._gimbal_angle = [0,0,0,0]

    def pub_cmds(self, _id, action, gimbal):
        """Publish commands for gimbal and robot into simulation"""
        tank_body, tank_control_body = self._tank_bodies[_id]
        space = self.space

        vel = Vec2d(action[0:2])-tank_body.position
        angle = action[2]

        if vel.length > 100:
            if abs(tank_body.angle-vel.angle)%pi < .7*pi:
                print("called")
                tank_control_body.angle = vel.angle
                active_rotation_vector = tank_body.rotation_vector
            else:
                tank_control_body.angle = (vel.angle+pi)%pi
                active_rotation_vector = tank_body.rotation_vector.cpvrotate(Vec2d(-1,0))

            if vel.dot(active_rotation_vector) > 0.0:
                direction = 1.0 
            else:
                direction = -1.0
            dv = Vec2d(30.0*direction, 0.0) # set velocity
            tank_control_body.velocity = active_rotation_vector.cpvrotate(dv)
        else:
            tank_control_body.angle = angle
            tank_control_body.velocity = 0,0

        self._gimbal_angle[_id] = min(0.698132, max(-.698132, gimbal))
        
        space.step(self._dt)
    
    def update_robot_coords(self, _id):
        """Update self.robot_coords[_id] and self.robot_plates_coords[_id]"""
        self._robot_coords[_id] = self.get_coords(_id)
        self._robot_plates_coords[_id] = self.get_armor(_id)
        self._odom_info[_id] = (*self._tank_bodies[_id][0].position,self._tank_bodies[_id][0].angle)

    def get_armor(self,_id):
        shapes = list(self._tank_bodies[_id][0].shapes)
        vertices = []
        for shape in shapes:
            if getattr(shape,'get_vertices',None):
                vertices = [v.rotated(shape.body.angle) + shape.body.position for v in shape.get_vertices()]
        armor = []
        for i in range(4):
            j = (i+1)%4
            armor.append((*vertices[i],*vertices[j]))
        return armor

    def get_coords(self, _id):
        shapes = list(self._tank_bodies[_id][0].shapes)
        for shape in shapes:
            if getattr(shape,'get_vertices',None):
                vertices = [v.rotated(shape.body.angle) + shape.body.position for v in shape.get_vertices()]
                return [*vertices[0],*vertices[1],*vertices[2],*vertices[3]]

    def get_hit(self, _id):
        pos, angle = self._tank_bodies[_id][0].position, self._gimbal_angle[_id]
        line_of_sight = (Vec2d(cos(angle)*1e5,sin(angle*1e5)) + pos).int_tuple

        for segment in self.segments:
            segment = tuple((p*1000*RATIO for  p in segment))
            if lines_cross(*segment,*(*line_of_sight,*pos.int_tuple)):
                line_of_sight = seg_intersect(*segment,*(*line_of_sight,*pos.int_tuple))

        hit_robot = None
        
        other_plate_coords = copy(self._robot_plates_coords)
        del other_plate_coords[_id]
        for i, robot in enumerate(other_plate_coords):
            for j, plate in enumerate(robot):
                if lines_cross(*plate,*(*line_of_sight,*pos.int_tuple)):
                    line_of_sight = seg_intersect(*plate,*(*line_of_sight,*pos.int_tuple))
                    hit_robot = (i,j)
                    print("robot hit")

        return hit_robot if hit_robot else (-1,-1)


    def generate_tank(space, center, angle):
        # We joint the tank to the control body and control the tank indirectly by modifying the control body.
        tank_control_body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
        tank_control_body.position = 320,480
        space.add(tank_control_body)

        tank_body = pymunk.Body()
        tank_body.position = int(center[0]*RATIO),int(center[1]*RATIO)
        space.add(tank_body)
        tank = pymunk.Poly.create_box(tank_body, (550*RATIO, 420*RATIO), 0.0)
        turret = pymunk.Circle(tank_body, 100*RATIO, (50*RATIO,0))
        tank.mass = 1
        tank.friction = 10
        tank.color = (0,255,100,255)
        turret.color = (0,100,100,100)
        space.add(tank)
        space.add(turret)

        pivot = pymunk.PivotJoint(tank_control_body, tank_body, (0,0), (0,0))
        space.add(pivot)
        pivot.max_bias = 0 # disable joint correction
        pivot.max_force = 10000 # emulate linear friction
        
        gear = pymunk.GearJoint(tank_control_body, tank_body, 0.0, 1.0)
        space.add(gear)    
        gear.error_bias = 0 # attempt to fully correct the joint each step
        gear.max_bias = 1.2  # but limit it's angular correction rate
        gear.max_force = 10000 # emulate angular friction

        pivot1 = pymunk.PivotJoint(tank_control_body, tank_body, (0,0), (0,0))
        space.add(pivot1)
        pivot1.max_bias = 0 # disable joint correction

        return (tank_body, tank_control_body)       

def KeyboardAgent(surface,env):
    key_dir = Vec2d(0,0) # key_dir exact length does not matter
    pressed = pygame.key.get_pressed()
    if pressed[pygame.K_a]:
        key_dir = Vec2d(-1e5,0)
    if pressed[pygame.K_w]:
        key_dir = Vec2d(0,1e5)
    if pressed[pygame.K_d]:
        key_dir = Vec2d(1e5,0)
    if pressed[pygame.K_s]:
        key_dir = Vec2d(0,-1e5)
    mpos = pygame.mouse.get_pos()
    mouse_pos = Vec2d(pymunk.pygame_util.from_pygame(Vec2d(mpos), surface))
    mouse_angles = [(mouse_pos-env._tank_bodies[i][0].position).angle - env._tank_bodies[i][0].angle for i in range(4)]
    angles = [env._tank_bodies[i][0].angle for i in range(4)]
    pos = [env._odom_info[i][0:2] for i in range(4)]

    mclick = pygame.mouse.get_pressed()[0]
    
    return [(*(key_dir if key_dir.length > 0 else pos[i]),angles[i],mclick,mouse_angles[i]) for i in range(4)]

if __name__ == '__main__':
    fps = 60.0
    env = PymunkEnv(1/fps)
    pygame.init()
    screen = pygame.display.set_mode(SCREEN) 
    clock = pygame.time.Clock()
    draw_options = pymunk.pygame_util.DrawOptions(screen)

    while True:
        for event in pygame.event.get():
            if event.type == QUIT or pygame.key.get_pressed()[K_ESCAPE]: 
                exit()
        
        screen.fill(pygame.color.THECOLORS["black"])
        env.space.debug_draw(draw_options)
        env.step(KeyboardAgent(screen,env))
        pygame.display.flip()
        
        clock.tick(fps)
