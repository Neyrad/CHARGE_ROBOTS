import pygame
import sys
import os
import numpy as np

pygame.init()

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (96, 96, 96)
LIGHT_GREY = (192, 192, 192)
RED = (255, 0, 0)

FPS = 60

CSV_DATA = open(os.path.join('Simulation_History', 'field.csv'))
FIELD = np.loadtxt(CSV_DATA, delimiter=",")

CSV_DATA = open(os.path.join('Simulation_History', 'robot_numbers.csv'))
ROBOT_NUMBERS = np.loadtxt(CSV_DATA, delimiter=",")

LOG_DATA = os.path.join('Simulation_History', 'robots.log')
LOG = np.genfromtxt(LOG_DATA, dtype=None)
LOG = [list(i) for i in LOG]

GRIDHEIGHT = FIELD.shape[0]
GRIDWIDTH = FIELD.shape[1]
TILESIZE = 100
WIDTH = GRIDWIDTH * TILESIZE
HEIGHT = GRIDHEIGHT * TILESIZE
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("ROBOTS")

ROBOT_IMAGE_RAW = pygame.image.load(os.path.join('Assets', 'Robot_new.png'))
ROBOT_IMAGE = pygame.transform.scale(ROBOT_IMAGE_RAW, (TILESIZE, TILESIZE))

ROBOT_VER = pygame.transform.rotate(ROBOT_IMAGE, 0)
ROBOT_HOR = pygame.transform.rotate(ROBOT_IMAGE, 90)

ROBOT_BOX_IMAGE_RAW = pygame.image.load(os.path.join('Assets', 'Robot_new_with_Box.png'))
ROBOT_BOX_IMAGE = pygame.transform.scale(ROBOT_BOX_IMAGE_RAW, (TILESIZE, TILESIZE))

ROBOT_BOX_VER = pygame.transform.rotate(ROBOT_BOX_IMAGE, 0)
ROBOT_BOX_HOR = pygame.transform.rotate(ROBOT_BOX_IMAGE, 90)

BOX_IMAGE_RAW = pygame.image.load(os.path.join('Assets', 'Box.png'))
BOX_IMAGE = pygame.transform.scale(BOX_IMAGE_RAW, (TILESIZE, TILESIZE))

CONTAINER_IMAGE_RAW = pygame.image.load(os.path.join('Assets', 'Container.png'))
CONTAINER_IMAGE = pygame.transform.scale(CONTAINER_IMAGE_RAW, (TILESIZE, TILESIZE))

CHARGER_IMAGE_RAW = pygame.image.load(os.path.join('Assets', 'Charger.png'))
CHARGER_IMAGE = pygame.transform.scale(CHARGER_IMAGE_RAW, (TILESIZE, TILESIZE))

N_ROBOTS = ROBOT_NUMBERS.shape[0]
N_FRAMES = np.asarray(LOG).shape[0]

class Robot:
    def __init__(self, x, y, ver, box):
        self.x = x
        self.y = y
        self.ver = ver
        self.box = box

def draw_window(ROBOTS, cur_frame):
    WIN.fill(LIGHT_GREY)
    for y in range(GRIDHEIGHT):
        for x in range(GRIDWIDTH):
            if FIELD[y][x] == 1:
                WALL = pygame.Rect(x * TILESIZE, y *
                                   TILESIZE, TILESIZE, TILESIZE)
                pygame.draw.rect(WIN, GREY, WALL)
            elif FIELD[y][x] == 5:
                WIN.blit(BOX_IMAGE, (x * TILESIZE, y * TILESIZE))
            elif FIELD[y][x] == 3:
                WIN.blit(CONTAINER_IMAGE, (x * TILESIZE, y * TILESIZE))
            elif FIELD[y][x] == 7:
                WIN.blit(CHARGER_IMAGE, (x * TILESIZE, y * TILESIZE))

    for i in range(N_ROBOTS):
        x = ROBOTS[i].x
        y = ROBOTS[i].y

        if ROBOTS[i].ver:
            if ROBOTS[i].box:
                 WIN.blit(ROBOT_BOX_VER, (x * TILESIZE, y * TILESIZE))
            else:
                 WIN.blit(ROBOT_VER, (x * TILESIZE, y * TILESIZE))
        else:
            if ROBOTS[i].box:
                 WIN.blit(ROBOT_BOX_HOR, (x * TILESIZE, y * TILESIZE))
            else:
                 WIN.blit(ROBOT_HOR, (x * TILESIZE, y * TILESIZE))    
        
        if cur_frame == N_FRAMES: #last frame
            continue

        if chr(LOG[cur_frame][i]) == 'U':
            ROBOTS[i].y -= 1
        elif chr(LOG[cur_frame][i]) == 'D':
            ROBOTS[i].y += 1
        elif chr(LOG[cur_frame][i]) == 'L':
            ROBOTS[i].x -= 1
        elif chr(LOG[cur_frame][i]) == 'R':
            ROBOTS[i].x += 1
        elif chr(LOG[cur_frame][i]) == 'F':
            ROBOTS[i].ver = not ROBOTS[i].ver
        elif chr(LOG[cur_frame][i]) == 'I':
            ROBOTS[i].box = True
        elif chr(LOG[cur_frame][i]) == 'O':
            ROBOTS[i].box = False
        elif chr(LOG[cur_frame][i]) == 'N':
            pass
        else:
            print("Undefined input")

    for x in range(0, WIDTH, TILESIZE):
        pygame.draw.line(WIN, BLACK, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, TILESIZE):
        pygame.draw.line(WIN, BLACK, (0, y), (WIDTH, y))

    pygame.display.update()


TIME_PER_FRAME = 1

def main():
    ROBOTS = []
    for i in range(N_ROBOTS):
        ROBOTS.append(Robot(0, 0, False, False))
        ROBOTS[i].x = ROBOT_NUMBERS[i][0]
        ROBOTS[i].y = ROBOT_NUMBERS[i][1]
        ROBOTS[i].ver = ROBOT_NUMBERS[i][2]
        ROBOTS[i].box = ROBOT_NUMBERS[i][3]
    
    cur_frame = -1
    clock = pygame.time.Clock()
    frame_change_time = 0


    run = True
    while run:
        clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()
                sys.exit()
 
        if pygame.time.get_ticks() - frame_change_time > TIME_PER_FRAME:
            if (cur_frame >= N_FRAMES):
                run = False
                break
            cur_frame += 1
            #print(cur_frame)
            draw_window(ROBOTS, cur_frame)
            frame_change_time = pygame.time.get_ticks()
    
    main()


if __name__ == "__main__":
    main()
