#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt

from team52_interfaces.msg import Obstacles
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


from math import sqrt


def norm(v):
    return sqrt(v[0]**2 + v[1]**2)

# calcul de la soustraction du vecteur v par le vecteur u
def vectsub(v, u):
    return(v[0] - u[0], v[1] - u[1])

def vectadd(v, u):
    return(v[0] + u[0], v[1] + u[1])

def scalar(a, v):
    return(a * v[0], a * v[1])

# renvoie vrai si le point q est sur le segment [p,r]
def onSegment(p, q, r):
    if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
       return True
    return False


# orientation du triplet (p,q,r)
# renvoie :
# 0 -> les trois points sont allignés
# 1 -> sens horaire
# 2 -> sens anti-horaire
def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0
    elif val > 0:
        return 1
    else:
        return 2


class PathFinder(Node):
    def __init__(self):
        super().__init__('path_finder')

        # list pour plot (debug) # 
        self.waypoints_x = []
        self.waypoints_y = []

        self.goal = (0.0, 0.0)
        self.pos = (0.0, 0.0)
        self.beacon = (0.0, 0.0)
        self.enemy = (0.0, 0.0)
        self.enemy_decay = 5 # pour savoir si on ne voit plus l'ennemi
        self.obstacles = []
        self.allies = []
        self.no_obstacle = False
        self.got_gps = False
        self.got_goal = False
        self.eps = 0.0001

        self.waypoint = (0.0, 0.0)

        # subscribers #
        self.gps = self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_gatherer, 10)
        self.sub_goal = self.create_subscription(NavSatFix, '/team52/goal', self.goal_callback, 10) # subscriber to get the coord to go to
        self.sub_obstacles = self.create_subscription(Obstacles, "/team52/obstacles", self.obstacles_callback, 10) # subsciber to get the list of coord of obstacles to avoid
        self.sub_allies = self.create_subscription(Obstacles, '/team52/boat_obstacles', self.allies_callback, 10)
        self.sub_beacon = self.create_subscription(NavSatFix, '/team52/beacon', self.beacon_callback, 10)
        self.sub_enemy = self.create_subscription(NavSatFix, '/team52/enemy',self.enemy_callback, 10)

        # publishers #
        self.pub_waypoint = self.create_publisher(NavSatFix, '/team52/waypoint', 10)

        self.get_logger().info("Running")
 
    # get position #
    def gps_gatherer(self, msg):
        self.pos = (msg.longitude, msg.latitude)
        self.got_gps = True

    # get final objective #
    def goal_callback(self, msg):
        self.goal = (msg.longitude, msg.latitude)
        self.got_goal = True

    def beacon_callback(self, msg):
        self.beacon = (msg.longitude, msg.latitude)
    
    def enemy_callback(self, msg):
        self.enemy = (msg.longitude, msg.latitude)
        self.enemy_decay = 5
        self.get_logger().info("\nfound enemy !")

    def allies_callback(self, msg):
        self.allies = []
        # On traduit le msg pour simplicité #
        for i in range(len(msg.gps_list)):
            # On récupère les nouveaux points #
            self.allies.append((msg.gps_list[i].longitude, msg.gps_list[i].latitude))

    # function qui garde les obstacles si les nouveaux points ne sont pas très loins de ce que l'on a avant, en éspérant que ca stabilise tout ca
    def merge_obst(self, lst):
        n = len(self.obstacles)
        # pour se souvenir de ceux qui sont a nouveaux dans lst #
        found = [0] * n

        for i in range(0, len(lst), 2):
            alreadyIn = False
            for j in range(0, n, 2):
                # Si la variation est suffisament faible #
                if (norm(vectsub(lst[i], self.obstacles[j])) < self.eps and norm(vectsub(lst[i+1], self.obstacles[j+1]))) or (norm(vectsub(lst[i], self.obstacles[j+1])) < self.eps and norm(vectsub(lst[i+1], self.obstacles[j]))):
                    # pas besoin de le rajouter #
                    found[j] = 1
                    alreadyIn = True
                    break
            # Sinon on le rajoute #
            if not alreadyIn:
                self.obstacles.append(lst[i])
                self.obstacles.append(lst[i+1])

        # à l'envers pour éviter des problèmes avec le pop (et toujours 2 par 2) #
        for j in range(n-2, -2, -2): 
            # si pas de nouveau dans lst, il dégage #
            if found[j] == 0:
                self.obstacles.pop(j+1)
                self.obstacles.pop(j)


    def intersect(self, p, q, r): # p et q doivent etre les 2 points du segments, et la fonction renvoie vrai si ce segment coupe le segment [pos,r] où r est le waypoint
        # On récupère les orientation utiles #
        o1 = orientation(p, q, self.pos) 
        o2 = orientation(p, q, r) 
        o3 = orientation(self.pos, r, p) 
        o4 = orientation(self.pos, r, q)

        # Cas général #
        if ((o1 != o2) and (o3 != o4)): 
            return True
        
        # Cas particulier -> un point est sur l'un des segments
        if ((o1 == 0) and onSegment(p, self.pos, q)): 
            return True
    
        if ((o2 == 0) and onSegment(p, r, q)): 
            return True
    
        if ((o3 == 0) and onSegment(self.pos, p, r)): 
            return True
    
        if ((o4 == 0) and onSegment(self.pos, q, r)): 
            return True
     
        # Si aucun des cas n'a été relevé #
        return False
    
  

    # Done le waypoint qui correspond au chemin le plus court vers l'objectif, en évitant les obstacles visible actuellement
    def get_shortest_dist(self, objective):
        # On boucle sur les paires de points #
        # je viens de me rendre compte que tu passais la taille automatiquement, donc je peux tout changer si tu veux
        for i in range(0, len(self.obstacles), 2):

            # On récupère les points déjà éloigné d'une certaine distance pour éviter que le bateau passe trop près des rochers #

            # Petit check au cas ou les deux points sont superposés (ca arrive) #
            if norm(vectsub(self.obstacles[i+1], self.obstacles[i])) == 0:
                way1=self.obstacles[i]
                way2=self.obstacles[i+1]
            else:
                way1 = vectadd(self.obstacles[i], scalar(0.0007 / norm(vectsub(self.obstacles[i+1], self.obstacles[i])), vectsub(self.obstacles[i+1], self.obstacles[i])))
                way2 = vectadd(self.obstacles[i+1], scalar(0.0007 / norm(vectsub(self.obstacles[i], self.obstacles[i+1])), vectsub(self.obstacles[i], self.obstacles[i+1])))
                
            # Si il est sur le passage #
            if self.intersect(way1, way2, objective):
                # On compare les distances #
                dist1 = norm(vectsub(way1, self.pos)) + norm(vectsub(objective, way1))
                dist2 = norm(vectsub(way2, self.pos)) + norm(vectsub(objective, way2))
                
                # On renvoie la plus petite des deux #
                if dist1 < dist2:
                    ###print("1 < 2")
                    return way1, i
                else:
                    ###print("2 < 1")
                    return way2, i
                
        # Si on arrive ici, aucun obstacle est sur le chemin, le waypoint c'est l'objectif donné
        self.no_obstacle = True
        return objective, 0
                

    def obstacles_callback(self, msg):
        if (not self.got_gps) or (not self.got_goal):
            return
        
        ###self.get_logger().info("\ndist to beac = %s" % norm(vectsub(self.pos, self.beacon)))
        # Si on est a moins de 5m du beacon #
        if norm(vectsub(self.pos, self.beacon)) < 0.0002: 
            # On demande a s'éloigner de 5m de la balise #
            self.goal = vectadd(self.pos, scalar(0.00007 / norm(vectsub(self.pos, self.beacon)), vectsub(self.pos, self.beacon))) 

        norm_enem = norm(vectsub(self.pos, self.enemy))
        print(self.enemy, norm_enem)
        ###self.get_logger().info("\ndist to enem = %s" % norm_enem)
        # Si on est a moins de 30m de l'ennemi #
        if norm_enem < 0.00015:
            print("DEMI TOUR !!!!")
            self.goal = vectadd(self.pos, scalar(0.00007/norm_enem, vectsub(self.pos, self.enemy)))

        # On traduit le msg pour simplicité #
        lst = []
        for i in range(len(msg.gps_list)):
            # On récupère les nouveaux points #
            lst.append((msg.gps_list[i].longitude, msg.gps_list[i].latitude))

        # On fusionne avec les points qu'on avait déjà #
        self.merge_obst(lst)

        # On ajoute les bateaux #
        self.obstacles = self.obstacles + self.allies
        if self.goal == (0.0, 0.0):
            ###self.get_logger().info("\nstop moving")
            # si on arrive ici c'est qu'on ne veut pas bouger #
            waypoint_msg = NavSatFix()
            waypoint_msg.longitude = self.goal[0]
            waypoint_msg.latitude = self.goal[1]
            self.pub_waypoint.publish(waypoint_msg)
            return # on return pour pas faire le reste, qui pourrait demander au bateau de bouger alors qu'il doit rester immobile #
        
        # On regarde si un obstacle est sur le chemin #
        self.waypoint, i = self.get_shortest_dist(self.goal)
        
        # Si il y a un obstacle -> on reregarde avec comme obj le waypoint (au cas ou un obstacle est sur la nouvelle traj) #
        if not self.no_obstacle :
            self.waypoint, i = self.get_shortest_dist(self.waypoint)
            self.no_obstacle = True
        
        # Une fois sortie de la boucle, on a trouvé le waypoint le plus imminent #
        self.no_obstacle = False

        ###self.get_logger().info("\nwaypoint = (%s, %s)\npos      = (%s, %s)" % (self.waypoint[0], self.waypoint[1], self.pos[0], self.pos[1]))


        self.waypoints_x.append(self.waypoint[0])
        self.waypoints_y.append(self.waypoint[1])
        # On publie le waypoint #
        waypoint_msg = NavSatFix()
        waypoint_msg.longitude = self.waypoint[0]
        waypoint_msg.latitude = self.waypoint[1]
        self.pub_waypoint.publish(waypoint_msg)

        


def main(args=None):
    rclpy.init(args=args)
    
    pathfind = PathFinder()
    rclpy.spin(pathfind)

    pathfind.destroy_node()
    rclpy.shutdown()

