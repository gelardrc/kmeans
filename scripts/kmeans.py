#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sklearn.cluster import KMeans
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point,Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import random as rnd
import sys


class KMeansCoverage:
    def __init__(self,agents):
        rospy.init_node("kmeans_coverage")
        
        # Configurações
        ##self.free_threshold = rospy.get_param("~free_threshold", 0)  # Threshold para células livres
        self.free_threshold = 50  # Threshold para células livres
        ag = [[rnd.randint(0,64),rnd.randint(0,64)] for i in range(agents)] ## precisa ser nos free_cells por enquanto ta aleatorio
        self.robot_positions = np.array(ag)
        print("robot_position {}".format(self.robot_positions))
        self.colors = []
        [self.colors.append([rnd.random(),rnd.random(),rnd.random()]) for i in range(len(self.robot_positions))]
        # Publishers
        self.cell_publishers = [rospy.Publisher("/robot_{}/target_cells".format(i), Point, queue_size=10)
                                for i in range(len(self.robot_positions))]
        self.marker_publishers = [rospy.Publisher("/robot_{}/marker".format(i), Marker, queue_size=10)
                                  for i in range(len(self.robot_positions))]
        
        self.robot_marker = rospy.Publisher("/robots_poses", Marker, queue_size=10)
        
        # Subscriber do mapa
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        
        self.map_received = False
        self.free_cells = None
        rospy.loginfo("Aguardando o mapa no tópico /map...")
    
    def map_callback(self, msg):
        """
        Callback para processar o mapa recebido do tópico /map.
        """
        rospy.loginfo("Mapa recebido. Processando...")
        self.mapa = msg
        map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.free_cells = np.array([[x, y] for y in range(msg.info.height)
                                    for x in range(msg.info.width)
                                    if map_data[y, x] == 0]) ## 0 é sem ocupação!!!!!
        self.map_received = True
        rospy.loginfo("Número de células livres: {}".format(len(self.free_cells)))

    def apply_kmeans(self):
        """
        Aplica o algoritmo K-Means para segmentar as células livres.
        """
        num_clusters = len(self.robot_positions)
        kmeans = KMeans(n_clusters=num_clusters, init=self.robot_positions, n_init=1)
        labels = kmeans.fit_predict(self.free_cells)
        final_centroids = kmeans.cluster_centers_
        rospy.loginfo("Centroides finais: {}".format(final_centroids))
        clustered_cells = {}
        for i in range(num_clusters):
            clustered_cells[i] = self.free_cells[labels == i]
        return clustered_cells, final_centroids

    def publish_cells(self, clustered_cells):
        """
        Publica as células atribuídas a cada robô.
        """
        for robot_id, cells in clustered_cells.items():
            for cell in cells:
                point = Point(x=cell[0], y=cell[1], z=0)
                self.cell_publishers[robot_id].publish(point)
    
    def publish_robots(self):
        for i in range(len(self.robot_positions)):
            marker = Marker()
            marker.header = Header(frame_id="map")
            marker.ns = "robot_{}".format(i)
            marker.id = i  # Defina um ID fixo para o marcador
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = 1.0  # Ajuste a escala para garantir que os pontos sejam visíveis
            marker.scale.y = 1.0  # Ajuste a escala para garantir que os pontos sejam visíveis
            marker.scale.z = 1.0  # Ajuste a escala para garantir que os pontos sejam visíveis

            marker.color.a = 1.0  # Torne o marcador completamente opaco
    
            marker.color.r = self.colors[i][0]
            marker.color.g = self.colors[i][1]
            marker.color.b = self.colors[i][2]
            
            marker.pose.position = Point(x=self.robot_positions[i][0]+self.mapa.info.origin.position.x,
                                         y=self.robot_positions[i][1]+self.mapa.info.origin.position.y,
                                         z=0)

            marker.pose.orientation = Quaternion(x=0, y=0.7071, z=0, w=0.7071)

            self.robot_marker.publish(marker)


    def publish_markers(self, clustered_cells):
        """
        Publica marcadores para visualização no RViz.
        """
        for robot_id, cells in clustered_cells.items():
            #rospy.loginfo("robot id ->{}".format(robot_id))
            marker = Marker()
            marker.header = Header(frame_id="map")
            marker.ns = "robot_{}".format(robot_id)
            marker.id = 0  # Defina um ID fixo para o marcador
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 1.0  # Ajuste a escala para garantir que os pontos sejam visíveis
            marker.scale.y = 1.0  # Ajuste a escala para garantir que os pontos sejam visíveis
            marker.color.a = 1.0  # Torne o marcador completamente opaco
          
            marker.color.r = self.colors[robot_id][0]
            marker.color.g = self.colors[robot_id][1]
            marker.color.b = self.colors[robot_id][2]
          
            for cell in cells:
                x1 = cell[0] + self.mapa.info.origin.position.x
                y1 = cell[1] + self.mapa.info.origin.position.y

                marker.points.append(Point(x=x1, y=y1, z=0))
            self.marker_publishers[robot_id].publish(marker)

    def run(self):
        """
        Loop principal para executar a segmentação e publicar os resultados.
        """
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.map_received and self.free_cells is not None:
                rospy.loginfo("Executando o K-Means...")
                clustered_cells, _ = self.apply_kmeans()
                self.publish_cells(clustered_cells)
                self.publish_markers(clustered_cells)
                self.publish_robots()
            rate.sleep()

if __name__ == "__main__":
    if len(sys.argv) < 2 or len(sys.argv) > 2:
        print("Use: rosrun ghybrid kmeans.py <number_of_agents>")
    else:
        agents = int(sys.argv[1])
	rospy.loginfo("Numero de agentes".format(sys.argv[1]))
    try:
	rospy.loginfo("Numero de agentes".format(sys.argv[1]))
        node = KMeansCoverage(agents)
        node.run()
    except rospy.ROSInterruptException:
        pass

