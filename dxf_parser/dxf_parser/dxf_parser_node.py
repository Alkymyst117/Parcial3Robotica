import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
import ezdxf
import math
from typing import List, Tuple, Optional
import time


class DXFParserNode(Node):
    def __init__(self):
        super().__init__('dxf_parser_node')
        
        self.waypoint_publisher = self.create_publisher(PointStamped, 'scara_waypoints', 1000)
        
        self.arc_resolution = 0.3
        self.line_resolution = 2.0
        self.z_travel = 10.0
        self.z_cut = 0.0
        
        self.waypoints = []
        self.point_count = 0
        self.x_coordinates = []
        self.y_coordinates = []
        
        self.timer = self.create_timer(0.2, self.publish_waypoints)
        self.current_waypoint_index = 0
        
        dxf_file = '/home/alkymyst117/ros2_ws/src/dxf_parser/archivos/SCARA.dxf'
        self.parse_dxf_file(dxf_file)
        
    def publish_waypoints(self):
        if self.current_waypoint_index < len(self.waypoints):
            x, y, z = self.waypoints[self.current_waypoint_index]
            
            msg = PointStamped()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.point.x = float(x)
            msg.point.y = float(y)
            msg.point.z = float(z)
            
            self.waypoint_publisher.publish(msg)
            self.current_waypoint_index += 1
        else:
            if len(self.waypoints) > 0:
                self.current_waypoint_index = 0
        
    def parse_dxf_file(self, dxf_file: str):
        try:
            doc = ezdxf.readfile(dxf_file)
            msp = doc.modelspace()
            
            entities = []
            for entity in msp:
                if entity.dxftype() in ["LWPOLYLINE", "POLYLINE", "LINE", "CIRCLE", "ARC", "POINT"]:
                    entities.append(entity)
            
            lwpolylines = [e for e in entities if e.dxftype() == "LWPOLYLINE"]
            polylines = [e for e in entities if e.dxftype() == "POLYLINE"]
            lines = [e for e in entities if e.dxftype() == "LINE"]
            circles = [e for e in entities if e.dxftype() == "CIRCLE"]
            arcs = [e for e in entities if e.dxftype() == "ARC"]
            points = [e for e in entities if e.dxftype() == "POINT"]
            
            all_entities = []
            all_entities.extend([(e, "LWPOLYLINE") for e in lwpolylines])
            all_entities.extend([(e, "POLYLINE") for e in polylines])
            all_entities.extend([(e, "CIRCLE") for e in circles])
            all_entities.extend([(e, "ARC") for e in arcs])
            
            if len(lwpolylines) == 0:
                all_entities.extend([(e, "LINE") for e in lines])
                
            all_entities.extend([(e, "POINT") for e in points])
            for i, (entity, entity_type) in enumerate(all_entities):
                if i > 0:
                    self.add_transition_point()
                
                if entity_type == "LWPOLYLINE":
                    self.process_lwpolyline(entity)
                elif entity_type == "POLYLINE":
                    self.process_polyline(entity)
                elif entity_type == "LINE":
                    self.process_line(entity)
                elif entity_type == "CIRCLE":
                    self.process_circle(entity)
                elif entity_type == "ARC":
                    self.process_arc(entity)
                elif entity_type == "POINT":
                    self.process_point(entity)
            
            if len(lwpolylines) > 0 and len(lines) > 0:
                pass
                    
        except Exception as e:
            pass
        
        return self.waypoints

    def add_waypoint(self, x: float, y: float, z: float = None):
        if z is None:
            z = self.z_cut
        self.waypoints.append((x, y, z))
        self.x_coordinates.append(round(x, 3))
        self.y_coordinates.append(round(y, 3))
        self.point_count += 1
        if self.point_count % 100 == 0:
            pass
    
    def add_transition_point(self):
        if len(self.waypoints) > 0:
            last_x, last_y, _ = self.waypoints[-1]
            self.add_waypoint(last_x, last_y, self.z_travel)
            
    def start_new_entity(self, x: float, y: float):
        self.add_waypoint(x, y, self.z_travel)
        self.add_waypoint(x, y, self.z_cut)


    def interpolate_line(self, x1: float, y1: float, x2: float, y2: float):
        """Interpola puntos intermedios en una línea recta según line_resolution."""
        # Calcular distancia
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx*dx + dy*dy)
        
        if length < self.line_resolution:
            # Línea muy corta, solo agregar punto final
            return [(x2, y2)]
        
        # Calcular número de segmentos
        segments = max(1, int(length / self.line_resolution))
        points = []
        
        # Generar puntos intermedios
        for i in range(1, segments + 1):
            t = i / segments
            x = x1 + t * dx
            y = y1 + t * dy
            points.append((x, y))
        

        return points



    def process_lwpolyline(self, entity):
        """Procesa una LWPOLYLINE con alta resolución para arcos y líneas rectas precisas."""
        try:
            points = list(entity.get_points("xyb"))
            if not points:
                return
                

            
            # Imprimir todos los puntos para diagnóstico
            for i, point in enumerate(points):
                x, y, bulge = point[0], point[1], point[2]
                self.get_logger().info(f"  Punto {i}: ({x:.3f}, {y:.3f}), bulge: {bulge:.6f}")
            
            # Verificar si es una polilínea cerrada y procesarla correctamente
            if entity.closed and len(points) > 2:
                # Para polilíneas cerradas, procesamos como una trayectoria continua
                self.get_logger().info("Procesando polilínea cerrada como trayectoria continua")
                
                # Separar en trayectorias individuales si es necesario
                # Para evitar líneas extrañas, vamos a procesar solo los segmentos válidos
                
                # Iniciar nueva entidad (aproximación y descenso)
                first_point = points[0]
                self.start_new_entity(first_point[0], first_point[1])
                
                # Procesar cada segmento secuencialmente
                for i in range(len(points)):
                    current_point = points[i]
                    next_point = points[(i + 1) % len(points)]
                    
                    x1, y1, bulge = current_point[0], current_point[1], current_point[2]
                    x2, y2 = next_point[0], next_point[1]
                    
                    # Verificar que no es el mismo punto
                    if abs(x2 - x1) < 1e-10 and abs(y2 - y1) < 1e-10:
                        self.get_logger().debug(f"  Saltando segmento {i+1}: puntos idénticos")
                        continue
                    
                    self.get_logger().info(f"  Segmento {i+1}: ({x1:.3f}, {y1:.3f}) -> ({x2:.3f}, {y2:.3f}), bulge: {bulge:.6f}")
                    
                    if abs(bulge) > 1e-10:  # Hay un arco - usar alta resolución
                        arc_points = self.interpolate_arc_segment(x1, y1, x2, y2, bulge)
                        self.get_logger().info(f"    Arco: generando {len(arc_points)} puntos")
                        for point in arc_points:
                            self.add_waypoint(point[0], point[1], self.z_cut)  # Z=0 durante corte
                    else:  # Línea recta - interpolar puntos
                        line_points = self.interpolate_line(x1, y1, x2, y2)
                        for point in line_points:
                            self.add_waypoint(point[0], point[1], self.z_cut)  # Z=0 durante corte
                        self.get_logger().info(f"    Línea recta interpolada: {len(line_points)} puntos hasta ({x2:.3f}, {y2:.3f})")
            else:
                # Polilínea abierta - procesamiento normal
                self.get_logger().info("Procesando polilínea abierta")
                
                # Iniciar nueva entidad (aproximación y descenso)
                first_point = points[0]
                self.start_new_entity(first_point[0], first_point[1])
                
                # Procesar cada segmento
                num_segments = len(points) - 1
                
                for i in range(num_segments):
                    current_point = points[i]
                    next_point = points[i + 1]
                    
                    x1, y1, bulge = current_point[0], current_point[1], current_point[2]
                    x2, y2 = next_point[0], next_point[1]
                    
                    # Verificar que no es el mismo punto
                    if abs(x2 - x1) < 1e-10 and abs(y2 - y1) < 1e-10:
                        continue
                    
                    self.get_logger().info(f"  Segmento {i+1}: ({x1:.3f}, {y1:.3f}) -> ({x2:.3f}, {y2:.3f}), bulge: {bulge:.6f}")
                    
                    if abs(bulge) > 1e-10:  # Hay un arco - usar alta resolución
                        arc_points = self.interpolate_arc_segment(x1, y1, x2, y2, bulge)
                        self.get_logger().info(f"    Arco: generando {len(arc_points)} puntos")
                        for point in arc_points:
                            self.add_waypoint(point[0], point[1], self.z_cut)  # Z=0 durante corte
                    else:  # Línea recta - interpolar puntos
                        line_points = self.interpolate_line(x1, y1, x2, y2)
                        for point in line_points:
                            self.add_waypoint(point[0], point[1], self.z_cut)  # Z=0 durante corte
                        self.get_logger().info(f"    Línea recta interpolada: {len(line_points)} puntos hasta ({x2:.3f}, {y2:.3f})")
                        
        except Exception as e:
            self.get_logger().error(f"Error procesando LWPOLYLINE: {str(e)}")

    def process_line(self, entity):
        """Procesa una línea recta independiente - solo punto inicial y final."""
        try:
            start = entity.dxf.start
            end = entity.dxf.end
            
            # Verificar que no es un punto degenerado
            if abs(end.x - start.x) < 1e-10 and abs(end.y - start.y) < 1e-10:
                self.get_logger().debug("Saltando línea degenerada (inicio = fin)")
                return
            
            self.get_logger().info(f"LINE desde ({start.x:.3f}, {start.y:.3f}) hasta ({end.x:.3f}, {end.y:.3f})")
            
            # Iniciar nueva entidad y procesarla con interpolación
            self.start_new_entity(start.x, start.y)
            
            # Interpolar puntos en la línea recta
            line_points = self.interpolate_line(start.x, start.y, end.x, end.y)
            for point in line_points:
                self.add_waypoint(point[0], point[1], self.z_cut)  # Z=0 durante corte
                
            self.get_logger().info(f"  Línea con {len(line_points)} puntos interpolados")
                
        except Exception as e:
            self.get_logger().error(f"Error procesando LINE: {str(e)}")

    def process_circle(self, entity):
        """Procesa un círculo con alta resolución."""
        try:
            cx, cy = entity.dxf.center.x, entity.dxf.center.y
            r = entity.dxf.radius
            
            self.get_logger().info(f"CIRCLE centro ({cx:.3f}, {cy:.3f}), radio {r:.3f}")
            
            # Calcular número de segmentos
            circumference = 2 * math.pi * r
            steps = max(8, int(circumference / self.arc_resolution))
            
            self.get_logger().debug(f"  Generando {steps} puntos de círculo")
            
            # Iniciar nueva entidad en el primer punto del círculo
            start_x = cx + r
            start_y = cy
            self.start_new_entity(start_x, start_y)
            
            # Generar puntos del círculo (empezando desde el segundo punto)
            for i in range(1, steps):  
                angle = 2 * math.pi * i / steps
                x = cx + r * math.cos(angle)
                y = cy + r * math.sin(angle)
                self.add_waypoint(x, y, self.z_cut)  # Z=0 durante corte
                
            # Cerrar el círculo volviendo al punto inicial
            self.add_waypoint(start_x, start_y, self.z_cut)
                
        except Exception as e:
            self.get_logger().error(f"Error procesando CIRCLE: {str(e)}")

    def process_arc(self, entity):
        """Procesa un arco con alta resolución."""
        try:
            cx, cy = entity.dxf.center.x, entity.dxf.center.y
            r = entity.dxf.radius
            start_angle = math.radians(entity.dxf.start_angle)
            end_angle = math.radians(entity.dxf.end_angle)
            
            # Normalizar ángulos
            if end_angle < start_angle:
                end_angle += 2 * math.pi
                
            angle_span = end_angle - start_angle
            arc_length = r * angle_span
            steps = max(3, int(arc_length / self.arc_resolution))
            
            self.get_logger().info(f"ARC centro ({cx:.3f}, {cy:.3f}), radio {r:.3f}")
            self.get_logger().debug(f"  Ángulos {math.degrees(start_angle):.1f}° - {math.degrees(end_angle):.1f}°")
            self.get_logger().debug(f"  Generando {steps} puntos de arco")
            
            # Iniciar nueva entidad en el primer punto del arco
            start_x = cx + r * math.cos(start_angle)
            start_y = cy + r * math.sin(start_angle)
            self.start_new_entity(start_x, start_y)
            
            # Generar puntos del arco
            for i in range(1, steps + 1):  # Empezar desde 1 porque el primer punto ya fue agregado
                angle = start_angle + angle_span * i / steps
                x = cx + r * math.cos(angle)
                y = cy + r * math.sin(angle)
                self.add_waypoint(x, y, self.z_cut)  # Z=0 durante corte
                
        except Exception as e:
            self.get_logger().error(f"Error procesando ARC: {str(e)}")

    def process_point(self, entity):
        """Procesa un punto individual."""
        try:
            point = entity.dxf.location
            self.get_logger().info(f"POINT en ({point.x:.3f}, {point.y:.3f})")
            # Para puntos individuales, usar aproximación y tocar
            self.start_new_entity(point.x, point.y)
        except Exception as e:
            self.get_logger().error(f"Error procesando POINT: {str(e)}")



    def interpolate_arc_segment(self, x1: float, y1: float, x2: float, y2: float, bulge: float) -> List[Tuple[float, float]]:
        """Interpola un segmento con bulge (arco) usando la fórmula correcta DXF."""
        if abs(bulge) < 1e-10:
            return [(x2, y2)]  # Si no hay bulge, solo retorna el punto final
        
        # Calcular parámetros del arco según especificación DXF
        chord_length = math.hypot(x2 - x1, y2 - y1)
        if chord_length < 1e-10:
            return []
        
        # El bulge es la tangente de 1/4 del ángulo incluido
        # Radio = chord_length / (2 * sin(atan(abs(bulge))))
        angle_quarter = math.atan(abs(bulge))
        angle_included = 4 * angle_quarter
        
        # Calcular radio usando la fórmula correcta
        if abs(bulge) > 1e-10:
            radius = chord_length / (2 * math.sin(angle_quarter))
        else:
            return [(x2, y2)]
        
        # Calcular el centro del arco
        # Punto medio de la cuerda
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        
        # Distancia del centro al punto medio de la cuerda
        # h = radius * cos(angle_quarter)
        h = math.sqrt(radius * radius - (chord_length / 2) * (chord_length / 2))
        
        # Vector unitario perpendicular a la cuerda
        chord_dx = x2 - x1
        chord_dy = y2 - y1
        perp_x = -chord_dy / chord_length  # Vector perpendicular normalizado
        perp_y = chord_dx / chord_length
        
        # Centro del arco (el signo del bulge determina la dirección)
        if bulge > 0:  # Arco en sentido antihorario
            center_x = mid_x + h * perp_x
            center_y = mid_y + h * perp_y
        else:  # Arco en sentido horario
            center_x = mid_x - h * perp_x
            center_y = mid_y - h * perp_y
        
        # Calcular ángulos de inicio y fin
        start_angle = math.atan2(y1 - center_y, x1 - center_x)
        end_angle = math.atan2(y2 - center_y, x2 - center_x)
        
        # Ajustar ángulos según la dirección del arco
        if bulge > 0:  # Antihorario
            while end_angle <= start_angle:
                end_angle += 2 * math.pi
            sweep_angle = end_angle - start_angle
        else:  # Horario
            while end_angle >= start_angle:
                end_angle -= 2 * math.pi
            sweep_angle = start_angle - end_angle
        
        # Calcular número de segmentos basado en la resolución
        arc_length = radius * abs(sweep_angle)
        num_segments = max(2, int(arc_length / self.arc_resolution))
        
        # Generar puntos del arco
        points = []
        for i in range(1, num_segments + 1):
            t = i / num_segments
            
            if bulge > 0:  # Antihorario
                current_angle = start_angle + sweep_angle * t
            else:  # Horario
                current_angle = start_angle - sweep_angle * t
            
            x = center_x + radius * math.cos(current_angle)
            y = center_y + radius * math.sin(current_angle)
            points.append((x, y))
        
        return points

    def process_polyline(self, entity):
        """Procesa una POLYLINE tradicional."""
        try:
            vertices = list(entity.vertices)
            if not vertices:
                return
                
            self.get_logger().info(f"POLYLINE con {len(vertices)} vértices")
            
            # Primer punto
            first_vertex = vertices[0]
            self.add_waypoint(first_vertex.dxf.location.x, first_vertex.dxf.location.y)
            
            # Resto de puntos - solo vértices para POLYLINE
            for i in range(1, len(vertices)):
                curr_vertex = vertices[i]
                # Solo agregar el vértice actual (líneas rectas entre vértices)
                self.add_waypoint(curr_vertex.dxf.location.x, curr_vertex.dxf.location.y)
                self.get_logger().debug(f"    Agregado vértice {i+1}")
                
        except Exception as e:
            self.get_logger().error(f"Error procesando POLYLINE: {str(e)}")


def main(args=None):
    """Función principal para ejecutar el nodo ROS."""
    rclpy.init(args=args)
    
    try:
        node = DXFParserNode()
        node.get_logger().info("Nodo DXF Parser iniciado")
        node.get_logger().info(f"Publicando {len(node.waypoints)} waypoints en el tópico 'scara_waypoints'")
        
        # Mantener el nodo activo
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass  # Error silencioso
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()