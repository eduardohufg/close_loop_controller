#!/usr/bin/env python3
# color_detector_node.py  (con visualización)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        # ── Parámetros dinámicos ─────────────────────────────────
        self.declare_parameter('min_area', 1000)   # píxeles²
        self.declare_parameter('camera_id', 0)
        self.min_area = self.get_parameter('min_area').value
        camera_id      = self.get_parameter('camera_id').value
        self.add_on_set_parameters_callback(self._param_cb)

        # ── Publishers ───────────────────────────────────────────
        qos = 1
        self.pub_red    = self.create_publisher(Bool, '/red',    qos)
        self.pub_green  = self.create_publisher(Bool, '/green',  qos)
        self.pub_yellow = self.create_publisher(Bool, '/yellow', qos)

        # ── Cámara ───────────────────────────────────────────────
        self.cap = cv2.VideoCapture(camera_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'No se pudo abrir la cámara {camera_id}')
            raise RuntimeError('Camera open failed')

        # ── Timer principal ─────────────────────────────────────
        self.timer = self.create_timer(0.1, self.loop)   # 10 Hz
        self.get_logger().info('Color detector con vista iniciada')

    # Permite cambiar el área mínima en caliente
    def _param_cb(self, params):
        for p in params:
            if p.name == 'min_area' and p.type_ == p.Type.INTEGER:
                self.min_area = p.value
                self.get_logger().info(f'Nuevo min_area: {self.min_area}')
        return rclpy.parameter.SetParametersResult(successful=True)

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Frame no disponible')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ── Máscaras HSV ────────────────────────────────────────
        masks = {
            'Red':    self._mask_red(hsv),
            'Green':  cv2.inRange(hsv, (40,  40,  40), ( 85, 255, 255)),
            'Yellow': cv2.inRange(hsv, (25, 50, 50), ( 38, 255, 255)),
        }

        # Resultados para publicación
        detections = {}

        # Procesar cada color
        for label, mask in masks.items():
            detected, frame = self._process_mask(label, mask, frame)
            detections[label] = detected

        # ── Publicar Bools ──────────────────────────────────────
        self.pub_red.publish   (Bool(data=detections['Red']))
        self.pub_green.publish (Bool(data=detections['Green']))
        self.pub_yellow.publish(Bool(data=detections['Yellow']))

        # ── Mostrar ventana ─────────────────────────────────────
        cv2.imshow("Color detector", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # Al pulsar q cerramos correctamente
            rclpy.shutdown()

    # Rojo = dos regiones en HSV
    @staticmethod
    def _mask_red(hsv):
        lower1 = cv2.inRange(hsv, (0,   10, 255), (4.5, 255, 255))
        lower2 = cv2.inRange(hsv, (169.91, 10, 255), (169.91, 255, 255))
        return cv2.bitwise_or(lower1, lower2)

    # Revisa contornos, dibuja y devuelve bool de detección
    def _process_mask(self, label, mask, frame):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= self.min_area:
                detected = True
                # Dibujar contorno y etiqueta
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
                cv2.putText(frame, label, (x, y-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 255), 2, cv2.LINE_AA)
        return detected, frame

def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)
    finally:
        if hasattr(node, 'cap'):
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
