"""Keşif düğümü — 4c: en iyi frontier'ı seç, Nav2 ile oraya git."""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from oda_kesif import sinirlar


class Kesif(Node):
    def __init__(self):
        super().__init__('kesif')
        self.grid = None
        self.bilgi = None
        harita_qos = QoSProfile(depth=1,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, 'map', self._harita_cb, harita_qos)
        self.tf_buffer = Buffer()
        self.tf_dinleyici = TransformListener(self.tf_buffer, self)

    def _harita_cb(self, msg):
        self.bilgi = msg.info
        self.grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)

    def robot_konumu(self):
        """Robotun haritadaki (x, y) konumu, metre. Henüz bilinmiyorsa None."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def hucre_to_dunya(self, satir, sutun):
        """Hücre (satır, sütun) → harita koordinatı (x, y), metre."""
        o = self.bilgi.origin.position
        res = self.bilgi.resolution
        return o.x + (sutun + 0.5) * res, o.y + (satir + 0.5) * res

    def en_iyi_sinir(self, min_mesafe=0.5):
        """Robota en yakın frontier (x, y). Yoksa ya da henüz bilinmiyorsa None."""
        konum = self.robot_konumu()
        if self.grid is None or konum is None:
            return None
        rx, ry = konum
        adaylar = []
        for (satir, sutun), boyut in sinirlar.sinirlari_bul(self.grid, self.bilgi.resolution):
            x, y = self.hucre_to_dunya(satir, sutun)
            mesafe = math.hypot(x - rx, y - ry)
            if mesafe < min_mesafe:
                continue   # robotun kendi gölgesi, oraya gitmeye gerek yok
            adaylar.append((mesafe, x, y))
        if not adaylar:
            return None
        _, x, y = min(adaylar)
        return x, y


def poz(nav, x, y, yaw):
    """(x, y, yaw) → Nav2'nin istediği PoseStamped mesajı."""
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def main():
    rclpy.init()
    kesif = Kesif()
    log = kesif.get_logger()
    nav = BasicNavigator()

    log.info('Nav2 bekleniyor...')
    nav.waitUntilNav2Active(localizer='robot_localization')

    log.info('Harita, robot konumu ve frontier bekleniyor...')
    hedef = None
    while rclpy.ok() and hedef is None:
        rclpy.spin_once(kesif, timeout_sec=0.5)
        hedef = kesif.en_iyi_sinir()

    rx, ry = kesif.robot_konumu()
    yaw = math.atan2(hedef[1] - ry, hedef[0] - rx)
    log.info(f'Hedef frontier: ({hedef[0]:.2f}, {hedef[1]:.2f}) m — Nav2\'ye gönderiliyor')
    nav.goToPose(poz(nav, hedef[0], hedef[1], yaw))

    while not nav.isTaskComplete():
        rclpy.spin_once(kesif, timeout_sec=0.1)
        fb = nav.getFeedback()
        if fb:
            log.info(f'Kalan mesafe: {fb.distance_remaining:.2f} m',
                     throttle_duration_sec=1.0)

    sonuc = nav.getResult()
    if sonuc == TaskResult.SUCCEEDED:
        log.info('Frontier\'a varıldı!')
    elif sonuc == TaskResult.CANCELED:
        log.warn('Görev iptal edildi.')
    else:
        log.error('Frontier\'a gidilemedi.')

    kesif.destroy_node()
    rclpy.shutdown()