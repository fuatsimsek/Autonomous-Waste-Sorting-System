#!/usr/bin/env python3
"""
Smart Sorter Controller - Banda yakın kapılar, süpürme hareketiyle kutuları iterler
"""

import rospy
from std_msgs.msg import String, Float64
import json
import time


class SmartSorterController:
    def __init__(self):
        rospy.init_node('sorter_controller', anonymous=True)
        
        # ═══════════════════════════════════════════════════════════════════
        #  KAPI BİLGİLERİ (v2.0 - STANDARTLAŞTIRILMIŞ)
        # ═══════════════════════════════════════════════════════════════════
        # Menteşe noktaları: y = ±0.27m (yan duvar boşluklarının dış kenarı)
        # Tüm kapılar: 0.06m × 0.55m × 0.24m (standart boyut)
        # ═══════════════════════════════════════════════════════════════════
        self.gate_info = {
            1: {'x': -3.0, 'side': 'left',  'category': 'metal',   'pivot_y': 0.27},
            2: {'x': -1.8, 'side': 'right', 'category': 'plastic', 'pivot_y': -0.27},
            3: {'x': -0.6, 'side': 'left',  'category': 'glass',   'pivot_y': 0.27},
            4: {'x': 0.6,  'side': 'right', 'category': 'paper',   'pivot_y': -0.27},
            5: {'x': 1.8,  'side': 'left',  'category': 'battery', 'pivot_y': 0.27},
            6: {'x': 3.0,  'side': 'right', 'category': 'organic', 'pivot_y': -0.27}
        }
        
        # Kategori → kapı mapping
        self.category_gates = {
            'metal': 1,
            'plastic': 2,
            'glass': 3,
            'paper': 4,
            'battery': 5,
            'organic': 6
        }
        
        # Gate publishers
        self.gate_pubs = {}
        for i in range(1, 7):
            topic = f'/gate{i}_controller/command'
            self.gate_pubs[i] = rospy.Publisher(topic, Float64, queue_size=10)
        
        # Subscribe to detections
        self.detection_sub = rospy.Subscriber('/object_detection', String, 
                                              self.detection_callback)
        
        # ═══════════════════════════════════════════════════════════════════
        #  KAPI AÇILARI (v2.0 - STANDARTLAŞTIRILMIŞ)
        # ═══════════════════════════════════════════════════════════════════
        # LEFT kapılar (y=+0.27): Banda doğru (POZİTİF açı)  → limitlerle uyumlu
        #   - DEFAULT (KAPALI): 0.0 rad → Boşluğu kapatır
        #   - OPEN (AÇIK): +1.15 rad (~66°) → Bandın ortasına süpürür
        #
        # RIGHT kapılar (y=-0.27): Banda doğru (NEGATİF açı) → limitlerle uyumlu
        #   - DEFAULT (KAPALI): 0.0 rad → Boşluğu kapatır
        #   - OPEN (AÇIK): -1.15 rad (~66°) → Bandın ortasına süpürür
        # ═══════════════════════════════════════════════════════════════════
        
        self.GATE_OPEN_ANGLE = 1.15  # rad (~66°) - STANDART
        self.GATE_CLOSED = 0.0       # rad - DEFAULT pozisyon
        
        self.gate_angles = {
            'left': {
                'closed': self.GATE_CLOSED,
                'open': +self.GATE_OPEN_ANGLE
            },
            'right': {
                'closed': self.GATE_CLOSED,
                'open': -self.GATE_OPEN_ANGLE
            }
        }
        
        # Süpürme parametreleri
        self.gate_push_duration = 1.6  # saniye - kapı açık kalma süresi (genel)
        self.gate_push_duration_override = {1: 1.0}  # Kapı1 (metal) daha kısa açık kalsın
        
        # Tracking
        self.processed_objects = set()
        self.gate_timers = {}
        
        # Belt parameters
        self.belt_speed = rospy.get_param('~belt_speed', 0.6)
        self.camera_x = -4.0
        
        # Initialize gates
        self.close_all_gates()
        
        rospy.loginfo("=" * 80)
        rospy.loginfo("🤖 SMART SORTER CONTROLLER INITIALIZED - BANDA YAKIN KAPILAR")
        rospy.loginfo("=" * 80)
        rospy.loginfo("Kapı Konfigürasyonu:")
        for gate_id, info in self.gate_info.items():
            side_str = f"{info['side']:5s} (y={info['pivot_y']:+.2f}m)"
            rospy.loginfo(
                f"  Kapı {gate_id}: x={info['x']:+5.1f}m | "
                f"Taraf: {side_str} | "
                f"Kategori: {info['category'].upper()}"
            )
        rospy.loginfo(f"Bant hızı: {self.belt_speed} m/s")
        rospy.loginfo(f"Kamera pozisyonu: x={self.camera_x}m")
        rospy.loginfo(f"Süpürme süresi: {self.gate_push_duration}s")
        rospy.loginfo(f"Açılma açıları: LEFT={+self.GATE_OPEN_ANGLE:.2f} rad, RIGHT={-self.GATE_OPEN_ANGLE:.2f} rad (~66°)")
        rospy.loginfo(f"Kapı boyutları: 0.06m × 0.55m × 0.24m (standart)")
        rospy.loginfo("=" * 80)

    def close_all_gates(self):
        """Tüm kapıları kapalı pozisyona getir"""
        rospy.sleep(0.5)
        for gate_id in range(1, 7):
            self.set_gate_closed(gate_id)
        rospy.sleep(0.5)
        rospy.loginfo("✓ Tüm kapılar KAPALI pozisyonda (açıklıkları kapatıyor)")

    def detection_callback(self, msg):
        """Algılanan nesneleri işle"""
        try:
            detections = json.loads(msg.data)
        except json.JSONDecodeError:
            rospy.logerr("Detection verisi parse edilemedi")
            return
        
        for detection in detections:
            category = detection.get('category')
            if category not in self.category_gates:
                rospy.logwarn(f"Bilinmeyen kategori: {category}")
                continue
            
            # Nesne pozisyonu
            x_pos = detection.get('x', 0)
            y_pos = detection.get('y', 0)
            
            # Benzersiz ID
            obj_id = f"{category}_{x_pos}_{y_pos}"
            
            # Daha önce işlendiyse atla
            if obj_id in self.processed_objects:
                continue
            
            self.processed_objects.add(obj_id)
            
            # İlgili kapıyı bul
            gate_id = self.category_gates[category]
            gate_x = self.gate_info[gate_id]['x']
            gate_side = self.gate_info[gate_id]['side']
            gate_pivot_y = self.gate_info[gate_id]['pivot_y']
            
            # Zamanlama hesapla
            distance = gate_x - self.camera_x
            if distance <= 0:
                continue
            
            travel_time = distance / self.belt_speed
            
            # Kapıyı kutu varmadan hemen önce aç
            if gate_id == 1:
                trigger_delay = 0.05  # Metal: kameradan hemen sonra
            else:
                trigger_delay = max(0.1, travel_time - 0.5)
            
            rospy.loginfo(
                f"🎯 {category.upper():8s} algılandı → Kapı {gate_id} "
                f"({gate_side:5s}, y={gate_pivot_y:+.2f}m) | "
                f"Mesafe: {distance:.2f}m | "
                f"Açılma zamanı: {trigger_delay:.2f}s sonra"
            )
            
            # Kapı tetikleyicisini zamanla
            timer = rospy.Timer(
                rospy.Duration(trigger_delay),
                lambda event, gid=gate_id, cat=category: self.trigger_gate(gid, cat),
                oneshot=True
            )
        
        # Bellek temizliği
        if len(self.processed_objects) > 100:
            self.processed_objects.clear()

    def trigger_gate(self, gate_id, category):
        """Kapıyı aç (banda doğru süpür) ve sonra kapat"""
        gate_side = self.gate_info[gate_id]['side']
        gate_pivot_y = self.gate_info[gate_id]['pivot_y']
        
        # Açı bilgisi
        angle_open = self.gate_angles[gate_side]['open']
        angle_closed = self.gate_angles[gate_side]['closed']
        
        rospy.loginfo(
            f"🚪 Kapı {gate_id} AÇILIYOR: {category.upper()} süpürülüyor | "
            f"Taraf: {gate_side} (y={gate_pivot_y:+.2f}m) | "
            f"Açı: {angle_closed:.1f}→{angle_open:.1f} rad"
        )
        
        # Kapıyı aç (banda doğru süpür)
        self.set_gate_open(gate_id)
        
        # Mevcut zamanlayıcıyı iptal et
        if gate_id in self.gate_timers and self.gate_timers[gate_id]:
            self.gate_timers[gate_id].shutdown()
        
        # Kapıyı kapatma zamanlayıcısı
        def close_callback(event):
            self.set_gate_closed(gate_id)
            rospy.loginfo(
                f"🚪 Kapı {gate_id} KAPANDI "
                f"(açıklığı tekrar kapatıyor)"
            )
        
        duration = self.gate_push_duration_override.get(gate_id, self.gate_push_duration)
        self.gate_timers[gate_id] = rospy.Timer(
            rospy.Duration(duration),
            close_callback,
            oneshot=True
        )

    def set_gate_open(self, gate_id):
        """Kapıyı aç (banda doğru süpür)"""
        gate_side = self.gate_info[gate_id]['side']
        angle = self.gate_angles[gate_side]['open']
        
        if gate_id in self.gate_pubs:
            msg = Float64()
            msg.data = angle
            self.gate_pubs[gate_id].publish(msg)
            rospy.sleep(0.05)  # Komutu garantilemek için kısa bekleme

    def set_gate_closed(self, gate_id):
        """Kapıyı kapat (açıklığı kapat)"""
        gate_side = self.gate_info[gate_id]['side']
        angle = self.gate_angles[gate_side]['closed']
        
        if gate_id in self.gate_pubs:
            msg = Float64()
            msg.data = angle
            self.gate_pubs[gate_id].publish(msg)
            rospy.sleep(0.05)  # Komutu garantilemek için kısa bekleme

    def run(self):
        """Ana döngü"""
        rate = rospy.Rate(10)
        
        rospy.loginfo("✓ Controller çalışıyor - Tespit bekleniyor...")
        
        # Periyodik durum güncellemesi
        last_status_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # Her 30 saniyede bir durum raporu
            if (rospy.Time.now() - last_status_time).to_sec() > 30:
                rospy.loginfo(
                    f"📊 Durum: {len(self.processed_objects)} nesne işlendi"
                )
                last_status_time = rospy.Time.now()
            
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = SmartSorterController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
