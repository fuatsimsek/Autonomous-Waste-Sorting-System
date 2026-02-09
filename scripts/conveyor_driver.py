#!/usr/bin/env python3
"""
Conveyor Belt Driver
Bandın durumunu kontrol eder ve yayınlar
"""

import rospy
from std_msgs.msg import Float32, Bool


class ConveyorDriver:
    def __init__(self):
        rospy.init_node('conveyor_driver', anonymous=True)
        
        # Parametreler
        self.speed = rospy.get_param('~speed', 0.6)
        self.enabled = True
        
        # Publishers
        self.speed_pub = rospy.Publisher('/conveyor/speed', Float32, queue_size=10)
        self.status_pub = rospy.Publisher('/conveyor/status', Bool, queue_size=10)
        
        # Subscribers (dashboard'dan gelen komutlar)
        rospy.Subscriber('/conveyor/set_speed', Float32, self.set_speed_callback)
        rospy.Subscriber('/conveyor/enable', Bool, self.enable_callback)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("🔧 CONVEYOR DRIVER BAŞLATILDI")
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Başlangıç hızı: {self.speed} m/s")
        rospy.loginfo(f"Durum: {'AKTIF' if self.enabled else 'DURDURULDU'}")
        rospy.loginfo("=" * 60)

    def set_speed_callback(self, msg):
        """Hız değişikliği komutu"""
        old_speed = self.speed
        self.speed = max(0.0, min(2.0, msg.data))  # 0-2 m/s arası
        rospy.loginfo(f"⚡ Hız değişti: {old_speed:.2f} → {self.speed:.2f} m/s")

    def enable_callback(self, msg):
        """Banda başlat/durdur komutu"""
        self.enabled = msg.data
        status = "BAŞLATILDI" if self.enabled else "DURDURULDU"
        rospy.loginfo(f"🔄 Bant {status}")

    def run(self):
        """Ana döngü - durumu yayınla"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Mevcut durumu yayınla
            speed_msg = Float32()
            speed_msg.data = self.speed if self.enabled else 0.0
            self.speed_pub.publish(speed_msg)
            
            status_msg = Bool()
            status_msg.data = self.enabled
            self.status_pub.publish(status_msg)
            
            rate.sleep()


if __name__ == '__main__':
    try:
        driver = ConveyorDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass