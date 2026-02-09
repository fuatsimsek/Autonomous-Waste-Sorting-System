#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
═══════════════════════════════════════════════════════════════════════════
KAPI TEST ARACI (Gate Test Tool)
═══════════════════════════════════════════════════════════════════════════

Tüm kapıları test eder ve doğru çalışıp çalışmadığını kontrol eder.

TEST MODLARI:
  1. Sıralı Test: Her kapı tek tek açılır/kapanır
  2. Aynı Anda: Tüm kapılar birlikte
  3. Karşılaştırmalı: LEFT vs RIGHT
  4. İnteraktif: Manuel kontrol

KULLANIM:
  rosrun pil_ayiklama_sim test_gates.py

═══════════════════════════════════════════════════════════════════════════
"""

import rospy

# Python3 uyumluluğu: raw_input -> input
try:
    raw_input  # type: ignore[name-defined]
except NameError:  # pragma: no cover
    raw_input = input
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import sys
import time

class GateTester:
    """Kapı test sınıfı"""
    
    def __init__(self):
        rospy.init_node('gate_tester', anonymous=True)
        self.gates_locked = False  # Kilitliyken açma komutları yok sayılır
        self.lock_guard_timer = None
        
        # Kapı parametreleri (v2.0 spesifikasyonu)
        self.GATE_OPEN_ANGLE = 1.15   # ±66°
        self.GATE_CLOSED = 0.0
        
        # Kapı bilgileri
        self.gates = {
            # LEFT kapılar pozitif açıyla açılır (URDF limit: 0 → +1.35 rad)
            1: {'side': 'LEFT',  'x': -3.0, 'open': +self.GATE_OPEN_ANGLE},
            3: {'side': 'LEFT',  'x': -0.6, 'open': +self.GATE_OPEN_ANGLE},
            5: {'side': 'LEFT',  'x': +1.8, 'open': +self.GATE_OPEN_ANGLE},
            # RIGHT kapılar negatif açıyla açılır (URDF limit: -1.35 → 0 rad)
            2: {'side': 'RIGHT', 'x': -1.8, 'open': -self.GATE_OPEN_ANGLE},
            4: {'side': 'RIGHT', 'x': +0.6, 'open': -self.GATE_OPEN_ANGLE},
            6: {'side': 'RIGHT', 'x': +3.0, 'open': -self.GATE_OPEN_ANGLE},
        }
        
        # Publisher'ları oluştur
        self.close_msg = Float64(self.GATE_CLOSED)
        self.publishers = {}
        for idx in range(1, 7):
            topic = '/gate{}_controller/command'.format(idx)
            self.publishers[idx] = rospy.Publisher(topic, Float64, queue_size=10)

        # Bant kontrolü
        self.belt_enable_pub = rospy.Publisher('/conveyor/enable', Bool, queue_size=1)
        
        rospy.sleep(0.5)  # Publisher'ların hazır olması için bekle
        
        rospy.loginfo("═" * 70)
        rospy.loginfo("KAPI TEST ARACI v2.0")
        rospy.loginfo("═" * 70)
        rospy.loginfo("✓ 6 kapı kontrolörü hazır")
        rospy.loginfo("✓ Açılma açısı: ±%.2f rad (±66°)" % self.GATE_OPEN_ANGLE)
        rospy.loginfo("✓ Kapalı pozisyon: 0.0 rad")
        rospy.loginfo("═" * 70)
    
    def close_all_gates(self):
        """Tüm kapıları kapat"""
        rospy.loginfo("▶ Tüm kapılar kapatılıyor...")
        self.publish_all_closed()
        rospy.sleep(1.0)
        rospy.loginfo("✓ Tüm kapılar KAPALI pozisyonda")
    
    def open_gate(self, gate_idx):
        """Belirli bir kapıyı aç"""
        angle = self.gates[gate_idx]['open']
        self.publishers[gate_idx].publish(Float64(angle))
    
    def close_gate(self, gate_idx):
        """Belirli bir kapıyı kapat"""
        self.publishers[gate_idx].publish(Float64(self.GATE_CLOSED))

    def publish_all_closed(self):
        """Tüm kapılara anında kapalı komutu gönder (beklemesiz)"""
        for idx in range(1, 7):
            self.publishers[idx].publish(self.close_msg)

    def set_lock_guard(self, enabled):
        """Kilit modunda periyodik kapatma göndererek diğer komutları bastır"""
        if enabled:
            if self.lock_guard_timer:
                self.lock_guard_timer.shutdown()
            self.lock_guard_timer = rospy.Timer(
                rospy.Duration(0.005),  # 200 Hz - diğer komutları bastırmak için
                lambda event: self.publish_all_closed()
            )
            self.gates_locked = True
            rospy.loginfo("▶ Kapılar KİLİT modunda (otomatik kapalı tutma aktif).")
        else:
            if self.lock_guard_timer:
                self.lock_guard_timer.shutdown()
                self.lock_guard_timer = None
            self.gates_locked = False
            rospy.loginfo("▶ Kapı kilidi KALDIRILDI (otomatik kapalı tutma kapatıldı).")
    
    def test_sequential(self):
        """Test 1: Her kapıyı sırayla test et"""
        rospy.loginfo("\n" + "═" * 70)
        rospy.loginfo("TEST 1: SIRAYLA TEST")
        rospy.loginfo("═" * 70)
        
        self.close_all_gates()
        rospy.sleep(1.0)
        
        for idx in range(1, 7):
            info = self.gates[idx]
            rospy.loginfo("\n▶ Kapı %d (%s, x=%.1fm) test ediliyor..." % 
                         (idx, info['side'], info['x']))
            
            # Aç
            rospy.loginfo("  ↳ Açılıyor: %.2f rad" % info['open'])
            self.open_gate(idx)
            rospy.sleep(1.5)
            
            # Kapat
            rospy.loginfo("  ↳ Kapatılıyor: 0.0 rad")
            self.close_gate(idx)
            rospy.sleep(1.0)
            
            rospy.loginfo("✓ Kapı %d tamamlandı" % idx)
        
        rospy.loginfo("\n" + "═" * 70)
        rospy.loginfo("✓ SIRAYLA TEST TAMAMLANDI")
        rospy.loginfo("═" * 70)
    
    def test_simultaneous(self):
        """Test 2: Tüm kapıları aynı anda test et"""
        rospy.loginfo("\n" + "═" * 70)
        rospy.loginfo("TEST 2: AYNI ANDA TEST")
        rospy.loginfo("═" * 70)
        
        self.close_all_gates()
        rospy.sleep(1.0)
        
        rospy.loginfo("\n▶ Tüm kapılar açılıyor...")
        for idx in range(1, 7):
            self.open_gate(idx)
        rospy.sleep(2.0)
        rospy.loginfo("✓ Tüm kapılar AÇIK")
        
        rospy.loginfo("\n▶ Tüm kapılar kapatılıyor...")
        self.close_all_gates()
        rospy.loginfo("✓ Tüm kapılar KAPALI")
        
        rospy.loginfo("\n" + "═" * 70)
        rospy.loginfo("✓ AYNI ANDA TEST TAMAMLANDI")
        rospy.loginfo("═" * 70)
    
    def test_sides(self):
        """Test 3: Sol ve sağ kapıları karşılaştırmalı test et"""
        rospy.loginfo("\n" + "═" * 70)
        rospy.loginfo("TEST 3: SAĞ/SOL KARŞILAŞTIRMALI TEST")
        rospy.loginfo("═" * 70)
        
        self.close_all_gates()
        rospy.sleep(1.0)
        
        # Sol kapıları test et
        rospy.loginfo("\n▶ SOL KAPILAR (1, 3, 5) açılıyor...")
        rospy.loginfo("  Açılma yönü: NEGATİF (-1.15 rad, banda doğru)")
        for idx in [1, 3, 5]:
            self.open_gate(idx)
        rospy.sleep(2.0)
        rospy.loginfo("✓ Sol kapılar açık (gözle kontrol: banda doğru dönmeli)")
        
        rospy.loginfo("\n▶ Sol kapılar kapatılıyor...")
        for idx in [1, 3, 5]:
            self.close_gate(idx)
        rospy.sleep(1.5)
        
        # Sağ kapıları test et
        rospy.loginfo("\n▶ SAĞ KAPILAR (2, 4, 6) açılıyor...")
        rospy.loginfo("  Açılma yönü: POZİTİF (+1.15 rad, banda doğru)")
        for idx in [2, 4, 6]:
            self.open_gate(idx)
        rospy.sleep(2.0)
        rospy.loginfo("✓ Sağ kapılar açık (gözle kontrol: banda doğru dönmeli)")
        
        rospy.loginfo("\n▶ Sağ kapılar kapatılıyor...")
        for idx in [2, 4, 6]:
            self.close_gate(idx)
        rospy.sleep(1.0)
        
        rospy.loginfo("\n" + "═" * 70)
        rospy.loginfo("✓ SAĞ/SOL TEST TAMAMLANDI")
        rospy.loginfo("═" * 70)
    
    def test_interactive(self):
        """Test 4: İnteraktif test"""
        rospy.loginfo("\n" + "═" * 70)
        rospy.loginfo("TEST 4: İNTERAKTİF TEST")
        rospy.loginfo("═" * 70)
        rospy.loginfo("Komutlar:")
        rospy.loginfo("  o<N>  : Kapı N'yi aç (örn: o1)")
        rospy.loginfo("  c<N>  : Kapı N'yi kapat (örn: c1)")
        rospy.loginfo("  oa    : Tüm kapıları aç")
        rospy.loginfo("  cl    : Tüm kapıları kapat (kilitlenmez)")
        rospy.loginfo("  lk    : Kapıları kapat VE kilitle (aç komutlarını reddeder)")
        rospy.loginfo("  ul    : Kilidi kaldır, kapıları kapalıya getir")
        rospy.loginfo("  bs    : Bandı durdur (enable=false)")
        rospy.loginfo("  bb    : Bandı normal başlat (enable=true)")
        rospy.loginfo("  sa    : Acil stop (bant dur, kapılar kapat+kilit)")
        rospy.loginfo("  durum : Kilit durumunu yazdır")
        rospy.loginfo("  q     : Çıkış")
        rospy.loginfo("═" * 70)
        
        self.close_all_gates()
        
        while not rospy.is_shutdown():
            try:
                cmd = raw_input("\nKomut: ").strip().lower()
                
                if cmd == 'q':
                    rospy.loginfo("▶ Çıkılıyor...")
                    if self.lock_guard_timer:
                        self.lock_guard_timer.shutdown()
                    self.close_all_gates()
                    break
                
                elif cmd == 'oa':
                    if self.gates_locked:
                        rospy.logwarn("✗ Kapılar KİLİTLİ. Önce 'ul' komutunu kullan.")
                    else:
                        rospy.loginfo("▶ Tüm kapılar açılıyor...")
                        for idx in range(1, 7):
                            self.open_gate(idx)
                
                elif cmd == 'cl':
                    self.close_all_gates()

                elif cmd == 'lk':
                    self.close_all_gates()
                    self.set_lock_guard(True)

                elif cmd == 'ul':
                    self.close_all_gates()
                    self.set_lock_guard(False)

                elif cmd == 'bs':
                    rospy.loginfo("▶ Bant DURDURULUYOR...")
                    self.belt_enable_pub.publish(Bool(False))
                    rospy.sleep(0.2)

                elif cmd == 'bb':
                    rospy.loginfo("▶ Bant BAŞLATILIYOR...")
                    self.belt_enable_pub.publish(Bool(True))
                    rospy.sleep(0.2)

                elif cmd == 'sa':
                    rospy.loginfo("▶ ACİL STOP / ETKİYİ KALDIR: Band durdur, tüm kapıları kapat ve KİLİTLE")
                    self.belt_enable_pub.publish(Bool(False))
                    self.close_all_gates()
                    self.set_lock_guard(True)

                elif cmd == 'durum':
                    rospy.loginfo("▶ DURUM → Kapı kilidi: %s | Otomatik kapalı tutma: %s" %
                                  ("KİLİTLİ" if self.gates_locked else "AÇIK",
                                   "AKTİF" if self.lock_guard_timer else "PASİF"))
                
                elif cmd.startswith('o') and len(cmd) == 2:
                    idx = int(cmd[1])
                    if 1 <= idx <= 6:
                        if self.gates_locked:
                            rospy.logwarn("✗ Kapılar kilitli. Önce 'ul' komutu ver.")
                        else:
                            rospy.loginfo("▶ Kapı %d açılıyor..." % idx)
                            self.open_gate(idx)
                    else:
                        rospy.logwarn("✗ Geçersiz kapı numarası: %d" % idx)
                
                elif cmd.startswith('c') and len(cmd) == 2:
                    idx = int(cmd[1])
                    if 1 <= idx <= 6:
                        rospy.loginfo("▶ Kapı %d kapatılıyor..." % idx)
                        self.close_gate(idx)
                    else:
                        rospy.logwarn("✗ Geçersiz kapı numarası: %d" % idx)
                
                else:
                    rospy.logwarn("✗ Bilinmeyen komut: %s" % cmd)
            
            except KeyboardInterrupt:
                rospy.loginfo("\n▶ Ctrl+C algılandı, çıkılıyor...")
                self.close_all_gates()
                break
            except Exception as e:
                rospy.logerr("✗ Hata: %s" % str(e))

def show_menu():
    """Ana menüyü göster"""
    print("\n" + "═" * 70)
    print("KAPI TEST ARACI - TEST MODU SEÇİMİ")
    print("═" * 70)
    print("1. Sıralı Test (Önerilen ilk test)")
    print("2. Aynı Anda Test")
    print("3. Sağ/Sol Karşılaştırmalı Test")
    print("4. İnteraktif Test")
    print("5. Çıkış")
    print("═" * 70)

def main():
    try:
        tester = GateTester()
        
        while not rospy.is_shutdown():
            show_menu()
            choice = raw_input("\nSeçim (1-5): ").strip()
            
            if choice == '1':
                tester.test_sequential()
            elif choice == '2':
                tester.test_simultaneous()
            elif choice == '3':
                tester.test_sides()
            elif choice == '4':
                tester.test_interactive()
            elif choice == '5':
                rospy.loginfo("▶ Çıkılıyor...")
                tester.close_all_gates()
                break
            else:
                rospy.logwarn("✗ Geçersiz seçim: %s" % choice)
        
    except KeyboardInterrupt:
        rospy.loginfo("\n▶ Ctrl+C algılandı, çıkılıyor...")
    except Exception as e:
        rospy.logerr("✗ Fatal hata: %s" % str(e))
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
