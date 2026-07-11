# -*- coding: utf-8 -*-
"""
สมองกล enhanced_inflation — Required-Deceleration + Threshold Inflation ชดเชย latency
----------------------------------------------------------------------
ต่อยอดจาก proposed_enhanced (required-decel) แต่ *ไม่พยากรณ์เป้า* — แค่ "เติมระยะ
ที่รถวิ่งไปช่วง latency L" เข้าไปในระยะหยุดที่ต้องใช้ แล้วเบรกเผื่อไว้ (worst-case robust)

  r_required = v_close² / (2·μ·g)     ระยะหยุดตามฟิสิกส์ (มองสิ่งกีดขวางนิ่ง, ใช้ closing speed)
  r_required += v_close · L            + ระยะที่วิ่งต่อช่วง perception latency (worst-case margin)
  urgency     = r_required / (gap − r_safe)   ≥1 = ระยะเหลือไม่พอ → map เข้าเกณฑ์ full/partial เดิม

ข้อได้เปรียบ: ไม่ต้องรู้ L เป๊ะ — ใช้แค่ขอบบน L_max ก็ออกแบบเผื่อได้
อ้างอิง: พจน์ delay (v_close·L) เป็นส่วนมาตรฐานของสูตร safety-distance ตระกูล Mazda/
         Berkeley-PATH ที่ Rajamani (2012, *Vehicle Dynamics and Control*) ครอบคลุม
L=0 → พจน์ inflation หาย → ลดรูปเป็น required-decel แบบไม่ชดเชย (สิ่งกีดขวางนิ่ง)
* ไม่แตะ baseline/proposed/proposed_enhanced — เพิ่มตัวนี้เพื่อเทียบการกู้ Rc *
"""
import math
from control.base_controller import BaseController, register, compensation_latency
from core.actors import REQ_GAP_EPS


@register("enhanced_inflation")
class EnhancedInflation(BaseController):
    def __init__(self, cfg):
        self.cfg = cfg
        self.partial = cfg.PARTIAL_BRAKE
        self.req_full = getattr(cfg, "REQ_FULL_FRAC", 0.9)   # urgency ≥ ค่านี้ → เบรกเต็ม
        self.req_warn = getattr(cfg, "REQ_WARN_FRAC", 0.6)   # urgency ≥ ค่านี้ → เบรกบางส่วน
        self.r_safe = getattr(cfg, "COMP_R_SAFE", 0.0)       # ระยะเผื่อยืนหยุด (default 0 = แตะกันชน)
        self.g0 = 9.81
        self.L = 0.0                 # วินาทีที่ชดเชย (ตั้งใน reset จาก run_spec)
        self.last_r_required = 0.0   # ไว้ดีบัก
        self.reset()

    def reset(self):
        # อ่านค่า L จาก run_spec ที่ make_controller ผูกไว้ (ไม่มี → L=0 = ไม่ชดเชย)
        self.L, self.comp_frames, self.comp_source = compensation_latency(
            getattr(self, "run_spec", {}), self.cfg)
        super().reset()

    def _desired(self, perc, ego):
        a_max = max(0.0, ego.mu) * self.g0
        # จวนชน (gap ≤ eps) → เบรกเต็มทันที (เกตเดียวกับ proposed_enhanced)
        if perc.distance <= REQ_GAP_EPS:
            self.last_r_required = 0.0
            return 1.0
        v_close = max(0.0, perc.rel_speed)          # closing speed (>0 = gap หด)
        avail = perc.distance - self.r_safe         # ระยะที่เหลือให้เบรกจริง
        if a_max <= 1e-6:
            self.last_r_required = math.inf
            return 1.0 if v_close > 0.0 else 0.0
        # ระยะหยุดที่ต้องใช้ = ระยะฟิสิกส์ + ระยะที่วิ่งต่อช่วง latency
        r_required = (v_close * v_close) / (2.0 * a_max) + v_close * self.L
        self.last_r_required = r_required
        if avail <= 1e-6:
            return 1.0 if r_required > 0.0 else 0.0
        urgency = r_required / avail
        if urgency >= self.req_full:
            return 1.0
        if urgency >= self.req_warn:
            return self.partial
        return 0.0

    def decide(self, perc, ego):
        # เกตเดียวกับ proposed_enhanced: ตัดสินใจเมื่อ 'ตรวจเจอ' หรือ latch เบรกไปแล้ว
        desired = self._desired(perc, ego) if (perc.detected or self._engaged) else 0.0
        return self._emit(desired)
