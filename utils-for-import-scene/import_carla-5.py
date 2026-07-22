import unreal
import os
import json

# ================= SETTINGS (Ubuntu & Unreal Engine 5.5) =================
SOURCE_DIR = "/home/robat/Carla/carla-api-demo/point_cloud_scene/all_ply_files"
CONFIG_PATH = "/home/robat/Carla/carla-api-demo/point_cloud_scene/all_ply_files/transform_config.json"
LUMA_ASSETS_DIR_UE = "/Game/Gaussian_Scans/all_scenes"

# ระยะห่างระหว่างแต่ละฉากในโลกจำลอง (หน่วยเป็นเซนติเมตร: 200000 = 2,000 เมตร)
SCENE_SPACING = 200000 
# =========================================================================

def find_luma_blueprint(scene_name):
    """ ค้นหาไฟล์ Blueprint หลักในโฟลเดอร์ย่อยของ Luma """
    search_path = f"{LUMA_ASSETS_DIR_UE}/{scene_name}"
    assets = unreal.EditorAssetLibrary.list_assets(search_path, recursive=True)
    for asset_path in assets:
        obj = unreal.EditorAssetLibrary.load_asset(asset_path)
        if obj and isinstance(obj, unreal.Blueprint):
            if not any(x in obj.get_name() for x in ["_Baked", "_Crop"]):
                return obj
    return None

def build_master_luma_map():
    # โหลด JSON
    config = {}
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, 'r') as f:
                config = json.load(f)
        except Exception as e:
            print(f"❌ อ่าน JSON ผิดพลาด: {e}")
            return

    editor_util = unreal.EditorLevelLibrary
    
    files = [f for f in os.listdir(SOURCE_DIR) if f.lower().endswith(('.ply', '.luma'))]
    if not files:
        print("❌ ไม่พบไฟล์ในโฟลเดอร์ต้นทางเพื่ออ้างอิง")
        return

    print(f"🎬 กำลังเริ่มวางฉากทั้งหมดลงในแมปปัจจุบัน (จำนวน {len(files)} ฉาก)...")

    # ตัวแปรสำหรับขยับพิกัดฉากไม่ให้ซับซ้อนกัน
    current_offset_x = 0

    for file_name in files:
        scene_name = os.path.splitext(file_name)[0]
        scene_cfg = config.get(file_name, {"location": [0,0,0], "rotation": [0,0,0], "scale": [1,1,1]})
        
        # ดึงค่าพรีเซ็ต
        rot = scene_cfg.get("rotation", [0, 0, 0]) 
        scl = scene_cfg.get("scale", [1, 1, 1])
        
        # ค้นหา Blueprint ของ Luma
        main_blueprint = find_luma_blueprint(scene_name)
        if not main_blueprint:
            print(f"   ❌ ไม่พบ Blueprint ของ {scene_name} (ข้าม)")
            continue
            
        # คำนวณพิกัดใหม่ โดยเรียงต่อกันไปเรื่อยๆ บนแกน X
        # ฉากแรกอยู่ที่ X=0, ฉากสอง X=200000, ฉากสาม X=400000
        spawn_loc = unreal.Vector(current_offset_x, 0, 0)
        spawn_rot = unreal.Rotator(rot[0], rot[1], rot[2]) # ตั้งองศาตาม JSON
        
        try:
            spawned_actor = editor_util.spawn_actor_from_object(main_blueprint, spawn_loc, spawn_rot)
            if spawned_actor:
                spawned_actor.set_actor_scale3d(unreal.Vector(scl[0], scl[1], scl[2]))
                # ตั้งชื่อ Actor ในโหมด Editor ให้ดูง่าย
                spawned_actor.set_actor_label(f"Luma_{scene_name}")
                
                print(f"   ✅ วางฉาก {scene_name} สำเร็จที่พิกัด X: {current_offset_x} | องศา Yaw: {rot[2]}")
                
                # บันทึก Log ค่าพิกัดเก็บไว้ เพื่อให้คุณรู้ว่าฉากไหนอยู่ตรงไหน
                print(f"      📌 [พิกัดสำหรับ CARLA -> X: {current_offset_x / 100.0}, Y: 0.0, Z: 0.0]")
        except Exception as e:
            print(f"   ⚠️ เกิดข้อผิดพลาดขณะวางวัตถุ: {e}")
            
        # ขยับพิกัดสำหรับฉากถัดไป
        current_offset_x += SCENE_SPACING

    # สั่งบันทึกแมป
    unreal.get_editor_subsystem(unreal.LevelEditorSubsystem).save_current_level()
    print("\n🎉 [เสร็จสมบูรณ์] วางทุกฉากพร้อมปรับองศาลงในแมปนี้เรียบร้อยแล้ว! ไม่พบอาการแครช")

build_master_luma_map()