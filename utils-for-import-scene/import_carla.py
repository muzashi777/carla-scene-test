import unreal
import os
import json

# ================= SETTINGS (Ubuntu & Unreal Engine 5.5) =================
# 1. OS Paths: สำหรับเข้าถึงไฟล์ภายนอกระบบปฏิบัติการ Ubuntu
SOURCE_DIR = "/home/robat/Carla/carla-api-demo/point_cloud_scene/all_ply_files"
CONFIG_PATH = "/home/robat/Carla/carla-api-demo/point_cloud_scene/all_ply_files/transform_config.json"

# 2. Unreal Virtual Paths: อ้างอิงโฟลเดอร์ภายใน Content Browser (ห้ามใส่ .umap)
CARLA_MAPS_DIR_UE = "/Game/Carla/Maps/all_ply_files"
LUMA_ASSETS_DIR_UE = "/Game/Gaussian_Scans/all_scenes"
TEMPLATE_MAP_UE = "/Game/Carla/Maps/all_ply_files/basemap"
# =========================================================================

def batch_import_luma_to_carla():
    # โหลดข้อมูลพรีเซ็ตองศาและพิกัดจาก JSON
    config = {}
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, 'r') as f:
                config = json.load(f)
            print("✅ โหลดไฟล์พรีเซ็ตองศา (JSON Config) สำเร็จ")
        except Exception as e:
            print(f"❌ เกิดข้อผิดพลาดในการอ่าน JSON: {e}")
    else:
        print(f"⚠️ ไม่พบไฟล์ Config ที่ {CONFIG_PATH} (จะใช้ค่าเริ่มต้น แกน 0,0,0 ทุกฉาก)")

    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
    editor_util = unreal.EditorLevelLibrary
    
    # ตรวจสอบว่าโฟลเดอร์ต้นทางบน Ubuntu มีอยู่จริงไหม
    if not os.path.exists(SOURCE_DIR):
        print(f"❌ ไม่พบโฟลเดอร์ต้นทางบนระบบ: {SOURCE_DIR}")
        return
        
    # ค้นหาไฟล์ .ply หรือ .luma (รองรับ Linux case-sensitive)
    files = [f for f in os.listdir(SOURCE_DIR) if f.lower().endswith(('.ply', '.luma'))]
    
    if not files:
        print("❌ ไม่พบไฟล์ .ply หรือ .luma ในโฟลเดอร์ต้นทาง")
        return

    print(f"📦 พบไฟล์ฉากทั้งหมด {len(files)} ไฟล์ กำลังเริ่มกระบวนการอัตโนมัติ...")

    for file_name in files:
        scene_name = os.path.splitext(file_name)[0]
        if scene_name != "scene000":
            print(f"⚠️ ข้ามฉาก {scene_name} (ไม่ใช่ scene000)")
            continue
        
        file_path = os.path.join(SOURCE_DIR, file_name)
        
        # ดึงค่าพิกัด/องศา/สเกล จาก JSON (หากไม่มีไฟล์นั้นใน Config จะใช้ค่า Default)
        scene_cfg = config.get(file_name, {"location": [0,0,0], "rotation": [0,0,0], "scale": [1,1,1]})
        loc = scene_cfg.get("location", [0, 0, 0])
        rot = scene_cfg.get("rotation", [0, 0, 0]) 
        scl = scene_cfg.get("scale", [1, 1, 1])
        
        print(f"\n🎬 [กำลังจัดการฉาก: {scene_name}]")
        
        # 1. สร้าง Map ใหม่โดยการ Duplicate จาก basemap ที่ตั้งค่าระบบ CARLA ไว้แล้ว
        new_map_package = f"{CARLA_MAPS_DIR_UE}/{scene_name}"
        if not unreal.EditorAssetLibrary.does_asset_exist(new_map_package):
            unreal.EditorAssetLibrary.duplicate_asset(TEMPLATE_MAP_UE, new_map_package)
            print(f"   -> คัดลอกเทมเพลตแผนที่สำเร็จ: {new_map_package}")
        
        # 2. โหลดเปิด Map ใหม่ขึ้นมาใน Editor
        editor_util.load_level(new_map_package)
        
        # 3. ตั้งค่า Task สำหรับการ Import ไฟล์ Point Cloud
        # ไฟล์ผลลัพธ์ย่อย 4 ไฟล์ของ Luma จะถูกแยกไปเก็บตามชื่อฉากในโฟลเดอร์ที่ระบุ
        asset_dest_path = f"{LUMA_ASSETS_DIR_UE}/{scene_name}"
        
        import_task = unreal.AssetImportTask()
        import_task.filename = file_path
        import_task.destination_path = asset_dest_path
        import_task.destination_name = scene_name
        import_task.automated = True
        import_task.save = True
        
        # สั่งประมวลผลการ Import
        asset_tools.import_asset_tasks([import_task])
        
        # 4. ค้นหาไฟล์หลัก (Blueprint/Actor) จาก 4 ไฟล์ที่ได้มา เพื่อเอามาวางในแมป
        imported_objs = import_task.get_objects()
        main_luma_asset = None
        
        for obj in imported_objs:
            if isinstance(obj, unreal.Blueprint) or "Luma" in obj.get_class().get_name():
                main_luma_asset = obj
                break
        
        # กรณีคลาสไม่ตรง ให้ดึงไฟล์แรกสุดมาใช้แก้ขัด
        if not main_luma_asset and imported_objs:
            main_luma_asset = imported_objs[0]
            
        # 5. วาง Luma Actor ลงในแมปปัจจุบันตามองศาและพิกัดที่ตั้งค่าไว้
        if main_luma_asset:
            spawn_loc = unreal.Vector(loc[0], loc[1], loc[2])
            spawn_rot = unreal.Rotator(rot[0], rot[1], rot[2]) # [Pitch, Roll, Yaw]
            
            try:
                spawned_actor = editor_util.spawn_actor_from_object(main_luma_asset, spawn_loc, spawn_rot)
                if spawned_actor:
                    spawned_actor.set_actor_scale3d(unreal.Vector(scl[0], scl[1], scl[2]))
                    print(f"   ✅ วาง Luma Actor สำเร็จ (องศา: {rot})")
            except Exception as e:
                print(f"   ⚠️ ไม่สามารถวาง Actor ลงในฉากได้: {e}")
        else:
            print("   ❌ เกิดข้อผิดพลาด: ไม่พบ Luma Asset หลักจากการ Import")
                
        # 6. บันทึกฉากที่ทำเสร็จแล้วลงระบบ
        editor_util.save_current_level()
        print(f"   💾 บันทึกแผนที่ {scene_name} เรียบร้อย")

    print("\n🎉 [เสร็จสมบูรณ์] แปลงไฟล์ Point Cloud ทุกฉากเข้าสู่ CARLA เรียบร้อยแล้ว!")

# เรียกใช้งานฟังก์ชัน
batch_import_luma_to_carla()