import unreal
import os

# ================= SETTINGS =================
SOURCE_DIR = "/home/robat/Carla/carla-api-demo/point_cloud_scene/all_ply_files"
LUMA_ASSETS_DIR_UE = "/Game/Gaussian_Scans/all_scenes"
# ============================================

def batch_import_assets_only():
    asset_tools = unreal.AssetToolsHelpers.get_asset_tools()
    
    if not os.path.exists(SOURCE_DIR):
        print(f"❌ ไม่พบโฟลเดอร์ต้นทาง: {SOURCE_DIR}")
        return
        
    files = [f for f in os.listdir(SOURCE_DIR) if f.lower().endswith(('.ply', '.luma'))]
    if not files:
        print("❌ ไม่พบไฟล์โมเดลในโฟลเดอร์")
        return

    print(f"📦 พบทั้งหมด {len(files)} ไฟล์ เริ่มทำการ Import เข้า Content Browser (ไม่มีการสลับแมป)...")

    for file_name in files:
        scene_name = os.path.splitext(file_name)[0]
        
        # if scene_name != "scene001":
        #     print(f"⚠️ ข้ามฉาก {scene_name} (ไม่ใช่ scene000)")
        #     continue

        file_path = os.path.join(SOURCE_DIR, file_name)
        asset_dest_path = f"{LUMA_ASSETS_DIR_UE}/{scene_name}"
        
        # ป้องกันการ Import ซ้ำถ้ามีโฟลเดอร์นี้อยู่แล้ว
        if unreal.EditorAssetLibrary.does_directory_exist(asset_dest_path):
            print(f"⏩ ข้าม {scene_name} (มีแอสเซทอยู่แล้ว)")
            continue
            
        import_task = unreal.AssetImportTask()
        import_task.filename = file_path
        import_task.destination_path = asset_dest_path
        import_task.destination_name = scene_name
        import_task.automated = True
        import_task.save = True
        
        print(f"📥 กำลัง Import: {scene_name}")
        asset_tools.import_asset_tasks([import_task])
        
    print("🎉 [เสร็จสิ้น] นำเข้าไฟล์พอยต์คลาวด์ทั้งหมดเป็น Asset เรียบร้อยโดยไม่แครช!")

batch_import_assets_only()