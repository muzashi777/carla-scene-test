mkdir -p all_ply_files

find . -path "*/viewer/*" -name "point_cloud.ply" | while read -r file; do
    dir_name=$(echo "$file" | cut -d'/' -f2)

    scene_num=$(echo "$dir_name" | grep -oE '[0-9]+$')
    
    cp "$file" "all_ply_files/scene${scene_num}.ply"
done

py "/home/robat/Carla/carla-scene-test/import_carla-1.py"
/home/robat/Carla/carla-scene-test/import_carla-2.py