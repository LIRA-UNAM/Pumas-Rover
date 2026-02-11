import os
import json
import shutil
import random


source_folders = ["raw_img"] 
output_dir = "Dataset_Rocas_Final"
train_ratio = 0.8  


class_map = {
    "red_stone": 0,
    "blue_stone": 1,
    "green_stone": 2
}

# ----------------------

def convert_to_yolo(size, box):
    # Matemáticas de normalización YOLO
    # dw = 1/ancho, dh = 1/alto
    dw = 1. / size[0]
    dh = 1. / size[1]
    x = (box[0] + box[1]) / 2.0
    y = (box[2] + box[3]) / 2.0
    w = box[1] - box[0]
    h = box[3] - box[2]
    return (x * dw, y * dh, w * dw, h * dh)

def find_image(base_name, folder):
    extensions = [".jpg", ".png", ".jpeg", ".JPG", ".PNG"]
    for ext in extensions:
        full_path = os.path.join(folder, base_name + ext)
        if os.path.exists(full_path):
            return full_path
    return None

def process_folders():
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    
    for split in ['train', 'val']:
        os.makedirs(os.path.join(output_dir, 'images', split), exist_ok=True)
        os.makedirs(os.path.join(output_dir, 'labels', split), exist_ok=True)

    print(f"Iniciando conversión para {len(source_folders)} carpetas...")
    total_images = 0

    for folder in source_folders:
        if not os.path.exists(folder):
            print(f"Error: No existe {folder}")
            continue

        json_files = [f for f in os.listdir(folder) if f.endswith('.json')]
        
        for json_file in json_files:
            json_path = os.path.join(folder, json_file)
            base_name = os.path.splitext(json_file)[0]
            image_path = find_image(base_name, folder)
            
            if not image_path: continue

            with open(json_path, 'r') as f:
                data = json.load(f)
            
            im_w, im_h = data['imageWidth'], data['imageHeight']
            yolo_lines = []
            
            for shape in data['shapes']:
                label = shape['label']
                if label in class_map:
                    class_id = class_map[label]
                    points = shape['points']
                    x_coords = [p[0] for p in points]
                    y_coords = [p[1] for p in points]
                    box = (min(x_coords), max(x_coords), min(y_coords), max(y_coords))
                    bb = convert_to_yolo((im_w, im_h), box)
                    yolo_lines.append(f"{class_id} {bb[0]} {bb[1]} {bb[2]} {bb[3]}")
            
            if yolo_lines:
                split = 'train' if random.random() < train_ratio else 'val'
                img_ext = os.path.splitext(image_path)[1]
                shutil.copy(image_path, os.path.join(output_dir, 'images', split, base_name + img_ext))
                
                with open(os.path.join(output_dir, 'labels', split, base_name + ".txt"), 'w') as out_f:
                    out_f.write('\n'.join(yolo_lines))
                total_images += 1

    print(f"¡Listo! {total_images} imágenes procesadas en {output_dir}")

if __name__ == '__main__':
    process_folders()