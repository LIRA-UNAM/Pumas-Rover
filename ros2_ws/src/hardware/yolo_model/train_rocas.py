from ultralytics import YOLO

def main():
    # 1. Cargamos el modelo YOLOv8 Nano (el más rápido para robótica)
    model = YOLO('yolov8n.pt')

    # 2. Entrenamos usando la GPU (device=0)
    # Si tienes problemas de memoria, baja el batch a 8 o 16
    results = model.train(
        data='rocas.yaml',
        epochs=100,        # Puedes subirlo a 200 si quieres más precisión
        imgsz=640,
        batch=16,
        device=0,          # USA TU RTX 4050
        project='runs_rocas',
        name='experimento_1'
    )

if __name__ == '__main__':
    main()