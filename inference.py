from ultralytics import YOLO
import cv2

# Carregando um modelo personalizado
model = YOLO('ml_model/best.pt')

# Lendo a imagem
image = cv2.imread('/home/junior/Documentos/EDUCA_DRONES/automatic_flight_drone/captured_images_tests_2/analog/alt_9_Image_3_2024-05-30_16-16-03.jpg')

# Realizando a predição
results = model.predict(image, imgsz=640, conf=0.70, iou=0.45)
results = results[0]

# Processando os resultados
for i in range(len(results.boxes)):
    box = results.boxes[i]
    tensor = box.xyxy[0]
    x1 = int(tensor[0].item())
    y1 = int(tensor[1].item())
    x2 = int(tensor[2].item())
    y2 = int(tensor[3].item())
    
    # Desenhando a caixa delimitadora
    cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 3)
    
    # Obtendo a taxa de confiança
    confidence = box.conf[0].item()
    label = f'{confidence:.2f}'
    
    # Colocando a taxa de confiança próxima à caixa
    (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
    cv2.rectangle(image, (x1, y1 - h - 10), (x1 + w, y1), (255, 0, 0), -1)
    cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

# Salvando a imagem resultante
cv2.imwrite('saves/test.jpg', image)

print(len(results.boxes))
