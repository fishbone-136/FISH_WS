"""Isolated YOLO inference; no ROS, camera ownership or motor access."""
import base64
import contextlib
import json
import os
import sys
import time

import cv2
import numpy as np


def decode_people(output, width, height, scale, pad):
    rows = np.asarray(output).squeeze()
    if rows.ndim != 2 or rows.shape[0] != 84 or not np.isfinite(rows).all():
        raise ValueError('不支持的人物模型输出')
    rows = rows.T
    scores = rows[:, 4:]
    mask = (scores.argmax(axis=1) == 0) & (scores[:, 0] >= .55)
    rows = rows[mask]
    boxes, confidences = [], []
    for row in rows:
        cx, cy, w, h = row[:4]
        x, y = (cx-w/2-pad[0])/scale, (cy-h/2-pad[1])/scale
        boxes.append([float(x), float(y), float(w/scale), float(h/scale)])
        confidences.append(float(row[4]))
    indices = cv2.dnn.NMSBoxes(boxes, confidences, .55, .45)
    result = []
    for index in np.asarray(indices).flatten():
        x, y, w, h = boxes[int(index)]
        box = [max(0., x/width), max(0., y/height), min(1., (x+w)/width), min(1., (y+h)/height)]
        if box[2]-box[0] >= .035 and box[3]-box[1] >= .1:
            result.append(box)
    return result


def main():
    cv2.setNumThreads(2)
    model_path = sys.argv[1]
    with contextlib.redirect_stdout(sys.stderr):
        import torch
        if not torch.cuda.is_available():
            raise RuntimeError('人物快速检测需要可用的 CUDA')
        from ultralytics import YOLO
        model = YOLO(model_path).model.eval().to('cuda').half()
        with torch.inference_mode():
            for _ in range(2):
                model(torch.zeros((1, 3, 640, 640), device='cuda', dtype=torch.float16))
        torch.cuda.synchronize()
    print(json.dumps({'ready': True, 'backend': 'YOLOv8n CUDA FP16'}), flush=True)
    for line in sys.stdin:
        try:
            payload = json.loads(line)
            image = cv2.imdecode(np.frombuffer(base64.b64decode(payload['image']), np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('无效图像')
            started = time.monotonic()
            height, width = image.shape[:2]
            scale = min(640/width, 640/height)
            w, h = round(width*scale), round(height*scale)
            left, top = (640-w)//2, (640-h)//2
            padded = np.full((640, 640, 3), 114, np.uint8)
            padded[top:top+h, left:left+w] = cv2.resize(image, (w, h))
            tensor = torch.from_numpy(np.ascontiguousarray(padded[:, :, ::-1].transpose(2, 0, 1))).to('cuda').half().unsqueeze(0)/255.
            with torch.inference_mode():
                output = model(tensor)[0].float().cpu().numpy()
            boxes = decode_people(output, width, height, scale, (left, top))
            print(json.dumps({'boxes': boxes, 'seconds': time.monotonic()-started}), flush=True)
        except Exception as error:
            print(json.dumps({'error': str(error)}), flush=True)


if __name__ == '__main__':
    main()
