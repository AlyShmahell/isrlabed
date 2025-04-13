import cv2 
import json 
import math
import numpy as np 
from ultralytics import YOLO


model = YOLO("yolo11n-seg.pt")

 

class processor:
    @staticmethod
    def draw(frame, xys, colors, alpha, title):
        overlay = frame.copy()
        output  = frame.copy()
        for xy, color in zip(xys, colors.values()):
            print(color)
            cv2.fillPoly(overlay, pts=[np.array(xy)], color = color)
        cv2.addWeighted(overlay, alpha, output, 1 - alpha, 0, output)
        cv2.imshow(title, output)
        cv2.waitKey(1)
    @staticmethod
    def cmap(*, colormap=cv2.COLORMAP_JET, num_colors=256):
        gradient = np.linspace(0, 255, num_colors).astype(np.uint8).reshape(-1, 1) 
        return [
            tuple(map(int, color)) 
            for color in cv2.applyColorMap(gradient, colormap)[:, 0, ::-1]
        ]
    @staticmethod
    def cam(idx=0, alpha=.4, title=''):
        vc = cv2.VideoCapture(idx)
        colors = {}
        while vc.isOpened():
            _, frame = vc.read()
            if frame is None:
                break
            results = model(frame)
            if not results:
                continue 
            results = [
                json.loads(
                    results[idx].to_json()
                )
                for idx in range(len(results))
            ]
            names = [
                seg[0].get('name')
                for seg in results
                if seg 
            ]
            if set(names) != set(colors.keys()):
                cmap = processor.cmap(num_colors=len(set(names) | set(colors.keys())))
                colors = {
                    name: color
                    for name, color in zip(set(names) | set(colors.keys()), cmap)
                }
            segs = [
                seg[0].get('segments', {})
                for seg in results
                if seg 
            ]
            xys = [
                [
                    [int(x) for x in pnt]
                    for pnt in zip(seg.get('x', []), seg.get('y', []))
                ]
                for seg in segs
            ]
            processor.draw(frame, xys, colors, alpha, title)
            
if __name__ == '__main__':
    processor.cam(2)