from ultralytics import YOLO 
import cv2
import pandas as pd
import numpy as np

class RedballTracker:
    def __init__(self,model_path):
        self.model = YOLO(model_path)
    
    def detect_frames(self,frames):
        ball_detections = []

        for frame in frames:
            ball_dict = self.detect_frame(frame)
            ball_detections.append(ball_dict)
        ball_detections_array = np.array(ball_detections)
        return ball_detections_array
        # return ball_detections

    def detect_frame(self,frame):
        try:
            results = self.model.track(frame, conf=0.2)[0]
        except Exception as e:
            print(f"Error in model tracking: {e}")
            return {}
        id_name_dict = results.names
        ball_dict = {}

        for box in results.boxes:
            # Ensure box.id is not None and has the necessary structure
            if box.id is None or not box.id.tolist():
                print("box.id is None or empty.")
                continue
            try:
                track_id = int(box.id.tolist()[0])
            except (AttributeError, IndexError, ValueError) as e:
                print(f"Error processing box.id: {e}")
                continue
            # Ensure box.xyxy is not None and has the necessary structure
            if box.xyxy is None or not box.xyxy.tolist():
                print("box.xyxy is None or empty.")
                continue
            try:
                result = box.xyxy.tolist()[0]
            except (AttributeError, IndexError) as e:
                print(f"Error processing box.xyxy: {e}")
                continue
            # Ensure box.cls is not None and has the necessary structure
            if box.cls is None or not box.cls.tolist():
                print("box.cls is None or empty.")
                continue
            try:
                object_cls_id = box.cls.tolist()[0]
                object_cls_name = id_name_dict[object_cls_id]
            except (AttributeError, IndexError, KeyError) as e:
                print(f"Error processing box.cls or id_name_dict: {e}")
                continue
            # Check for the specific object class name
            #if object_cls_name == "redball":
            if object_cls_name == "blueball":
                ball_dict[track_id] = result
        
        return ball_dict

    # def draw_bboxes(self,video_frames, ball_detections):
    #     output_video_frames = []
    #     for frame, ball_dict in zip(video_frames, ball_detections):
    #         # Draw Bounding Boxes
    #         for track_id, bbox in ball_dict.items():
    #             x1, y1, x2, y2 = bbox
    #             cv2.putText(frame, f"blueball ID: {track_id}",(int(bbox[0]),int(bbox[1] -10 )),cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    #             cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
    #         output_video_frames.append(frame)
        
    #     return output_video_frames


    def draw_bboxes(self, video_frames, ball_detections):

        # Check if ball_detections is None or empty
        if ball_detections is None or not ball_detections:
            return video_frames
        
        # print("ball_detections:", ball_detections)  # Debugging statement
        
        # Convert coordinates to integers
        ball_detections = [int(coord) for coord in ball_detections]
        
        # print("After converting to integers:", ball_detections)  # Debugging statement
        
        # Ensure ball_detections contains at least four elements
        if len(ball_detections) < 4:
            raise ValueError("ball_detections should contain at least four coordinates")
        
        # Draw bounding box
        output_video_frames = video_frames.copy()  # Create a copy to avoid modifying the original frame
        cv2.rectangle(output_video_frames, (ball_detections[0], ball_detections[1]), 
                    (ball_detections[2], ball_detections[3]), (0, 255, 0), 2)
        
        return output_video_frames

    
