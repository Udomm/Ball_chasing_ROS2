from ultralytics import YOLO
from .red_ball_tracker import RedballTracker

model = YOLO('models/best.pt')

def redball_tracking():
    # Read Video
    input_video_path = "videoplayback.mp4"
    video_frames = read_video(input_video_path)

    redball_tracker = RedballTracker(model_path='/home/udom/ball_detection/models/best.pt')
    

    ball_detections = redball_tracker.detect_frames(video_frames,
                                                    read_from_stub=True,
                                                    stub_path="tracker_stubs/ball_detections.pkl"
                                                    )
    ball_detections = redball_tracker.interpolate_ball_positions(ball_detections)
    
    # Draw output
    ## Draw Player Bounding Boxes
    #output_video_frames= redball_tracker.draw_bboxes(video_frames, ball_detections)
    #save_video(output_video_frames, "output_videos/output_video.avi")
    #print("output", ball_detections)
    return ball_detections