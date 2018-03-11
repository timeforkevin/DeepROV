from flask import Flask, render_template, Response, request
from camera import VideoCamera

app = Flask(__name__)

def gen_frame(camera, analyze):
    while True:
        if analyze:
            frame = camera.analyze_frame()
        else:
            frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed_camera0')
def video_feed_camera0():
    analyze = request.args.get('analyze')
    if analyze is None:
        analyze = False
    else:
        analyze = int(analyze)
    return Response(gen_frame(VideoCamera(0), analyze),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/video_feed_camera1')
def video_feed_camera1():
    analyze = request.args.get('analyze')
    if analyze is None:
        analyze = False
    else:
        analyze = int(analyze)
    return Response(gen_frame(VideoCamera(1), analyze),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
