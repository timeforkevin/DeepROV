from flask import Flask, render_template, Response, request
from camera import VideoCamera

app = Flask(__name__)

@app.route('/')
def index():
    return render_template('index.html')

def gen_frame(camera, analyze):
    while True:
        if analyze:
            frame = camera.analyze_frame()
        else:
            frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')

@app.route('/video_feed')
def video_feed():
    analyze = request.args.get('analyze')
    if analyze is None:
        analyze = False
    else:
        analyze = int(analyze)
    return Response(gen_frame(VideoCamera(), analyze),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
