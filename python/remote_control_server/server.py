from fastapi import FastAPI, WebSocket
from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.requests import Request
import json
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

app = FastAPI()
pcs = set()

from aiortc.contrib.media import MediaStreamTrack
import cv2
import av

class WebcamVideoStreamTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open video source")

    async def recv(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Error: Failed to capture frame.")
            return None
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        av_frame = av.VideoFrame.from_ndarray(frame, format="rgb24")
        av_frame.pts = None  # Avoid timestamp issues
        return av_frame


# Mount the "static" folder for serving JS and other static files
app.mount("/static", StaticFiles(directory="static"), name="static")

# Set up Jinja2 templates
templates = Jinja2Templates(directory="templates")

@app.get("/")
def serve_home(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    pc = RTCPeerConnection()
    pcs.add(pc)

    track = WebcamVideoStreamTrack()
    print("Created video track:", track)
    pc.addTransceiver("video", direction="sendonly")
    pc.addTrack(track)


    while True:
        data = await websocket.receive_text()
        message = json.loads(data)

        if message["type"] == "offer":
            offer = RTCSessionDescription(sdp=message["sdp"], type=message["type"])
            print("Received offer:", offer.sdp)
            await pc.setRemoteDescription(offer)
            answer = await pc.createAnswer()
            print("Created answer:", answer.sdp)
            await pc.setLocalDescription(answer)
            print("Set local description")
            await websocket.send_text(json.dumps({"type": "answer", "sdp": pc.localDescription.sdp}))
            print("Sent answer")