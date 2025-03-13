import asyncio
import json
import websockets
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRecorder

pcs = set()  # Stores peer connections

async def signaling(websocket, path):
    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("icecandidate")
    async def on_icecandidate(candidate):
        if candidate:
            await websocket.send(json.dumps({"type": "candidate", "candidate": candidate.to_dict()}))

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"Connection state: {pc.connectionState}")
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    async for message in websocket:
        data = json.loads(message)

        if data["type"] == "offer":
            offer = RTCSessionDescription(sdp=data["sdp"], type=data["type"])
            await pc.setRemoteDescription(offer)

            # Add a 
            pc.addTrack(player.video)

            # Create an answer
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)

            await websocket.send(json.dumps({
                "type": "answer",
                "sdp": pc.localDescription.sdp
            }))

        elif data["type"] == "candidate":
            candidate = data["candidate"]
            await pc.addIceCandidate(candidate)

# Run WebSocket signaling server
async def main():
    async with websockets.serve(signaling, "0.0.0.0", 8765):
        await asyncio.Future()  # Run forever

asyncio.run(main())
