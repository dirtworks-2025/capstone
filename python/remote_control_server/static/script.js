const socket = new WebSocket("ws://" + location.host + "/ws");
const pc = new RTCPeerConnection();

pc.ontrack = (event) => {
    document.getElementById("remoteVideo").srcObject = event.streams[0];
};

socket.onmessage = async (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "answer") {
        console.log("Received answer");
        await pc.setRemoteDescription(new RTCSessionDescription(message));
    }
};

async function startWebRTC() {
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    socket.send(JSON.stringify({ type: "offer", sdp: offer.sdp }));
}

socket.onopen = startWebRTC;